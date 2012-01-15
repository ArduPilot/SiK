// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2011 Andrew Tridgell, All Rights Reserved
// Copyright (c) 2011 Michael Smith, All Rights Reserved
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  o Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  o Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.
//

///
/// @file	tdm.c
///
/// time division multiplexing code
///

#include <stdarg.h>
#include "radio.h"
#include "tdm.h"
#include "freq_hopping.h"

// base tick counter. All of the TDM window calculations
// are based on this. It runs at 200Hz
static volatile uint8_t tick_counter;

// how many ticks are remaining in our transmit window
// this will be zero when we are receiving. We send this value in
// header byte 3 of every packet
static volatile uint8_t tx_window_remaining;

// the tick value of the start of our next transmit window. This is
// adjusted based on the header of any incoming packet, to keep the
// two radios in sync
static volatile uint8_t next_tx_window;

// the number of ticks to the other radios transmit window closing
static volatile uint8_t other_window_remaining;

// the number of ticks we grab for each transmit window
// this is enough to hold at least 3 packets and is based 
// on the configured air data rate
static uint8_t tx_window_width;

// the silence period between transmit windows. This is calculated as
// the number of ticks it would take to transmit a full sized packet
static uint8_t silence_period;

// activity indication. When tick_counter wraps we 
// check if we have received a packet in the last 1.25seconds. If we
// have the green radio LED is held on. Otherwise it blinks every 1.25
// seconds. The received_packet flag is set for any received packet,
// whether it contains user data or not
static __bit blink_state;
static volatile __bit received_packet;

// we prefer to send packets in tx_chunk_size byte chunks when 
// possible. This may be adjusted based on the air data rate
// to keep the TDM round time below 256
// we start at 63 to allow for 1 control byte
static uint8_t tx_chunk_size = 63;

// at startup we calculate how many milliticks a byte is expected
// to take to transmit with the configured air data rate. This is used
// to calculate the flight time of an incoming packet
static uint16_t milliticks_per_byte;

// how many bytes can we safely transmit in a single tick?
static uint8_t bytes_per_tick;

// number of ticks to wait for a preamble to turn into a packet. This
// is set when we get a preamble interrupt, and causes us to delay
// sending for the silence_period. This is used to make it more likely
// that two radios that happen to be exactly in sync in their sends
// will eventually get a packet through and get their transmit windows
// sorted out
static volatile uint8_t preamble_wait;

// the overhead in bytes of a packet. It consists of 5
// preamble bytes, 2 sync bytes, 2 header bytes, 2 CRC bytes and 1
// control byte
#define PACKET_OVERHEAD 12


/*
  synchronise tx windows

  we receive a 8 bit header with each packet which indicates how many
  more ticks the sender has in their transmit window

  The job of this function is to adjust our own transmit window to
  match the other radio
 */
static void sync_tx_windows(uint8_t rxheader, uint8_t packet_length)
__critical {
	uint8_t flight_time = (512 + ((packet_length + PACKET_OVERHEAD) * milliticks_per_byte))>>10;
	uint8_t old_tx_window = next_tx_window;

	if (rxheader > tx_window_width) {
		// the other radio has more ticks than is usually
		// allowed, so must be using yielded ticks from us. To
		// prevent a storm of yields we just return now
		debug("RXHEADER %d tww=%d\n", 
		       (int)rxheader,
		       (int)tx_window_width);
		return;
	} else if (rxheader >= flight_time) {
		// we are still in the other radios transmit
		// window. We can adjust our transmit window
		next_tx_window = tick_counter + (rxheader - flight_time) + silence_period;
		if (tx_window_remaining > 0) {
			tx_window_remaining = 0;
			fhop_window_change();
		}
		if (rxheader == flight_time) {
			// the other radios window has closed
			if (other_window_remaining > 0) {
				other_window_remaining = 0;
				fhop_window_change();
			}
		}
	} else if (flight_time - rxheader < silence_period) {
		// we're in the silence period between windows. Adjust
		// the transmit window, but don't start transmitting
		// just yet
		next_tx_window = tick_counter + silence_period - (flight_time - rxheader);
		if (tx_window_remaining > 0) {
			tx_window_remaining = 0;
			fhop_window_change();
		}
		if (other_window_remaining > 0) {
			other_window_remaining = 0;
			fhop_window_change();
		}
	} else {
		// we are in our transmit window. 
		tx_window_remaining = tx_window_width - (flight_time - rxheader);
		next_tx_window = tick_counter + tx_window_remaining + tx_window_width + silence_period*2;
	}

#if 0
	{
	uint8_t window_change;
	window_change = old_tx_window - next_tx_window;
	if (window_change > 1 && window_change != 255) {
		printf("otx=%d ntx=%d rx=%d ft=%d pl=%d bpt=%d\n",
		       (int)old_tx_window,
		       (int)next_tx_window,
		       (int)rxheader,
		       (int)flight_time,
		       (int)packet_length,
		       (int)bytes_per_tick);
	}
	}
#endif

#if 1
	// if the other end has sent a zero length packet and we don't
	// currently have any transmit window remaining then they are
	// yielding some ticks to us. 
	if (packet_length == 0 && tx_window_remaining == 0) {
		tx_window_remaining = (next_tx_window - tick_counter) + tx_window_width;
	}
#endif
}


// the main loop for the TDM based 
// transparent serial implementation
void tdm_serial_loop(void)
{
	// the number of bytes currently in the send fifo
	uint8_t tx_fifo_bytes = 0;
	// a tick count when we will send a short packet
	uint8_t force_send_time = 0;
	bool yielded_window = false;

	for (;;) {
		__pdata uint8_t	rlen;
		__xdata uint8_t	rbuf[64];
		uint16_t slen;
		uint8_t rxheader;
		uint8_t current_window;

		// give the AT command processor a chance to handle a command
		at_command();

		// set right receive channel
		radio_set_channel(fhop_receive_channel());

		// see if we have received a packet
		if (radio_receive_packet(&rlen, rbuf)) {
			// update the activity indication
			__critical {
				received_packet = 1;
			}
			fhop_set_locked(true);

			// we're not waiting for a preamble
			// any more
			preamble_wait = 0;

			if (rlen == 0) {
				// not a valid packet. We always send
				// a control byte at the end of every packet
				continue;
			}

			// extract control byte from end of packet
			rxheader = rbuf[rlen-1];
			rlen--;

			if (rxheader == 0 && rlen != 0) {
				// its a control packet for the
				// frequency hopping system
				//fhop_control_packet(rlen, rbuf);
			} else {
				// sync our transmit windows based on
				// received header
				sync_tx_windows(rxheader, rlen);

				if (rlen != 0) {
					// its user data - send it out
					// the serial port
					//printf("rcv(%d,[", rlen);
					LED_ACTIVITY = LED_ON;
					serial_write_buf(rbuf, rlen);
					LED_ACTIVITY = LED_OFF;
					//printf("]\n");
				}
			}
			continue;
		}

		// if we have received something via serial see how
		// much of it we could fit in the transmit FIFO
		slen = serial_read_available();
		if (slen + tx_fifo_bytes > tx_chunk_size) {
			slen = tx_chunk_size - tx_fifo_bytes;
		}
		if (slen > 0 && serial_read_buf(rbuf, slen)) {
			// put it in the send FIFO
			radio_write_transmit_fifo(slen, rbuf);
			tx_fifo_bytes += slen;
		}
		
		__critical {
			current_window = tx_window_remaining;
		}

		if (current_window == 0) {
			yielded_window = false;
			continue;
		}

		if (yielded_window) {
			continue;
		}

		if (current_window * (uint16_t)bytes_per_tick < tx_fifo_bytes+PACKET_OVERHEAD) {
			// we can't fit the whole fifo in our remaining
			// window, so start receiving instead
			continue;
		}

		if (preamble_wait > 0) {
			// we saw a preamble previously and are now
			// waiting for a possible packet
			continue;
		}

		if (radio_preamble_detected()) {
			// a preamble has been detected. Don't
			// transmit for a while
			preamble_wait = silence_period;
			debug("PREAMBLE %d\n", (int)preamble_wait);
			continue;
		}

		if (tx_fifo_bytes != 0) {
			// show the user that we're sending data
			LED_ACTIVITY = LED_ON;
		}

		// add the control byte
		rbuf[0] = current_window;
		radio_write_transmit_fifo(1, rbuf);

		// set right transmit channel
		radio_set_channel(fhop_transmit_channel());

		// start transmitting the packet
		radio_transmit_start(tx_fifo_bytes+1, current_window+silence_period);
		if (tx_fifo_bytes == 0) {
			debug("YIELD %d\n", (int)current_window);
			// sending a zero byte packet gives up
			// our window, but doesn't change the
			// start of the next window
			yielded_window = true;
		}

		// set right receive channel
		radio_set_channel(fhop_receive_channel());

		// re-enable the receiver
		radio_receiver_on();

		// clear the transmit FIFO. This shouldn't
		// actually be needed, but I have seen some
		// strange situations where the FIFO gets out
		// of sync
		radio_clear_transmit_fifo();
		if (tx_fifo_bytes != 0) {
			LED_ACTIVITY = LED_OFF;
		}
		tx_fifo_bytes = 0;
	}
}

// estimate packet sizes and TDM constants
static void tdm_estimate(void)
{
	// work out how many milliticks a byte takes to come over the air
	milliticks_per_byte = 204800UL / (radio_air_rate() / 8);

	// work out how many bytes we can safely transmit in one tick
	bytes_per_tick = 1024 / milliticks_per_byte;
	if (bytes_per_tick == 0) {
		bytes_per_tick = 1;
	}
	
	// work out how long neither end will transmit for between windows
	silence_period = (tx_chunk_size + PACKET_OVERHEAD + (bytes_per_tick/2)) / bytes_per_tick;
	if (silence_period < 2) {
		silence_period = 2;
	}
	
	// work out the default transmit window in ticks. This
	// guarantees 3 full sized packets
	tx_window_width = 3 * silence_period;
}

// initialise the TDM subsystem
void tdm_init(void)
{
	// at very low data rates we need to lower the packet size
	// to prevent uint8_t overflows
	while (true) {
		// estimate the TDM packet timings
		tdm_estimate();

		if (2*(silence_period+(uint16_t)tx_window_width) < 128) {
			// thats a reasonable time for a TDM round
			break;
		}

		// try a smaller packet size
		tx_chunk_size--;
		if (tx_chunk_size == 1) {
			// we need at least 1 byte in a packet
			break;
		}
	}
}

// blink the radio LED if we have not received any packets
static void link_update(void)
{
	if (received_packet) {
		LED_RADIO = LED_ON;
		received_packet = 0;
	} else {
		LED_RADIO = blink_state;
		blink_state = !blink_state;

		// randomise the next transmit window using some
		// entropy from the radio
		if (radio_entropy() & 1) {
			next_tx_window += silence_period;
		}
		fhop_set_locked(false);
	}
}

// called every 5ms to update TDM counters
void tdm_tick(void)
{
	tick_counter++;

	if (tick_counter == 0) {
		link_update();
	}

	if (preamble_wait > 0)
		preamble_wait--;

	// update transmit windows
	if (tx_window_remaining > 0) {
		tx_window_remaining--;
		if (tx_window_remaining == 0) {
			// tell the frequency hopping system that
			// our transmit window has closed
			fhop_window_change();
			other_window_remaining = silence_period+tx_window_width;
		}
	}
	if (tick_counter == next_tx_window) {
		// tell the frequency hopping system that
		// our transmit window has opened
		if (tx_window_remaining < tx_window_width) {
			tx_window_remaining = tx_window_width;
		}
		next_tx_window += 2*(tx_window_width + silence_period);
	}
	if (other_window_remaining > 0) {
		other_window_remaining--;
		if (other_window_remaining == 0) {
			// the other radios transmit window should be closing
			fhop_window_change();
		}
	}
}

