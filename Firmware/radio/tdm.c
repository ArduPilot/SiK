// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2012 Andrew Tridgell, All Rights Reserved
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

#define DEBUG 1

#include <stdarg.h>
#include "radio.h"
#include "tdm.h"
#include "timer.h"
#include "freq_hopping.h"

/// all of the tdm timings are in units of 25usec to match
/// the RTC resolution (assuming a 40kHz clock)
#define RTC_TICK_USEC 25

/// the state of the tdm system
enum tdm_state { TDM_TRANSMIT=0, TDM_SILENCE1=1, TDM_RECEIVE=2, TDM_SILENCE2=3 };
__pdata static enum tdm_state tdm_state;

/// how many 25usec ticks are remaining in the current state
__pdata static uint16_t tdm_state_remaining;

/// This is enough to hold at least 3 packets and is based
/// on the configured air data rate.
__pdata static uint16_t tx_window_width;

/// the silence period between transmit windows
/// This is calculated as the number of ticks it would take to transmit
/// two zero length packets
__pdata static uint16_t silence_period;

/// whether we can transmit in the other radios transmit window
/// due to the other radio yielding to us
static __bit bonus_transmit;

/// whether we have yielded our window to the other radio
static __bit transmit_yield;

// activity indication
// when the 16 bit RTC ticks wrap we  check if we have received a
// packet since the last wrap (ie. every 1.6 seconds)
// If we have the green radio LED is held on.
// Otherwise it blinks every 1.6 seconds. The received_packet flag
// is set for any received packet, whether it contains user data or
// not.
static __bit blink_state;
static __bit received_packet;

/// the latency in 25usec RTC ticks for sending a zero length packet
__pdata static uint16_t packet_latency;

/// the time in 25usec RTC ticks for sending one byte
__pdata static uint16_t ticks_per_byte;

/// number of 25usec ticks to wait for a preamble to turn into a packet
/// This is set when we get a preamble interrupt, and causes us to delay
/// sending for a maximum packet latency. This is used to make it more likely
/// that two radios that happen to be exactly in sync in their sends
/// will eventually get a packet through and get their transmit windows
/// sorted out
__pdata static uint16_t preamble_wait;


/// test data to display in the main loop. Updated when the tick
/// counter wraps, zeroed when display has happened
__pdata static uint8_t test_display;

/// set when we should send a statistics packet on the next round
static __bit send_statistics;

/// packet and RSSI statistics
__xdata static struct statistics {
	uint8_t average_rssi;
	uint8_t remote_average_rssi;
	uint8_t receive_count;
	uint8_t round_count;
} statistics, remote_statistics;

/// display test output
///
static void
display_test_output(void)
{
	if (test_display & AT_TEST_RSSI) {
		uint8_t pkt_pct, remote_pkt_pct;
		if (statistics.round_count == 0) {
			pkt_pct = 0;
		} else {
			pkt_pct = (100*(uint16_t)statistics.receive_count)/statistics.round_count;
		}
		if (remote_statistics.round_count == 0) {
			remote_pkt_pct = 0;
		} else {
			remote_pkt_pct = (100*(uint16_t)remote_statistics.receive_count)/remote_statistics.round_count;
		}
		printf("LOCAL RSSI: %d  pkts/round: %d%c   ",
		       (int)statistics.average_rssi,
		       (int)pkt_pct, '%');
		printf("REMOTE RSSI: %d pkts/round: %d%c",
		       (int)remote_statistics.average_rssi,
		       (int)remote_pkt_pct, '%');
#if 1
		printf("  txe=%d rxe=%d stx=%d srx=%d\n",
		       (int)errors.tx_errors,
		       (int)errors.rx_errors,
		       (int)errors.serial_tx_overflow,
		       (int)errors.serial_rx_overflow);
#endif
	}
}


/// estimate the flight time for a packet given the payload size
///
/// @param packet_len		payload length in bytes
///
/// @return			flight time in 25usec ticks
static uint16_t flight_time_estimate(uint8_t packet_len)
{
	return packet_latency + (packet_len * ticks_per_byte);
}


/// synchronise tx windows
///
/// we receive a 16 bit value with each packet which indicates how many
/// more 25usec ticks the sender has in their transmit window. The
/// sender has already adjusted the value for the flight time
///
/// The job of this function is to adjust our own transmit window to
/// match the other radio and thus bring the two radios into sync
///
static void
sync_tx_windows(uint16_t other_tx_window, uint8_t packet_length)
{
	enum tdm_state old_state = tdm_state;
	uint16_t old_remaining = tdm_state_remaining;

	if (other_tx_window > tx_window_width+silence_period) {
		// the other radio is using our transmit window
		// via yielded ticks
		tdm_state = TDM_TRANSMIT;
		tdm_state_remaining = other_tx_window - (tx_window_width+silence_period);
	} else if (other_tx_window > tx_window_width) {
		// the other radio is using our silence period
		// via yielded ticks. This shouldn't happen
		tdm_state = TDM_SILENCE1;
		tdm_state_remaining = other_tx_window - tx_window_width;
	} else {
		// we are in the other radios transmit window, our
		// receive window
		tdm_state = TDM_RECEIVE;
		tdm_state_remaining = other_tx_window;
	}

	// if the other end has sent a zero length packet and we are
	// in their transmit window then they are yielding some ticks to us.
	bonus_transmit = (tdm_state == TDM_RECEIVE && packet_length==0);

	// if we are not in transmit state then we can't be yielded
	if (tdm_state != TDM_TRANSMIT) {
		transmit_yield = 0;
	}

#if 0
	{
		int16_t delta;
		delta = old_remaining - tdm_state_remaining;
		if (old_state != tdm_state ||
		    delta > (int16_t)silence_period ||
		    delta < -(int16_t)silence_period) {
			printf("TDM: %d/%d ",
			       (int)old_state,
			       (int)tdm_state);
			printf(" delta: %d\n",
			       (int)delta);
		}
	}
#endif
}

/// update the TDM state machine
///
static void
tdm_state_update(uint16_t tdelta)
{
	// update the amount of time we are waiting for a preamble
	// to turn into a real packet
	if (tdelta > preamble_wait) {
		preamble_wait = 0;
	} else {
		preamble_wait -= tdelta;
	}

	// have we passed the next transition point?
	while (tdelta >= tdm_state_remaining) {
		// advance the tdm state machine
		tdm_state = (tdm_state+1)%4;

		// work out the time remaining in this state
		tdelta -= tdm_state_remaining;

		if (tdm_state == TDM_TRANSMIT || tdm_state == TDM_RECEIVE) {
			tdm_state_remaining = tx_window_width;
		} else {
			tdm_state_remaining = silence_period;
		}

		// change freqency at the start and end of our transmit window
		// this maximises the chance we will be on the right frequency
		// to match the other radio
		if (tdm_state == TDM_TRANSMIT || tdm_state == TDM_SILENCE1) {
			fhop_window_change();
		}

		// we lose the bonus on all state changes
		bonus_transmit = 0;

		// reset yield flag on all state changes
		transmit_yield = 0;

		// no longer waiting for a packet
		preamble_wait = 0;

		if (tdm_state == TDM_TRANSMIT) {
			// update round statistics
			statistics.round_count++;
			if (statistics.round_count == 255) {
				statistics.receive_count >>= 1;
				statistics.round_count >>= 1;
			}
		}
	}

	tdm_state_remaining -= tdelta;
}

/// blink the radio LED if we have not received any packets
///
static void
link_update(void)
{
	if (received_packet) {
		LED_RADIO = LED_ON;
		received_packet = 0;
	} else {
		LED_RADIO = blink_state;
		blink_state = !blink_state;

		// randomise the next transmit window using some
		// entropy from the radio
		if (timer_entropy() & 1) {
			tdm_state_remaining = 1;
		}
		fhop_set_locked(false);

		// reset statistics when unlocked
		memset(&statistics, 0, sizeof(statistics));
	}

	test_display = at_testmode;
	send_statistics = 1;
}

/// main loop for time division multiplexing transparent serial
///
void
tdm_serial_loop(void)
{
	// the number of bytes currently in the send fifo
	uint8_t tx_fifo_bytes = 0;
	uint16_t last_t = timer2_tick();
	uint16_t last_link_update = last_t;

	for (;;) {
		__pdata uint8_t	rlen;
		__xdata uint8_t	rbuf[64];
		uint16_t slen;
		uint16_t other_tx_window;
		uint16_t tnow, tdelta;
		uint8_t max_xmit;

		// give the AT command processor a chance to handle a command
		at_command();

		// display test data if needed
		if (test_display) {
			display_test_output();
			test_display = 0;
		}

		// set right receive channel
		radio_set_channel(fhop_receive_channel());

		// see if we have received a packet
		if (radio_receive_packet(&rlen, rbuf)) {
			// update the activity indication
			received_packet = 1;
			fhop_set_locked(true);
			
			// update filtered RSSI value and packet stats
			statistics.average_rssi = (radio_last_rssi() + 7*(uint16_t)statistics.average_rssi)/8;
			statistics.receive_count++;
			if (statistics.receive_count == 255) {
				statistics.receive_count >>= 1;
				statistics.round_count >>= 1;
			}
			
			// we're not waiting for a preamble
			// any more
			preamble_wait = 0;

			if (rlen < 2) {
				// not a valid packet. We always send
				// two control bytes at the end of every packet
				continue;
			}

			// extract control bytes from end of packet
			other_tx_window = rbuf[rlen - 2];
			other_tx_window |= rbuf[rlen - 1]<<8;
			rlen -= 2;

			if (other_tx_window == 0 && rlen != 0) {
				// its a control packet
				if (rlen == sizeof(struct statistics)) {
					memcpy(&remote_statistics, rbuf, rlen);
				}

				// don't count control packets in the stats
				statistics.receive_count--;
			} else {
				// sync our transmit windows based on
				// received header
				sync_tx_windows(other_tx_window, rlen);
				last_t = timer2_tick();

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

		// see how many 25usec ticks have passed and update
		// the tdm state machine
		tnow = timer2_tick();
		tdelta = tnow - last_t;
		tdm_state_update(tdelta);
		last_t += tdelta;

		// update link status every 0.8s
		if (tnow - last_link_update > 32768) {
			link_update();
			last_link_update = tnow;
		}

		// we are allowed to transmit in our transmit window
		// or in the other radios transmit window if we have
		// bonus ticks
		if (tdm_state != TDM_TRANSMIT &&
		    !(bonus_transmit && tdm_state == TDM_RECEIVE)) {
			// we cannot transmit now
			continue;
		}

		if (preamble_wait != 0) {
			// we're waiting for a preamble to turn into a packet
			continue;
		}

		if (transmit_yield != 0) {
			// we've give up our window
			continue;
		}

		if (radio_preamble_detected()) {
			// a preamble has been detected. Don't
			// transmit for a while
			preamble_wait = flight_time_estimate(64);
			continue;
		}

		// how many bytes could we transmit in the time we
		// have left?
		if (tdm_state_remaining <= flight_time_estimate(3)) {
			// none ....
			continue;
		}
		max_xmit = (tdm_state_remaining - packet_latency) / ticks_per_byte;
		if (max_xmit > 0) {
			max_xmit--;
		}
		if (max_xmit > 62) {
			max_xmit = 62;
		}
		if (tx_fifo_bytes > max_xmit) {
			continue;
		}

		// if we have received something via serial see how
		// much of it we could fit in the transmit FIFO
		slen = serial_read_available();
		if (slen + tx_fifo_bytes > max_xmit) {
			slen = max_xmit - tx_fifo_bytes;
		}
		if (slen > 0 && serial_read_buf(rbuf, slen)) {
			// put it in the send FIFO
			radio_write_transmit_fifo(slen, rbuf);
			tx_fifo_bytes += slen;
		}

		// set right transmit channel
		radio_set_channel(fhop_transmit_channel());

		if (tx_fifo_bytes == 0 && send_statistics) {
			// send a statistics packet
			send_statistics = 0;
			memcpy(rbuf, &statistics, sizeof(statistics));
			tx_fifo_bytes = sizeof(statistics);
			radio_write_transmit_fifo(tx_fifo_bytes, rbuf);
		
			// use a control word of zero to indicate a
			// statistics packet
			rbuf[0] = rbuf[1] = 0;
		} else {
			// calculate the control word as the number of
			// 25usec RTC ticks that will be left in this
			// tdm state after this packet is transmitted
			uint16_t control = tdm_state_remaining;

			if (tdm_state == TDM_RECEIVE) {
				// indicate that we are using bonus ticks
				control += tx_window_width+silence_period;
			}

			// subtract the flight time
			control -= flight_time_estimate(tx_fifo_bytes);

			if (control == 0) {
				panic("zero TDM control");
			}

			// put the current window in the control word
			rbuf[0] = control & 0xFF;
			rbuf[1] = control >> 8;
		}

		// add the control word to the fifo
		radio_write_transmit_fifo(2, rbuf);

		if (tx_fifo_bytes != 0) {
			// show the user that we're sending real data
			LED_ACTIVITY = LED_ON;
		}

		if (tx_fifo_bytes == 0) {
			// sending a zero byte packet gives up
			// our window, but doesn't change the
			// start of the next window
			transmit_yield = 1;
		}

		// start transmitting the packet
		radio_transmit_start(tx_fifo_bytes + 2, tdm_state_remaining + (silence_period/2));

		// set right receive channel
		radio_set_channel(fhop_receive_channel());

		// re-enable the receiver
		radio_receiver_on();

		if (tx_fifo_bytes != 0) {
			LED_ACTIVITY = LED_OFF;
		}

		tx_fifo_bytes = 0;
	}
}

/// table mapping air rate to measured latency and per-byte
/// timings in 25usec units
__code static const struct {
	uint32_t air_rate;
	uint16_t latency;
	uint16_t per_byte;
} timing_table[] = {
	{ 0,   13130, 1028 },
	{ 1,   6574,  514 },
	{ 2,   3293,  257 },
	{ 4,   1653,  128 },
	{ 8,   833,   64 },
	{ 9,   697,   53 },
	{ 16,  423,   32 },
	{ 19,  354,   27 },
	{ 24,  286,   21 },
	{ 32,  219,   16 },
	{ 64,  117,   8 },
	{ 128, 66,    4 },
	{ 192, 49,    3 },
};

#if 0
/// build the timing table
static void tdm_build_timing_table(void)
{
        __xdata uint8_t rbuf[32];
	__idata uint8_t i, j;

	for (i=0; i<ARRAY_LENGTH(timing_table); i++) {
		__idata uint32_t latency_sum=0, per_byte_sum=0;
		radio_configure(timing_table[i].air_rate*1000UL);
		for (j=0; j<10; j++) {
			__idata uint16_t time_0, time_32, t1, t2;
			radio_set_channel(1);
			radio_receiver_on();
			t1 = timer2_tick();
			if (!radio_transmit_start(0, 0xFFFF)) {
				j--;
				continue;
			}
			t2 = timer2_tick();
			radio_receiver_on();

			time_0 = t2-t1;

			radio_write_transmit_fifo(32, rbuf);
			radio_set_channel(2);
			t1 = timer2_tick();
			if (!radio_transmit_start(32, 0xFFFF)) {
				j--;
				continue;
			}

			t2 = timer2_tick();
			radio_receiver_on();

			time_32 = t2-t1;
			latency_sum += time_0;
			per_byte_sum += (16 + (time_32 - time_0))/32;
		}
		printf("{ %u, %u, %u },\n",
		       (unsigned)(radio_air_rate()/1000UL),
		       (unsigned)(latency_sum/j),
		       (unsigned)(per_byte_sum/j));
	}
}
#endif

void
tdm_init(void)
{
	uint8_t i;
	uint8_t air_rate = radio_air_rate() / 1000UL;
	const uint8_t num_rates = ARRAY_LENGTH(timing_table);
	__pdata uint32_t window_width;
#define REGULATORY_MAX_WINDOW (((1000000UL/16)*4)/10)

	//tdm_build_timing_table();

	// find the packet latency and time per byte from the timing
	// table.
	for (i=0; i<num_rates; i++) {
		if (timing_table[i].air_rate == air_rate) break;
	}
	if (i==num_rates) {
		panic("missing rate in timing_table");
	}

        // find the packet latency and time per byte from the timing
        // table.
	packet_latency = timing_table[i].latency;
        ticks_per_byte = timing_table[i].per_byte;

	// set the silence period to two times the packet latency
        silence_period = 2*packet_latency;

        // set the transmit window to allow for 3 full sized packets
	window_width = 3*(packet_latency+(64*(uint32_t)ticks_per_byte));

	// the window width cannot be more than 0.4 seconds to meet US
	// regulations
	if (window_width >= REGULATORY_MAX_WINDOW) {
		window_width = REGULATORY_MAX_WINDOW;
	}

	tx_window_width = window_width;
}


/// report tdm timings
///
void tdm_report_timing(void)
{
	printf("silence_period: %d\n", (int)silence_period); delay_msec(1);
	printf("tx_window_width: %d\n", (int)tx_window_width); delay_msec(1);
}

