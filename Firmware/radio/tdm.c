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

#include <stdarg.h>
#include "radio.h"
#include "tdm.h"
#include "timer.h"
#include "packet.h"
#include "golay.h"
#include "freq_hopping.h"
#include "crc.h"

#define USE_TICK_YIELD 1

/// the state of the tdm system
enum tdm_state { TDM_TRANSMIT=0, TDM_SILENCE1=1, TDM_RECEIVE=2, TDM_SILENCE2=3 };
__pdata static enum tdm_state tdm_state;

/// a packet buffer for the TDM code
__xdata uint8_t	pbuf[MAX_DATA_PACKET_LENGTH];

/// how many 16usec ticks are remaining in the current state
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

/// the latency in 16usec RTC ticks for sending a zero length packet
__pdata static uint16_t packet_latency;

/// the time in 16usec RTC ticks for sending one byte
__pdata static uint16_t ticks_per_byte;

/// number of 16usec ticks to wait for a preamble to turn into a packet
/// This is set when we get a preamble interrupt, and causes us to delay
/// sending for a maximum packet latency. This is used to make it more likely
/// that two radios that happen to be exactly in sync in their sends
/// will eventually get a packet through and get their transmit windows
/// sorted out
__pdata uint16_t transmit_wait;


/// test data to display in the main loop. Updated when the tick
/// counter wraps, zeroed when display has happened
__pdata uint8_t test_display;

/// set when we should send a statistics packet on the next round
static __bit send_statistics;

/// packet and RSSI statistics
__pdata struct statistics {
	uint8_t average_rssi;
	uint8_t remote_average_rssi;
	uint8_t receive_count;
	uint8_t round_count;
} statistics, remote_statistics;

struct tdm_trailer {
	uint16_t window:14;
	uint8_t bonus:1;
	uint8_t resend:1;
};
__pdata struct tdm_trailer trailer;

/// display test output
///
static void
display_test_output(void)
{
	if (test_display & AT_TEST_RSSI) {
		printf("LOCAL RSSI: %d  pkts/rounds: %u/%u   ",
		       (unsigned)statistics.average_rssi,
		       (unsigned)statistics.receive_count,
		       (unsigned)statistics.round_count);
		printf("REMOTE RSSI: %d pkts/rounds: %u/%u",
		       (int)remote_statistics.average_rssi,
		       (unsigned)remote_statistics.receive_count,
		       (unsigned)remote_statistics.round_count);
		printf("  txe=%u rxe=%u stx=%u srx=%u ecc=%u\n",
		       (unsigned)errors.tx_errors,
		       (unsigned)errors.rx_errors,
		       (unsigned)errors.serial_tx_overflow,
		       (unsigned)errors.serial_rx_overflow,
		       (unsigned)errors.corrected_errors);
	}
}


/// estimate the flight time for a packet given the payload size
///
/// @param packet_len		payload length in bytes
///
/// @return			flight time in 16usec ticks
static uint16_t flight_time_estimate(__pdata uint8_t packet_len)
{
	return packet_latency + (packet_len * ticks_per_byte);
}


/// synchronise tx windows
///
/// we receive a 16 bit value with each packet which indicates how many
/// more 16usec ticks the sender has in their transmit window. The
/// sender has already adjusted the value for the flight time
///
/// The job of this function is to adjust our own transmit window to
/// match the other radio and thus bring the two radios into sync
///
static void
sync_tx_windows(__pdata uint8_t packet_length)
{
	__data enum tdm_state old_state = tdm_state;
	__pdata uint16_t old_remaining = tdm_state_remaining;

	if (trailer.bonus) {
		// the other radio is using our transmit window
		// via yielded ticks
		if (old_state == TDM_SILENCE1) {
			// This can be caused by a packet
			// taking longer than expected to arrive.
			// don't change back to transmit state or we
			// will cause an extra frequency change which
			// will get us out of sequence
			tdm_state_remaining = silence_period;
		} else if (old_state == TDM_RECEIVE || old_state == TDM_SILENCE2) {
			// this is quite strange. We received a packet
			// so we must have been on the right
			// frequency. Best bet is to set us at the end
			// of their silence period
			tdm_state = TDM_SILENCE2;
			tdm_state_remaining = 1;
		} else {
			tdm_state = TDM_TRANSMIT;
			tdm_state_remaining = trailer.window;
		}
	} else {
		// we are in the other radios transmit window, our
		// receive window
		tdm_state = TDM_RECEIVE;
		tdm_state_remaining = trailer.window;
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
tdm_state_update(__pdata uint16_t tdelta)
{
	// update the amount of time we are waiting for a preamble
	// to turn into a real packet
	if (tdelta > transmit_wait) {
		transmit_wait = 0;
	} else {
		transmit_wait -= tdelta;
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

		// change frequency at the start and end of our transmit window
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
		transmit_wait = 0;

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
			if (tdm_state_remaining > silence_period) {
				tdm_state_remaining -= silence_period;
			} else {
				tdm_state_remaining = 1;
			}
		}
		fhop_set_locked(false);

		// reset statistics when unlocked
		memset(&statistics, 0, sizeof(statistics));
	}

	test_display = at_testmode;
	send_statistics = 1;
}

// a stack carary to detect a stack overflow
__at(0xFF) uint8_t __idata _canary;

/// main loop for time division multiplexing transparent serial
///
void
tdm_serial_loop(void)
{
	__pdata uint16_t last_t = timer2_tick();
	__pdata uint16_t last_link_update = last_t;

	_canary = 42;

	for (;;) {
		__pdata uint8_t	len;
		// add an extra 3 bytes to hold length and CRC in ecc.c
		__pdata uint16_t tnow, tdelta;
		__pdata uint8_t max_xmit;

		if (_canary != 42) {
			panic("stack blown\n");
		}

		if (pdata_canary != 0x41) {
			panic("pdata canary changed\n");
		}

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
		if (radio_receive_packet(&len, pbuf)) {

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
			transmit_wait = 0;

			if (len < 2) {
				// not a valid packet. We always send
				// two control bytes at the end of every packet
				continue;
			}

			// extract control bytes from end of packet
			memcpy(&trailer, &pbuf[len-sizeof(trailer)], sizeof(trailer));
			len -= sizeof(trailer);

			if (trailer.window == 0 && len != 0) {
				// its a control packet
				if (len == sizeof(struct statistics)) {
					memcpy(&remote_statistics, pbuf, len);
				}

				// don't count control packets in the stats
				statistics.receive_count--;
			} else if (trailer.window == 0) {
				panic("zero window recv");
			} else {
				// sync our transmit windows based on
				// received header
				sync_tx_windows(len);
				last_t = timer2_tick();

				if (len != 0 && 
				    !packet_is_duplicate(len, pbuf, trailer.resend) &&
				    !at_mode_active) {
					// its user data - send it out
					// the serial port
					//printf("rcv(%d,[", len);
					LED_ACTIVITY = LED_ON;
					serial_write_buf(pbuf, len);
					LED_ACTIVITY = LED_OFF;
					//printf("]\n");
				}
			}
			continue;
		}

		// see how many 16usec ticks have passed and update
		// the tdm state machine
		tnow = timer2_tick();
		tdelta = tnow - last_t;
		tdm_state_update(tdelta);
		last_t += tdelta;

		// update link status every 0.5s
		if (tnow - last_link_update > 32768) {
			link_update();
			last_link_update = tnow;
		}

		// we are allowed to transmit in our transmit window
		// or in the other radios transmit window if we have
		// bonus ticks
#if USE_TICK_YIELD
		if (tdm_state != TDM_TRANSMIT &&
		    !(bonus_transmit && tdm_state == TDM_RECEIVE)) {
			// we cannot transmit now
			continue;
		}
#else
		if (tdm_state != TDM_TRANSMIT) {
			continue;
		}		
#endif

		if (transmit_wait != 0) {
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
			transmit_wait = flight_time_estimate(MAX_DATA_PACKET_LENGTH);
			continue;
		}

		// how many bytes could we transmit in the time we
		// have left?
		if (tdm_state_remaining < packet_latency) {
			// none ....
			continue;
		}
		max_xmit = (tdm_state_remaining - packet_latency) / ticks_per_byte;
		if (max_xmit < sizeof(trailer)+1) {
			// can't fit the trailer in with a byte to spare
			continue;
		}
		max_xmit -= sizeof(trailer)+1;
		if (max_xmit > MAX_DATA_PACKET_LENGTH-sizeof(trailer)) {
			max_xmit = MAX_DATA_PACKET_LENGTH-sizeof(trailer);
		}

		// ask the packet system for the next packet to send
		len = packet_get_next(max_xmit, pbuf);

		trailer.bonus = (tdm_state == TDM_RECEIVE);
		trailer.resend = packet_is_resend();

		if (tdm_state == TDM_TRANSMIT &&
		    len == 0 && 
		    send_statistics && 
		    max_xmit >= sizeof(statistics)) {
			// send a statistics packet
			send_statistics = 0;
			memcpy(pbuf, &statistics, sizeof(statistics));
			len = sizeof(statistics);
		
			// mark a stats packet with a zero window
			trailer.window = 0;
			trailer.resend = 0;
		} else {
			// calculate the control word as the number of
			// 16usec RTC ticks that will be left in this
			// tdm state after this packet is transmitted
			trailer.window = (uint16_t)(tdm_state_remaining - flight_time_estimate(len+sizeof(trailer)));
		}

		// set right transmit channel
		radio_set_channel(fhop_transmit_channel());

		memcpy(&pbuf[len], &trailer, sizeof(trailer));

		if (len != 0 && trailer.window != 0) {
			// show the user that we're sending real data
			LED_ACTIVITY = LED_ON;
		}

		if (len == 0) {
			// sending a zero byte packet gives up
			// our window, but doesn't change the
			// start of the next window
			transmit_yield = 1;
		}

		// after sending a packet leave a bit of time before
		// sending the next one. The receivers don't cope well
		// with back to back packets
		transmit_wait = packet_latency;

		// start transmitting the packet
		if (!radio_transmit(len + sizeof(trailer), pbuf, tdm_state_remaining + (silence_period/2)) &&
		    len != 0 && trailer.window != 0) {
			packet_force_resend();
		}

		// set right receive channel
		radio_set_channel(fhop_receive_channel());

		// re-enable the receiver
		radio_receiver_on();

		if (len != 0 && trailer.window != 0) {
			LED_ACTIVITY = LED_OFF;
		}
	}
}

/// table mapping air rate to measured latency and per-byte
/// timings in 16usec units
__code static const struct {
	uint32_t air_rate;
	uint16_t latency;
	uint16_t per_byte;
} timing_table[] = {
#if USE_ECC_CODE
	{ 0,   17000, 1918 },
	{ 1, 10692, 1030 },
	{ 2, 5355, 515 },
	{ 4, 2687, 258 },
	{ 8, 1352, 130 },
	{ 9, 1130, 108 },
	{ 16, 685, 65 },
	{ 19, 574, 54 },
	{ 24, 464, 44 },
	{ 32, 352, 33 },
	{ 64, 186, 17 },
	{ 128, 103, 9 },
	{ 192, 75, 6 },
	{ 256,    40, 3 },
#else
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
	{ 256, 30,    2 },
#endif
};

#if 1
/// build the timing table
static void tdm_build_timing_table(void)
{
        __xdata uint8_t pbuf[MAX_DATA_PACKET_LENGTH];
	__idata uint8_t i, j;

	for (i=2; i<ARRAY_LENGTH(timing_table); i++) {
		__idata uint32_t latency_sum=0, per_byte_sum=0;
		uint8_t size = MAX_DATA_PACKET_LENGTH;
		radio_configure(timing_table[i].air_rate*1000UL);
		for (j=0; j<10; j++) {
			__idata uint16_t time_0, time_max, t1, t2;
			radio_set_channel(1);
			radio_receiver_on();
			if (serial_read_available() > 0) {
				return;
			}
			t1 = timer2_tick();
			if (!radio_transmit(0, pbuf, 0xFFFF)) {
				break;
			}
			t2 = timer2_tick();
			radio_receiver_on();

			time_0 = t2-t1;

			radio_set_channel(2);
			t1 = timer2_tick();
			if (!radio_transmit(size, pbuf, 0xFFFF)) {
				size /= 2;
				j--;
				continue;
			}

			t2 = timer2_tick();
			radio_receiver_on();

			time_max = t2-t1;
			latency_sum += time_0;
			per_byte_sum += ((size/2) + (time_max - time_0))/size;
		}
		if (j > 0) {
			printf("{ %u, %u, %u },\n",
			       (unsigned)(radio_air_rate()/1000UL),
			       (unsigned)(latency_sum/j),
			       (unsigned)(per_byte_sum/j));
		}
	}
}

/// test the timing table
static void tdm_test_timing(void)
{
        __xdata uint8_t pbuf[MAX_DATA_PACKET_LENGTH+3];
	__pdata uint8_t i, failures=0;
	
	memset(pbuf, 42, sizeof(pbuf));

	for (i=0; i<255; i++) {
		__pdata uint8_t len = rand() & 0x3f;
		__pdata uint16_t fte = flight_time_estimate(len);
		__pdata int16_t diff;
		__pdata uint16_t t1, t2;

		radio_set_channel(1);
		//radio_receiver_on();
		t1 = timer2_tick();
		if (!radio_transmit(len, pbuf, fte + silence_period)) {
			printf("max timeout len=%u\n", 
			       (unsigned)len);
			failures++;
			continue;
		}
		t2 = timer2_tick();
		diff = (int16_t)(((uint16_t)(t2-t1)) - fte);
		printf("i=%u len=%u fte=%u t=%u diff=%d\n",
		       (unsigned)i, (unsigned)len, 
		       fte, (uint16_t)(t2-t1), diff);
		if (diff > (int16_t)silence_period || diff < -(int16_t)silence_period) {
			failures++;
		}
	}
	printf("%u failures\n", (unsigned)failures);
}

// test hardware CRC code
static void crc_test(void)
{
	__xdata uint8_t d[4] = { 0x01, 0x00, 0xbb, 0xcc };
	__pdata uint16_t crc;
	crc = crc16(4, &d[0]);
	printf("CRC: %x %x\n", crc, 0xb166);	
}

// test golay encoding
static void golay_test(void)
{
	uint8_t i;
	uint16_t t1, t2;
	__xdata uint8_t	buf[MAX_AIR_PACKET_LENGTH];
	for (i=0; i<MAX_DATA_PACKET_LENGTH; i++) {
		pbuf[i] = i;
	}
	t1 = timer2_tick();
	golay_encode(MAX_DATA_PACKET_LENGTH, pbuf, buf);
	t2 = timer2_tick();
	printf("encode %u bytes took %u 16usec ticks\n",
	       (unsigned)MAX_DATA_PACKET_LENGTH,
	       t2-t1);
	// add an error in the middle
	buf[MAX_DATA_PACKET_LENGTH] ^= 0x23;
	buf[1] ^= 0x70;
	t1 = timer2_tick();
	golay_decode(MAX_DATA_PACKET_LENGTH*2, buf, pbuf);
	t2 = timer2_tick();
	printf("decode %u bytes took %u 16usec ticks\n",
	       (unsigned)MAX_DATA_PACKET_LENGTH*2,
	       t2-t1);
	for (i=0; i<MAX_DATA_PACKET_LENGTH; i++) {
		if (pbuf[i] != i) {
			printf("golay error at %u\n", (unsigned)i);
		}
	}
	t1 = timer2_tick();
	crc16(MAX_DATA_PACKET_LENGTH, pbuf);
	t2 = timer2_tick();
	printf("crc %u bytes took %u 16usec ticks\n",
	       (unsigned)MAX_DATA_PACKET_LENGTH,
	       t2-t1);
}
#endif


// initialise the TDM subsystem
void
tdm_init(void)
{
	__pdata uint8_t i;
	__pdata uint8_t air_rate = radio_air_rate() / 1000UL;
	__pdata uint32_t window_width;

#define REGULATORY_MAX_WINDOW (((1000000UL/16)*4)/10)

	// tdm_build_timing_table();

	// find the packet latency and time per byte from the timing
	// table.
	for (i=0; i<ARRAY_LENGTH(timing_table); i++) {
		if (timing_table[i].air_rate == air_rate) break;
	}
	if (i == ARRAY_LENGTH(timing_table)) {
		panic("missing rate in timing_table");
	}

        // find the packet latency and time per byte from the timing
        // table.
	packet_latency = timing_table[i].latency;
        ticks_per_byte = timing_table[i].per_byte;

	// set the silence period to two times the packet latency
        silence_period = 2*packet_latency;

        // set the transmit window to allow for 3 full sized packets
	window_width = 3*(packet_latency+(MAX_DATA_PACKET_LENGTH*(uint32_t)ticks_per_byte));

	// the window width cannot be more than 0.4 seconds to meet US
	// regulations
	if (window_width >= REGULATORY_MAX_WINDOW) {
		window_width = REGULATORY_MAX_WINDOW;
	}

	// make sure it fits in the 14 bits of the trailer window
	if (window_width > 0x3FFF) {
		window_width = 0x3FFF;
	}

	tx_window_width = window_width;

	// tell the packet subsystem our max packet size, which it
	// needs to know for MAVLink packet boundary detection
	i = (tx_window_width - packet_latency) / ticks_per_byte;
	if (i > MAX_DATA_PACKET_LENGTH - sizeof(trailer)) {
		i = MAX_DATA_PACKET_LENGTH - sizeof(trailer);
	}
	packet_set_max_xmit(i);

	// crc_test();

	// tdm_test_timing();

	// golay_test();
}


/// report tdm timings
///
void tdm_report_timing(void)
{
	printf("silence_period: %u\n", (unsigned)silence_period); delay_msec(1);
	printf("tx_window_width: %u\n", (unsigned)tx_window_width); delay_msec(1);
}

