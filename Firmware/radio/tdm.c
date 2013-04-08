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
#include "freq_hopping.h"
#include "crc.h"

#define USE_TICK_YIELD 1

/// the state of the tdm system
enum tdm_state { TDM_TRANSMIT=0, TDM_SILENCE1=1, TDM_RECEIVE=2, TDM_SILENCE2=3 };
__pdata static enum tdm_state tdm_state;

/// a packet buffer for the TDM code
__xdata uint8_t	pbuf[MAX_PACKET_LENGTH];

/// how many 16usec ticks are remaining in the current state
__pdata static uint16_t tdm_state_remaining;

/// This is enough to hold at least 3 packets and is based
/// on the configured air data rate.
__pdata static uint16_t tx_window_width;

/// the maximum data packet size we can fit
__pdata static uint8_t max_data_packet_length;

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
// when the 16 bit timer2_tick() value wraps we check if we have received a
// packet since the last wrap (ie. every second)
// If we have the green radio LED is held on.
// Otherwise it blinks every 1 seconds. The received_packet flag
// is set for any received packet, whether it contains user data or
// not.
static __bit blink_state;
static __bit received_packet;

/// the latency in 16usec timer2 ticks for sending a zero length packet
__pdata static uint16_t packet_latency;

/// the time in 16usec ticks for sending one byte
__pdata static uint16_t ticks_per_byte;

/// number of 16usec ticks to wait for a preamble to turn into a packet
/// This is set when we get a preamble interrupt, and causes us to delay
/// sending for a maximum packet latency. This is used to make it more likely
/// that two radios that happen to be exactly in sync in their sends
/// will eventually get a packet through and get their transmit windows
/// sorted out
__pdata uint16_t transmit_wait;

/// the long term duty cycle we are aiming for
__pdata uint8_t duty_cycle;

/// the average duty cycle we have been transmitting
__data static float average_duty_cycle;

/// duty cycle offset due to temperature
__pdata uint8_t duty_cycle_offset;

/// set to true if we need to wait for our duty cycle average to drop
static bool duty_cycle_wait;

/// how many ticks we have transmitted for in this TDM round
__pdata static uint16_t transmitted_ticks;

/// the LDB (listen before talk) RSSI threshold
__pdata uint8_t lbt_rssi;

/// how long we have listened for for LBT
__pdata static uint16_t lbt_listen_time;

/// how long we have to listen for before LBT is OK
__pdata static uint16_t lbt_min_time;

/// random addition to LBT listen time (see European regs)
__pdata static uint16_t lbt_rand;

/// test data to display in the main loop. Updated when the tick
/// counter wraps, zeroed when display has happened
__pdata uint8_t test_display;

/// set when we should send a statistics packet on the next round
static __bit send_statistics;

/// set when we should send a MAVLink report pkt
extern bool seen_mavlink;

struct tdm_trailer {
	uint16_t window:13;
	uint16_t command:1;
	uint16_t bonus:1;
	uint16_t resend:1;
};
__pdata struct tdm_trailer trailer;

/// buffer to hold a remote AT command before sending
static bool send_at_command;
static __pdata char remote_at_cmd[AT_CMD_MAXLEN + 1];

/// display RSSI output
///
void
tdm_show_rssi(void)
{
	printf("L/R RSSI: %u/%u  L/R noise: %u/%u pkts: %u ",
	       (unsigned)statistics.average_rssi,
	       (unsigned)remote_statistics.average_rssi,
	       (unsigned)statistics.average_noise,
	       (unsigned)remote_statistics.average_noise,
	       (unsigned)statistics.receive_count);
	printf(" txe=%u rxe=%u stx=%u srx=%u ecc=%u/%u temp=%d dco=%u\n",
	       (unsigned)errors.tx_errors,
	       (unsigned)errors.rx_errors,
	       (unsigned)errors.serial_tx_overflow,
	       (unsigned)errors.serial_rx_overflow,
	       (unsigned)errors.corrected_errors,
	       (unsigned)errors.corrected_packets,
	       (int)radio_temperature(),
	       (unsigned)duty_cycle_offset);
	statistics.receive_count = 0;
}

/// display test output
///
static void
display_test_output(void)
{
	if (test_display & AT_TEST_RSSI) {
		tdm_show_rssi();
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

	if (at_testmode & AT_TEST_TDM) {
		__pdata int16_t delta;
		delta = old_remaining - tdm_state_remaining;
		if (old_state != tdm_state ||
		    delta > (int16_t)packet_latency/2 ||
		    delta < -(int16_t)packet_latency/2) {
			printf("TDM: %u/%u len=%u ",
			       (unsigned)old_state,
			       (unsigned)tdm_state,
			       (unsigned)packet_length);
			printf(" delta: %d\n",
			       (int)delta);
		}
	}
}


// a stack carary to detect a stack overflow
__at(0xFF) uint8_t __idata _canary;

static uint8_t
swap_bit_order(register uint8_t v)
{
	register uint8_t v2;
	v2 = 0;
	v2 |= (v&1)<<7;
	v2 |= (v&2)<<5;
	v2 |= (v&4)<<3;
	v2 |= (v&8)<<1;
	v2 |= (v&16)>>1;
	v2 |= (v&32)>>3;
	v2 |= (v&64)>>5;
	v2 |= (v&128)>>7;
	return v2;
}

static void 
swap_packet_bit_order(__pdata uint8_t len)
{
	__pdata uint8_t i;
	for (i=0; i<len; i++) {
		pbuf[i] = swap_bit_order(pbuf[i]);
	}
}

static __pdata one_second_counter;

static void one_second(void)
{
	one_second_counter++;
	if (one_second_counter == 5) {
		fhop_next();
		radio_set_frequency(fhop_receive_freqency());
		radio_receiver_on();
		one_second_counter = 0;
	}
}

/// main loop for time division multiplexing transparent serial
///
void
tdm_serial_loop(void)
{
	__pdata uint16_t last_t = timer2_tick();
	__pdata uint16_t last_link_update = last_t;

	_canary = 42;

	// set right receive channel
	radio_set_frequency(fhop_receive_freqency());

	delay_set_ticks(100);

	for (;;) {
		__pdata uint8_t	len;

		if (delay_expired()) {
			one_second();
			delay_set_ticks(100);
		}

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

		// see if we have received a packet
		if (radio_receive_packet(&len, pbuf)) {
			__pdata uint8_t i;
			for (i=0; i<len; i++) {
				printf("%x ", (unsigned)pbuf[i]);
			}
			printf("\n");
			swap_packet_bit_order(len);
			for (i=0; i<len; i++) {
				printf("%x ", (unsigned)pbuf[i]);
			}
			printf("\n");
			serial_write_buf(pbuf, len);
			fhop_next();

			// re-enable the receiver
			radio_set_frequency(fhop_receive_freqency());
			radio_receiver_on();

			delay_set_ticks(100);
			one_second_counter = 0;
		}
	}
}



// initialise the TDM subsystem
void
tdm_init(void)
{
	__pdata uint16_t i;
	__pdata uint8_t air_rate = radio_air_rate();
	__pdata uint32_t window_width;

#define REGULATORY_MAX_WINDOW (((1000000UL/16)*4)/10)
#define LBT_MIN_TIME_USEC 5000

	// tdm_build_timing_table();

	// calculate how many 16usec ticks it takes to send each byte
	ticks_per_byte = (8+(8000000UL/(air_rate*1000UL)))/16;

	// calculate the minimum packet latency in 16 usec units
	// we initially assume a preamble length of 40 bits, then
	// adjust later based on actual preamble length. This is done
	// so that if one radio has antenna diversity and the other
	// doesn't, then they will both using the same TDM round timings
	packet_latency = (8+(10/2)) * ticks_per_byte + 13;

	max_data_packet_length = MAX_PACKET_LENGTH - sizeof(trailer);

	// set the silence period to two times the packet latency
        silence_period = 2*packet_latency;

        // set the transmit window to allow for 3 full sized packets
	window_width = 3*(packet_latency+(max_data_packet_length*(uint32_t)ticks_per_byte));

	// if LBT is enabled, we need at least 3*5ms of window width
	if (lbt_rssi != 0) {
		// min listen time is 5ms
		lbt_min_time = LBT_MIN_TIME_USEC/16;
		window_width = constrain(window_width, 3*lbt_min_time, window_width);
	}

	tx_window_width = window_width;

	// now adjust the packet_latency for the actual preamble
	// length, so we get the right flight time estimates, while
	// not changing the round timings
	packet_latency += ((settings.preamble_length-10)/2) * ticks_per_byte;

	// tell the packet subsystem our max packet size, which it
	// needs to know for MAVLink packet boundary detection
	i = (tx_window_width - packet_latency) / ticks_per_byte;
	if (i > max_data_packet_length) {
		i = max_data_packet_length;
	}
	packet_set_max_xmit(i);

	// crc_test();

	// tdm_test_timing();
	
	// golay_test();
}


/// report tdm timings
///
void 
tdm_report_timing(void)
{
	printf("silence_period: %u\n", (unsigned)silence_period); delay_msec(1);
	printf("tx_window_width: %u\n", (unsigned)tx_window_width); delay_msec(1);
	printf("max_data_packet_length: %u\n", (unsigned)max_data_packet_length); delay_msec(1);
}

