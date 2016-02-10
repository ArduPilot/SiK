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
#include "radio_old.h"
#include "timer.h"
#include "packet.h"
#include "freq_hopping.h"
#include "at.h"
#include "parameters.h"
#include "serial.h"
#include "pins_user.h"
#include "printfl.h"
#include "aes.h"

//#define USE_TICK_YIELD 1
#define TIMECODE 0

#if TIMECODE
#define TimeFunction(res,fn)\
tickStart = timer2_tick();\
fn;\
tdelta = (uint16_t) (timer2_tick() - tickStart);\
res = (tdelta > res) ? (tdelta) : (res)

#else
#define TimeFunction(res,fn) fn;
#endif


/// the state of the tdm system
enum tdm_state {
	TDM_TRANSMIT = 0, TDM_SILENCE1 = 1, TDM_RECEIVE = 2, TDM_SILENCE2 = 3
};
static enum tdm_state tdm_state;

/// a packet buffer for the TDM code
static uint8_t pbuf[MAX_PACKET_LENGTH];

static uint16_t tdm_end;																												// tdm remaining time will be relative to tick counter
static uint16_t tdm_ticks;

/// This is enough to hold at least 3 packets and is based
/// on the configured air data rate.
static uint16_t tx_window_width;

/// the maximum data packet size we can fit
static uint16_t max_data_packet_length;

/// the silence period between transmit windows
/// This is calculated as the number of ticks it would take to transmit
/// two zero length packets
static uint16_t silence_period;

/// whether we can transmit in the other radios transmit window
/// due to the other radio yielding to us
static bool bonus_transmit;

/// whether we have yielded our window to the other radio
static bool transmit_yield;

// activity indication
// when the 16 bit timer2_tick() value wraps we check if we have received a
// packet since the last wrap (ie. every second)
// If we have the green radio LED is held on.
// Otherwise it blinks every 1 seconds. The received_packet flag
// is set for any received packet, whether it contains user data or
// not.
static bool blink_state;
static bool received_packet;

/// the latency in 16usec timer2 ticks for sending a zero length packet
static uint16_t packet_latency;

/// the time in 16usec ticks for sending one byte
static uint16_t ticks_per_byte;

/// number of 16usec ticks to wait for a preamble to turn into a packet
/// This is set when we get a preamble interrupt, and causes us to delay
/// sending for a maximum packet latency. This is used to make it more likely
/// that two radios that happen to be exactly in sync in their sends
/// will eventually get a packet through and get their transmit windows
/// sorted out
static uint16_t wait_ticks;
static uint16_t wait_end;

/// the long term duty cycle we are aiming for
uint16_t duty_cycle;

/// the average duty cycle we have been transmitting
static float average_duty_cycle;

/// duty cycle offset due to temperature
uint16_t duty_cycle_offset;

/// set to true if we need to wait for our duty cycle average to drop
static bool duty_cycle_wait;

/// how many ticks we have transmitted for in this TDM round
static uint16_t transmitted_ticks;

/// the LDB (listen before talk) RSSI threshold
uint16_t lbt_rssi;

/// how long we have listened for for LBT
static uint16_t lbt_listen_time;

/// how long we have to listen for before LBT is OK
static uint16_t lbt_min_time;

/// random addition to LBT listen time (see European regs)
static uint16_t lbt_rand;

/// test data to display in the main loop. Updated when the tick
/// counter wraps, zeroed when display has happened
uint16_t test_display;

/// set when we should send a statistics packet on the next round
static bool send_statistics;

static uint16_t maxtdelta = 0;
static uint16_t maxTicksR = 0;
static uint16_t maxTicksX = 0;
static uint16_t maxTicksC = 0;
static uint16_t maxTicksA = 0;
static uint16_t maxTicksTx = 0;
static uint16_t maxTicksOn = 0;
static uint8_t  seqNo;
/// set when we should send a MAVLink report pkt
extern bool seen_mavlink;

struct __attribute__ ((__packed__)) tdm_trailer {
	uint16_t window;																												//:13;
	uint8_t  seqNo;
	uint8_t command :1;
	uint8_t bonus :1;
	uint8_t resend :1;
};
struct tdm_trailer trailer;


/// buffer to hold a remote AT command before sending
static bool send_at_command;
static char remote_at_cmd[AT_CMD_MAXLEN + 1];
static int16_t Set_tdm_state_remaining(uint16_t val, uint16_t reltick);
static int32_t tdm_state_remaining(void);
static int16_t transmit_wait(void);
static void Set_transmit_wait(uint16_t val, uint16_t reltick);

#define PACKET_OVERHEAD (sizeof(trailer))

/// display RSSI output
///
void tdm_show_rssi(void)
{
	printf("L/R RSSI: %u/%u  L/R noise: %u/%u pkts: %u ",
			(unsigned )statistics.average_rssi,
			(unsigned )remote_statistics.average_rssi,
			(unsigned )statistics.average_noise,
			(unsigned )remote_statistics.average_noise,
			(unsigned )statistics.receive_count);
	printf(" txe=%u rxe=%u stx=%u srx=%u ecc=%u/%u temp=%d dco=%u\n",
			(unsigned )errors.tx_errors, (unsigned )errors.rx_errors,
			(unsigned )errors.serial_tx_overflow,
			(unsigned )errors.serial_rx_overflow, (unsigned )errors.corrected_errors,
			(unsigned )errors.corrected_packets, (int )radio_temperature(),
			(unsigned )duty_cycle_offset);
	statistics.receive_count = 0;
}

/// display test output
///
static void display_test_output(void)
{
	if (test_display & AT_TEST_RSSI)
	{
		tdm_show_rssi();
	}
}

/// estimate the flight time for a packet given the payload size
///
/// @param packet_len		payload length in bytes
///
/// @return			flight time in 16usec ticks
static uint16_t flight_time_estimate(uint16_t packet_len)
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
static int16_t sync_tx_windows(uint16_t packet_length, uint16_t Tick)
{
	enum tdm_state old_state = tdm_state;
	//uint16_t old_remaining = tdm_state_remaining();
  int16_t delta =0;
	if (trailer.bonus)
	{
		// the other radio is using our transmit window
		// via yielded ticks
		if (old_state == TDM_SILENCE1)
		{
			// This can be caused by a packet
			// taking longer than expected to arrive.
			// don't change back to transmit state or we
			// will cause an extra frequency change which
			// will get us out of sequence
			delta = Set_tdm_state_remaining(silence_period, timer2_tick());
		}
		else if (old_state == TDM_RECEIVE || old_state == TDM_SILENCE2)
		{
			// this is quite strange. We received a packet
			// so we must have been on the right
			// frequency. Best bet is to set us at the end
			// of their silence period
			tdm_state = TDM_SILENCE2;
			delta = Set_tdm_state_remaining(1, timer2_tick());
		}
		else
		{
			tdm_state = TDM_TRANSMIT;
			delta = Set_tdm_state_remaining(trailer.window, Tick);
		}
	}
	else
	{
		// we are in the other radios transmit window, our
		// receive window
		tdm_state = TDM_RECEIVE;
		delta = Set_tdm_state_remaining(trailer.window, Tick);// offset for elapsed ticks since packet irq complete occurred
	}

	// if the other end has sent a zero length packet and we are
	// in their transmit window then they are yielding some ticks to us.
	bonus_transmit = (tdm_state == TDM_RECEIVE && packet_length == 0);

	// if we are not in transmit state then we can't be yielded
	if (tdm_state != TDM_TRANSMIT)
	{
		transmit_yield = 0;
	}

	if(at_testmode & AT_TEST_TDM) {
		uint16_t remaining = tdm_state_remaining();
		if (old_state != tdm_state || delta > (int16_t)(packet_latency>>1) || delta < -(int16_t) packet_latency>>1)
		{
			printf("TDM: %u/%u len=%u rem=%u win=%u T%u R%u Rx%u Ch%u AT%u Tx%u On%u",
					(unsigned )old_state, (unsigned )tdm_state, (unsigned )packet_length,
					remaining, trailer.window, maxtdelta, maxTicksR, maxTicksX, maxTicksC,
					maxTicksA, maxTicksTx, maxTicksOn);
			printf(" delta: %d\n", (int )delta);
			maxtdelta = 0;
			maxTicksR = 0;
			maxTicksX = 0;
			maxTicksC = 0;
			maxTicksA = 0;
			maxTicksTx = 0;
			maxTicksOn = 0;
		}
	}
	return(delta);
}

/// update the TDM state machine
///
static void tdm_state_update(void)
{
	// update the amount of time we are waiting for a preamble
	// to turn into a real packet
	// have we passed the next transition point?
	if(tdm_state_remaining() <= 0)
	{
		// advance the tdm state machine
		tdm_state = (tdm_state + 1) % 4;

		if (tdm_state == TDM_TRANSMIT || tdm_state == TDM_RECEIVE)
		{
			Set_tdm_state_remaining(tx_window_width+tdm_state_remaining(),timer2_tick());
		}
		else
		{
			Set_tdm_state_remaining(silence_period+tdm_state_remaining(),timer2_tick());
		}

		// change frequency at the start and end of our transmit window
		// this maximises the chance we will be on the right frequency
		// to match the other radio
		if (tdm_state == TDM_TRANSMIT || tdm_state == TDM_SILENCE1)
		{
			fhop_window_change();
			radio_set_channel(fhop_receive_channel(), false);
			radio_receiver_on();
			// TODO shouldn't we be changing channel now?, where is the call to  radio_set_channel ?

			if (num_fh_channels > 1)
			{
				// reset the LBT listen time
				lbt_listen_time = 0;
				lbt_rand = 0;
			}
		}

		if (tdm_state == TDM_TRANSMIT && (duty_cycle - duty_cycle_offset) != 100)
		{
			// update duty cycle averages
			average_duty_cycle = (0.95 * average_duty_cycle)
					+ (0.05 * (100.0 * transmitted_ticks)
							/ (2 * (silence_period + tx_window_width)));
			transmitted_ticks = 0;
			duty_cycle_wait =
					(average_duty_cycle >= (duty_cycle - duty_cycle_offset));
		}

		// we lose the bonus on all state changes
		bonus_transmit = 0;

		// reset yield flag on all state changes
		transmit_yield = 0;

		// no longer waiting for a packet
		Set_transmit_wait(0,0);
	}
}

/// change tdm phase
///
void tdm_change_phase(void)
{
	tdm_state = (tdm_state + 2) % 4;
}

/// called to check temperature
///
static void temperature_update(void)
{
	register int16_t diff;
	if (radio_get_transmit_power() <= 20)
	{
		duty_cycle_offset = 0;
		return;
	}

	diff = radio_temperature() - MAX_PA_TEMPERATURE;
	if (diff <= 0 && duty_cycle_offset > 0)
	{
		// under temperature
		duty_cycle_offset -= 1;
	}
	else if (diff > 10)
	{
		// getting hot!
		duty_cycle_offset += 10;
	}
	else if (diff > 5)
	{
		// well over temperature
		duty_cycle_offset += 5;
	}
	else if (diff > 0)
	{
		// slightly over temperature
		duty_cycle_offset += 1;
	}
	// limit to minimum of 20% duty cycle to ensure link stays up OK
	if ((duty_cycle - duty_cycle_offset) < 20)
	{
		duty_cycle_offset = duty_cycle - 20;
	}
}

/// blink the radio LED if we have not received any packets
///
static void link_update(void)
{
	static uint16_t unlock_count, temperature_count;
	if (received_packet)
	{
		unlock_count = 0;
		received_packet = false;
#ifdef TDM_SYNC_LOGIC
		TDM_SYNC_PIN = true;
#endif // TDM_SYNC_LOGIC
	}
	else
	{
		unlock_count++;
	}
	if (unlock_count < 6)
	{
		LED_RADIO(LED_ON);
	}
	else
	{
#ifdef TDM_SYNC_LOGIC
		TDM_SYNC_PIN = false;
#endif // TDM_SYNC_LOGIC
		LED_RADIO(blink_state);
		blink_state = !blink_state;
	}
	if (unlock_count > 20)
	{
		// if we have been unlocked for 20 seconds
		// then start frequency scanning again

		unlock_count = 5;
		// randomise the next transmit window using some
		// entropy from the radio if we have waited
		// for a full set of hops with this time base
		if (timer_entropy() & 1)
		{
			register uint16_t old_remaining = tdm_state_remaining();
			if (tdm_state_remaining() > silence_period)
			{
				Set_tdm_state_remaining(tdm_state_remaining()-(2*packet_latency),timer2_tick());
			}
			else
			{
				Set_tdm_state_remaining(1,timer2_tick());
			}
			if (at_testmode & AT_TEST_TDM)
			{
				printf("TDM: change timing %u/%u\n", (unsigned )old_remaining,
						(unsigned )tdm_state_remaining());
			}
		}
		if (at_testmode & AT_TEST_TDM)
		{
			printf("TDM: scanning\n");
		}
		fhop_set_locked(false);
	}

	if (unlock_count != 0)
	{
		statistics.average_rssi = (radio_last_rssi()
				+ 3 * (uint16_t) statistics.average_rssi) / 4;

		// reset statistics when unlocked
		statistics.receive_count = 0;
	}
	if (unlock_count > 5)
	{
		memset(&remote_statistics, 0, sizeof(remote_statistics));
	}

	test_display = at_testmode;
	send_statistics = 1;

	temperature_count++;
	if (temperature_count == 4)
	{
		// check every 2 seconds
		temperature_update();
		temperature_count = 0;
	}
}

// dispatch an AT command to the remote system
void tdm_remote_at(void)
{
	memcpy(remote_at_cmd, at_cmd, strlen(at_cmd) + 1);
	send_at_command = true;
}

// handle an incoming at command from the remote radio
static void handle_at_command(uint8_t len)
{
	if (len < 2 || len > AT_CMD_MAXLEN || pbuf[0] != (uint8_t) 'R'
			|| pbuf[1] != (uint8_t) 'T')
	{
		// assume its an AT command reply
		register uint8_t i;
		for (i = 0; i < len; i++)
		{
			putChar(pbuf[i]);
		}
		return;
	}

	// setup the command in the at_cmd buffer
	memcpy(at_cmd, pbuf, len);
	at_cmd[len] = 0;
	at_cmd[0] = 'A'; // replace 'R'
	at_cmd_len = len;
	at_cmd_ready = true;

	// run the AT command, capturing any output to the packet
	// buffer
	// this reply buffer will be sent at the next opportunity
	printf_start_capture(pbuf, sizeof(pbuf));
	at_command();
	len = printf_end_capture();
	if (len > 0)
	{
		packet_inject(pbuf, len);
	}
}

// a stack carary to detect a stack overflow
//__at(0xFF) uint8_t __idata _canary;

/// main loop for time division multiplexing transparent serial
///
void tdm_serial_loop(void)
{
	uint16_t tickStart;	// used for timing code execution time
	uint16_t tickEnd;	// used for timing code execution time
	static uint16_t TXSetupTicks=0;	// how many ticks to setup tx before tx start called
	static uint16_t TXDelayTicks=0; // how many ticks from max xmit calc to Tx call (when TxSetup Starts)
	static uint16_t tnow;
	static uint16_t tdelta;
	static bool Init = false;
	static uint16_t last_t;
	static uint16_t last_link_update;
	if (!Init)
	{
		last_t = timer2_tick();
		last_link_update = last_t;
		Init = true;
	}
	TimeFunction(maxTicksR,radio_daemon());

	#ifdef RADIO_SPLAT_TESTING_MODE
	for (;;)
	{
		radio_set_channel(0,false);
		radio_transmit(MAX_PACKET_LENGTH, pbuf, 0);
		//radio_receiver_on();
	}
#else
	uint16_t i;
	for (i = 0; i < 1; i++)
	{																// do a once loop so continue statements work
		static uint8_t lastSeqNo;
		static uint16_t len;
		static int16_t max_xmit;
		uint16_t RxTick;
		// give the AT command processor a chance to handle a command
		TimeFunction(maxTicksA,at_command());
		// display test data if needed
		if (test_display)
		{
			display_test_output();
			test_display = 0;
		}

		if (seen_mavlink && feature_mavlink_framing && !at_mode_active)
		{
			seen_mavlink = false;
			MAVLink_report();
		}
		TimeFunction(maxTicksC,radio_set_channel(fhop_receive_channel(), true));

		// get the time before we check for a packet coming in
		tnow = timer2_tick();
		tickStart = tnow;
		// see if we have received a packet
		if (radio_receive_packet(&len, pbuf, &RxTick))
		{
			tdelta = (uint16_t) (timer2_tick() - tickStart);
			maxTicksX = (tdelta > maxTicksX) ? (tdelta) : (maxTicksX);
			LED_ACTIVITY(LED_ON);

			// update the activity indication
			received_packet = true;
			fhop_set_locked(true);

			// update filtered RSSI value and packet stats
			statistics.average_rssi = (radio_last_rssi()
					+ 7 * (uint16_t) statistics.average_rssi) / 8;
			statistics.receive_count++;

			// we're not waiting for a preamble
			// any more
			Set_transmit_wait(0,0);

			if (len < sizeof(trailer))
			{
				// not a valid packet. We always send
				// two control bytes at the end of every packet
				continue;
			}

			// extract control bytes from end of packet
			memcpy(&trailer, &pbuf[len - sizeof(trailer)], sizeof(trailer));
			len -= sizeof(trailer);
			if(lastSeqNo == trailer.seqNo)																					// if packet is repeated
			{
				errors.rx_errors++;
				//printf("duplicate\n");																								// make it easy to see faults
			}
			else if(trailer.seqNo != (uint8_t)(lastSeqNo+1))
			{
				errors.rx_errors++;
				//printf("missed x%u e%u\n",trailer.seqNo,lastSeqNo+1);																								// make it easy to see faults
			}
			lastSeqNo = trailer.seqNo;
			if (trailer.window == 0 && len != 0)
			{
				// its a control packet
				if (len == sizeof(statistics))
				{
					memcpy(&remote_statistics, pbuf, len);
				}

				// don't count control packets in the stats
				statistics.receive_count--;
			}
			else if (trailer.window != 0)
			{
				// sync our transmit windows based on
				// received header
				int16_t delt;
				uint16_t old = tdm_state_remaining();
				delt = sync_tx_windows(len, RxTick);
				last_t = tnow;
#if 0
				uint16_t remaining = tdm_state_remaining();
				// need to see tick count when rxed,tx.window value, old&new remaining,
				printf("RX%u seq%u rem%u remo%u win%u del%d\n",
						(unsigned )timer2_tick(),
						(unsigned )trailer.seqNo,
						(unsigned )remaining,
						(unsigned )old,
						(unsigned )trailer.window,
						(int )delt);
#else
				(void)delt;
				(void)old;
#endif
				if (trailer.command == 1)
				{
					handle_at_command(len);
				}
				else if (len != 0 && !packet_is_duplicate(len, pbuf, trailer.resend)
						&& !at_mode_active)
				{
					uint8_t out_len = len;
					bool send = true;
					// its user data - send it out
					// the serial port
					//printf("rcv(%d,[", len);
					//LED_ACTIVITY(LED_ON);
					if(aes_get_encryption_level() > 0)
					{
						send = !aes_decrypt(pbuf,len,pbuf,&out_len,lastSeqNo);
					}
					if(send){serial_write_buf(pbuf,out_len);}
					//LED_ACTIVITY(LED_OFF);
					//printf("]\n");
				}
			}
			LED_ACTIVITY(LED_OFF);
			continue;
		}

		// see how many 16usec ticks have passed and update
		// the tdm state machine. We re-fetch tnow as a bad
		// packet could have cost us a lot of time.
		tnow = timer2_tick();
		tdelta = tnow - last_t;
		last_t = tnow;
		maxtdelta = (tdelta > maxtdelta) ? (tdelta) : (maxtdelta);
		tdm_state_update();

		// update link status every 0.5s
		if ((uint16_t)(tnow - last_link_update) > 32768U)
		{
			link_update();
			last_link_update = tnow;
		}

		if (lbt_rssi != 0)
		{
			// implement listen before talk
			if (radio_current_rssi() < lbt_rssi)
			{
				lbt_listen_time += tdelta;
			}
			else
			{
				lbt_listen_time = 0;
				if (lbt_rand == 0)
				{
					lbt_rand = ((uint16_t) rand()) % lbt_min_time;
				}
			}
			if (lbt_listen_time < lbt_min_time + lbt_rand)
			{
				// we need to listen some more
				continue;
			}
		}

		// we are allowed to transmit in our transmit window
		// or in the other radios transmit window if we have
		// bonus ticks
#if USE_TICK_YIELD
		if (tdm_state != TDM_TRANSMIT &&
				!(bonus_transmit && tdm_state == TDM_RECEIVE))
		{
			// we cannot transmit now
			continue;
		}
#else
		if (tdm_state != TDM_TRANSMIT)
		{
			continue;
		}
#endif

		if (transmit_yield != 0)
		{
			// we've give up our window
			continue;
		}

		if (transmit_wait() != 0)
		{
			// we're waiting for a preamble to turn into a packet
			continue;
		}

		if (radio_preamble_detected() || radio_receive_in_progress())
		{
			// a preamble has been detected. Don't
			// transmit for a while
			Set_transmit_wait(packet_latency,timer2_tick());
			continue;
		}

		// sample the background noise when it is out turn to
		// transmit, but we are not transmitting,
		// averaged over around 4 samples
		statistics.average_noise = (radio_current_rssi()
				+ 3 * (uint16_t) statistics.average_noise) / 4;

		if (duty_cycle_wait)
		{
			// we're waiting for our duty cycle to drop
			continue;
		}

		// how many bytes could we transmit in the time we
		// have left?
		if (tdm_state_remaining() < (int32_t)packet_latency)
		{
			// none ....
			continue;
		}
		tickStart = timer2_tick();
		int32_t remaining = tdm_state_remaining();
		remaining = remaining- (int32_t)(packet_latency+TXSetupTicks+TXDelayTicks);	// how many ticks till we actually get to transmit
		if(remaining <= 0)
		{continue;}
		max_xmit = (remaining)/ (ticks_per_byte+1);																	// allow extra ticks/byte  for errors in calculations
		if (max_xmit < PACKET_OVERHEAD)
		{
			// can't fit the trailer in with a byte to spare
			continue;
		}
		max_xmit -= PACKET_OVERHEAD;
    if (aes_get_encryption_level() > 0) {
      if (max_xmit < 16) {
        // With AES, the cipher is up to 16 bytes larger than the text
        // we are encrypting. So we make sure we have sufficient space
        // i.e. min size of any cipher text is 16 bytes
        continue;
      }
      max_xmit -= 1; // there is one byte overhead for encoding
    }

		if (max_xmit > max_data_packet_length)
		{
			max_xmit = max_data_packet_length;
		}

#if PIN_MAX > 0
		// Check to see if any pins have changed state
		pins_user_check();
#endif

		// ask the packet system for the next packet to send
		if (send_at_command && max_xmit >= strlen(remote_at_cmd))
		{
			// send a remote AT command
			len = strlen(remote_at_cmd);
			memcpy(pbuf, remote_at_cmd, len);
			trailer.command = 1;
			send_at_command = false;
		}
		else
		{
			// get a packet from the serial port
			len = packet_get_next(max_xmit, pbuf,seqNo);
			trailer.command = packet_is_injected();
		}

		if (len > max_data_packet_length)
		{
			panic("oversized tdm packet");
		}

		trailer.bonus = (tdm_state == TDM_RECEIVE);
		trailer.resend = packet_is_resend();

		if (tdm_state == TDM_TRANSMIT && len == 0 && send_statistics
				&& max_xmit >= sizeof(statistics))
		{
			// send a statistics packet
			send_statistics = 0;
			memcpy(pbuf, &statistics, sizeof(statistics));
			len = sizeof(statistics);

			// mark a stats packet with a zero window
			trailer.window = 0;
			trailer.resend = 0;
		}
		else
		{
			// calculate the control word as the number of
			// 16usec ticks that will be left in this
			// tdm state after this packet is transmitted
			trailer.window = 1;			// set it correctly later just before sending
		}

		uint16_t temp = timer2_tick();
		// set right transmit channel
		radio_set_channel(fhop_transmit_channel(), false);
		tdelta = (uint16_t) (timer2_tick() - temp);
		maxTicksC = (tdelta > maxTicksC) ? (tdelta) : (maxTicksC);
		//LED_ACTIVITY(LED_ON);																												// show any data going out
		if (len == 0)
		{
			// sending a zero byte packet gives up
			// our window, but doesn't change the
			// start of the next window
			transmit_yield = 1;
		}

		// if we're implementing a duty cycle, add the
		// transmit time to the number of ticks we've been transmitting
		if ((duty_cycle - duty_cycle_offset) != 100)
		{
			transmitted_ticks += flight_time_estimate(len + sizeof(trailer));
		}
		tickEnd = timer2_tick();
		TXDelayTicks = tickEnd-tickStart;
		tickStart = tickEnd;
		// start transmitting the packet
		uint16_t val;
		val = TXSetupTicks + flight_time_estimate(len + sizeof(trailer));// time to send packet and receive across the air
		remaining = tdm_state_remaining();
		if (trailer.window != 0)
		{
			if (remaining <= val)			// if no time left, whoopsie problem!
			{
				trailer.window = 1;
				panic("no time left!");
			}
			else
			{
				trailer.window = remaining - val;// set window remaining to estimated time
			}
		}
		trailer.seqNo = seqNo++;
		memcpy(&pbuf[len], &trailer, sizeof(trailer));
		bool res;
		if (!(res = radio_transmit(len + sizeof(trailer), pbuf,
				remaining ,&tickEnd)) && len != 0
				&& trailer.window != 0 && trailer.command == 0)
		{
			packet_force_resend();
		}
		// after sending a packet leave a bit of time before
		// sending the next one. The receivers don't cope well
		// with back to back packets
		Set_transmit_wait(packet_latency,timer2_tick());
		if(res)
		{
			TXSetupTicks = tickEnd-tickStart;
		}
		else
		{
			errors.tx_errors++;
		}

		tdelta = (uint16_t) (timer2_tick() - tickStart);
		maxTicksTx = (tdelta > maxTicksTx) ? (tdelta) : (maxTicksTx);
#if 0
		printf("TX%u seq%u rem%u remo%u fli%u win%u\n",
				(unsigned )res,
				(unsigned )trailer.seqNo,
				(unsigned )tdm_state_remaining(),
				(unsigned )remaining,
				(unsigned )val,
				(unsigned )trailer.window);
#endif

		if (lbt_rssi != 0)
		{
			// reset the LBT listen time
			lbt_listen_time = 0;
			lbt_rand = 0;
		}

		// set right receive channel
		radio_set_channel(fhop_receive_channel(), false);

		// re-enable the receiver
		TimeFunction(maxTicksOn,radio_receiver_on());
		//received_packet = false;
//		LED_ACTIVITY(LED_OFF);
#if 0
		// If we have any packets that need decrypting lets do it now.
    if(tdm_state_remaining() > (int32_t)(tx_window_width/2))
    {
       // If it is starting to get really full, we want to try decrypting
       // not just one, but a few packets.
       if (encrypt_buffer_getting_full()) {
          while (!encrypt_buffer_getting_empty()) {
            decryptPackets(lastSeqNo);
          }
       } else {
         decryptPackets(lastSeqNo);
       }
    }
#endif
	}
#endif
}

#if 0
/// build the timing table
static void
tdm_build_timing_table(void)
{
	uint8_t j;
	uint16_t rate;
	bool golay_saved = feature_golay;
	feature_golay = false;

	for (rate=2; rate<256; rate=(rate*3)/2)
	{
		uint32_t latency_sum=0, per_byte_sum=0;
		uint8_t size = MAX_PACKET_LENGTH;
		radio_configure(rate);
		for (j=0; j<50; j++)
		{
			uint16_t time_0, time_max, t1, t2;
			radio_set_channel(1,true);
			radio_receiver_on();
			if (serial_read_available() > 0)
			{
				feature_golay = golay_saved;
				return;
			}
			t1 = timer2_tick();
			if (!radio_transmit(0, pbuf, 0xFFFF))
			{
				break;
			}
			t2 = timer2_tick();
			radio_receiver_on();

			time_0 = t2-t1;

			radio_set_channel(2,false);
			t1 = timer2_tick();
			if (!radio_transmit(size, pbuf, 0xFFFF))
			{
				if (size == 0)
				{
					break;
				}
				size /= 4;
				j--;
				continue;
			}

			t2 = timer2_tick();
			radio_receiver_on();

			time_max = t2-t1;
			latency_sum += time_0;
			per_byte_sum += ((size/2) + (time_max - time_0))/size;
		}
		if (j > 0)
		{
			printf("{ %u, %u, %u },\n",
					(unsigned)(radio_air_rate()),
					(unsigned)(latency_sum/j),
					(unsigned)(per_byte_sum/j));
		}
	}
	feature_golay = golay_saved;
}

// test hardware CRC code
static void
crc_test(void)
{
	uint8_t d[4] =
	{	0x01, 0x00, 0xbb, 0xcc};
	uint16_t crc;
	uint16_t t1, t2;
	crc = crc16(4, &d[0]);
	printf("CRC: %x %x\n", crc, 0xb166);
	t1 = timer2_tick();
	crc16(MAX_PACKET_LENGTH/2, pbuf);
	t2 = timer2_tick();
	printf("crc %u bytes took %u 16usec ticks\n",
			(unsigned)MAX_PACKET_LENGTH/2,
			t2-t1);
}

// test golay encoding
static void
golay_test(void)
{
	uint8_t i;
	uint16_t t1, t2;
	uint8_t buf[MAX_PACKET_LENGTH];
	for (i=0; i<MAX_PACKET_LENGTH/2; i++)
	{
		pbuf[i] = i;
	}
	t1 = timer2_tick();
	golay_encode(MAX_PACKET_LENGTH/2, pbuf, buf);
	t2 = timer2_tick();
	printf("encode %u bytes took %u 16usec ticks\n",
			(unsigned)MAX_PACKET_LENGTH/2,
			t2-t1);
	// add an error in the middle
	buf[MAX_PACKET_LENGTH/2] ^= 0x23;
	buf[1] ^= 0x70;
	t1 = timer2_tick();
	golay_decode(MAX_PACKET_LENGTH, buf, pbuf);
	t2 = timer2_tick();
	printf("decode %u bytes took %u 16usec ticks\n",
			(unsigned)MAX_PACKET_LENGTH,
			t2-t1);
	for (i=0; i<MAX_PACKET_LENGTH/2; i++)
	{
		if (pbuf[i] != i)
		{
			printf("golay error at %u\n", (unsigned)i);
		}
	}
}
#endif

// initialise the TDM subsystem
void tdm_init(void)
{
	uint16_t i;
	uint16_t air_rate;
	uint32_t window_width;
	uint32_t freq_min, freq_max;
	uint32_t channel_spacing;
	uint16_t txpower;

	g_board_frequency = calibration_get(CalParam_BAND);
	g_board_frequency =
			(BoardFrequencyValid(g_board_frequency)) ?
					(g_board_frequency) : (FREQ_915);// default to 915, set cal value 31 to change this
	g_board_bl_version = *((uint8_t*)(USERDATA_BASE));	// bl version in user data area

	if (!radio_initialise(param_s_get(PARAM_AIR_SPEED)))
	{
		panic("radio_initialise failed");
	}

	switch (g_board_frequency) {
	case FREQ_868:
		freq_min = 868000000UL;
		freq_max = 869000000UL;
		txpower = 10;
		num_fh_channels = 10;
		break;
	case FREQ_915:
		freq_min = 915000000UL;
		freq_max = 928000000UL;
		txpower = 20;
		num_fh_channels = MAX_FREQ_CHANNELS;
		break;
	default:
		freq_min = 0;
		freq_max = 0;
		txpower = 0;
		//panic("bad board frequency %d", g_board_frequency);
		break;
	}

	if (param_s_get(PARAM_NUM_CHANNELS) != 0)
	{
		num_fh_channels = param_s_get(PARAM_NUM_CHANNELS);
	}
	if (param_s_get(PARAM_MIN_FREQ) != 0)
	{
		freq_min = param_s_get(PARAM_MIN_FREQ) * 1000UL;
	}
	if (param_s_get(PARAM_MAX_FREQ) != 0)
	{
		freq_max = param_s_get(PARAM_MAX_FREQ) * 1000UL;
	}
	if (param_s_get(PARAM_TXPOWER) != 0)
	{
		txpower = param_s_get(PARAM_TXPOWER);
	}

	// constrain power and channels
	txpower = constrain(txpower, BOARD_MINTXPOWER, BOARD_MAXTXPOWER);
	num_fh_channels = constrain(num_fh_channels, 1, MAX_FREQ_CHANNELS);

	// double check ranges the board can do
	switch (g_board_frequency) {
	case FREQ_868:
		freq_min = constrain(freq_min, 849000000UL, 889000000UL);
		freq_max = constrain(freq_max, 849000000UL, 889000000UL);
		break;
	case FREQ_915:
		freq_min = constrain(freq_min, 868000000UL, 935000000UL);
		freq_max = constrain(freq_max, 868000000UL, 935000000UL);
		break;
	default:
		//panic("bad board frequency %d", g_board_frequency);
		break;
	}

	if (freq_max == freq_min)
	{
		freq_max = freq_min + 1000000UL;
	}

	// get the duty cycle we will use
	duty_cycle = param_s_get(PARAM_DUTY_CYCLE);
	duty_cycle = constrain(duty_cycle, 0, 100);
	param_s_set(PARAM_DUTY_CYCLE, duty_cycle);

	// get the LBT threshold we will use
	lbt_rssi = param_s_get(PARAM_LBT_RSSI);
	if (lbt_rssi != 0)
	{
		// limit to the RSSI valid range
		lbt_rssi = constrain(lbt_rssi, 25, 220);
	}
	param_s_set(PARAM_LBT_RSSI, lbt_rssi);

	// sanity checks
	param_s_set(PARAM_MIN_FREQ, freq_min / 1000);
	param_s_set(PARAM_MAX_FREQ, freq_max / 1000);
	param_s_set(PARAM_NUM_CHANNELS, num_fh_channels);

	channel_spacing = (freq_max - freq_min) / (num_fh_channels + 2);

	// add half of the channel spacing, to ensure that we are well
	// away from the edges of the allowed range
	freq_min += channel_spacing / 2;

	// add another offset based on network ID. This means that
	// with different network IDs we will have much lower
	// interference
	srand(param_s_get(PARAM_NETID));
	if (num_fh_channels > 5)
	{
		freq_min += ((unsigned long) (rand() * 625)) % channel_spacing;
	} debug("freq low=%lu high=%lu spacing=%lu\n",
			freq_min, freq_min+(num_fh_channels*channel_spacing),
			channel_spacing);
	// set the frequency and channel spacing
	// change base freq based on netid
	radio_set_frequency(freq_min);																		// 915373412

	// set channel spacing
	radio_set_channel_spacing(channel_spacing);

	// start on a channel chosen by network ID
	radio_set_channel(param_s_get(PARAM_NETID) % num_fh_channels, true);

	// And intilise the radio with them.
	if (!radio_configure()
			&& !radio_configure()
			&& !radio_configure())
	{
		panic("radio_configure failed");
	}

	// report the real air data rate in parameters
	param_s_set(PARAM_AIR_SPEED, radio_air_rate());

	// setup network ID
	radio_set_network_id(param_s_get(PARAM_NETID));

	// setup transmit power
	radio_set_transmit_power(txpower);

	// report the real transmit power in settings
	param_s_set(PARAM_TXPOWER, radio_get_transmit_power());

	// initialise frequency hopping system
	fhop_init(param_s_get(PARAM_NETID));

	air_rate = radio_air_rate();

	// setup boolean features
	feature_mavlink_framing = param_s_get(PARAM_MAVLINK);
	feature_opportunistic_resend = param_s_get(PARAM_OPPRESEND) ? true : false;
	feature_golay = param_s_get(PARAM_ECC) ? true : false;
	feature_rtscts = param_s_get(PARAM_RTSCTS) ? true : false;

#define REGULATORY_MAX_WINDOW (((1000000UL/16)*4)/10)
#define LBT_MIN_TIME_USEC 5000

	// tdm_build_timing_table();

	// calculate how many 16usec ticks it takes to send each byte
	ticks_per_byte = (8 + (8000000UL / (air_rate * 1000UL))) / 16;
	//ticks_per_byte++;

	// calculate the minimum packet latency in 16 usec units
	// we initially assume a preamble length of 40 bits, then
	// adjust later based on actual preamble length. This is done
	// so that if one radio has antenna diversity and the other
	// doesn't, then they will both using the same TDM round timings
	// s=30Km, v=3E8m/s;3E5km/s;30E4km/s , t = s/v=30/30E4=1E-4=100=-6=100uS
	// pream(8):sync(2):ID(2):len(1):data[n]:CRC(2)
	// rest of preamble added in later
	// for 4gfsk (>=500K) the preamble and sync byte are half speed
	// so need to modify for this case
	if(air_rate >= 500)
	{
		packet_latency = (5 + ((8+2)*2)) * ticks_per_byte + ((100 + 8) / 16);
	}
	else
	{
		packet_latency = (5 + 8 + 2) * ticks_per_byte + ((100 + 8) / 16);
	}
	if (feature_golay)
	{
		max_data_packet_length = ((MAX_PACKET_LENGTH-12)/2) - sizeof(trailer);

		// and adds 6 bytes
		packet_latency += 6 * ticks_per_byte;
		// golay encoding doubles the cost per byte
		ticks_per_byte *= 2;

	}
	else
	{
		max_data_packet_length = MAX_PACKET_LENGTH - sizeof(trailer);
	}

	// set the silence period to two times the packet latency
	silence_period = 4 * packet_latency;

	// set the transmit window to allow for 3 full sized packets
	window_width = 3
			* (packet_latency + (max_data_packet_length * (uint32_t) ticks_per_byte));

	if(window_width < 3600)
		window_width = 3600;

	// if LBT is enabled, we need at least 3*5ms of window width
	if (lbt_rssi != 0)
	{
		// min listen time is 5ms
		lbt_min_time = LBT_MIN_TIME_USEC / 16;
		window_width = constrain(window_width, 3 * lbt_min_time, window_width);
	}

	// the window width cannot be more than 0.4 seconds to meet US
	// regulations

	if (window_width >= REGULATORY_MAX_WINDOW && num_fh_channels > 1)
	{
		window_width = REGULATORY_MAX_WINDOW;
	}
	// user specified window is in milliseconds
	if (window_width > param_s_get(PARAM_MAX_WINDOW)*(1000/16))
	{
		window_width = param_s_get(PARAM_MAX_WINDOW)*(1000/16);
	}

	// make sure it fits in the 13 bits of the trailer window
	if (window_width > 0x1fff)
	{
		window_width = 0x1fff;
	}


	tx_window_width = window_width;

	// now adjust the packet_latency for the actual preamble
	// length, so we get the right flight time estimates, while
	// not changing the round timings
	//packet_latency += ((settings.preamble_length - 10) / 2) * ticks_per_byte;

	// tell the packet subsystem our max packet size, which it
	// needs to know for MAVLink packet boundary detection
	i = (tx_window_width - packet_latency) / ticks_per_byte;
	if (i > max_data_packet_length)
	{
		i = max_data_packet_length;
	}
	packet_set_max_xmit(i);

#ifdef TDM_SYNC_LOGIC
	TDM_SYNC_PIN = false;
#endif // TDM_SYNC_LOGIC
	// crc_test();

	// tdm_test_timing();

	// golay_test();
}

/// report tdm timings
///
void tdm_report_timing(void)
{
	printf("silence_period: %u\n", (unsigned )silence_period);
	delay_msec(1);
	printf("tx_window_width: %u\n", (unsigned )tx_window_width);
	delay_msec(1);
	printf("max_data_packet_length: %u\n", (unsigned )max_data_packet_length);
	delay_msec(1);
	printf("packet_latency: %u\n", (unsigned )packet_latency);
	delay_msec(1);
}


// read the tdm time remaining now
static int32_t tdm_state_remaining(void)
{
	uint16_t tn = timer2_tick();
	uint16_t elapsed = (uint16_t) (tdm_end - tn);
	if (elapsed < tdm_ticks)
	{
		return (elapsed);
	}
	else
	{
		return ((int32_t) tdm_end - (int32_t)tn);
	}
}

// set the tdm time remaining relative to a specified system tick count
// return the change in ticks
static int16_t Set_tdm_state_remaining(uint16_t val, uint16_t reltick)
{
	uint16_t lastend = tdm_end;
	tdm_end = val + reltick;
	tdm_ticks = val;
	return((uint16_t)(lastend-reltick) - val);																		// find the amount we changed it by, relative to same point
}


// read the tx wait time remaining now
static int16_t transmit_wait(void)
{
	uint16_t tn = timer2_tick();
	uint16_t elapsed = (uint16_t) (wait_end - tn);
	if (elapsed < wait_ticks)
	{
		return (elapsed);
	}
	else
	{
		return (0);
	}
}

// set the tx wait time remaining relative to a specified system tick count
static void Set_transmit_wait(uint16_t val, uint16_t reltick)
{
	wait_end = val + reltick;
	wait_ticks = val;
}

