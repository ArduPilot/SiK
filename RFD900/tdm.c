//TODO check tick timer for overflow errors
//		 add tx slot sharing in
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
#include "ppm.h"
#include "relay.h"

// ******************** defines and typedefs *************************
#define USE_TICK_YIELD 1
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

#define PACKET_OVERHEAD (sizeof(trailer))

/// the state of the tdm system
enum tdm_state {
    TDM_TRANSMIT = 0, TDM_SILENCE1 = 1, TDM_RECEIVE = 2, TDM_SILENCE2 = 3
};
typedef enum {
    Data_Data = 0,
    Data_AT,
    Data_PPM
} DataCmd_t;

// nodeId values for OBC'2016 special case
enum Node_Type {
    NODE_RELAY=0,
    NODE_GCS=1,
    NODE_RETRIEVAL=2
};

struct __attribute__ ((__packed__)) tdm_trailer {
    uint16_t window;                                                        //:13;
    uint8_t command :2;
    uint8_t bonus   :1;
    uint8_t resend  :1;
    uint8_t relayed :1; // packet has been relayed by NODE_RELAY
    uint8_t oddeven :1; // is this an odd or even window
    uint8_t nodeId  :2; // source nodeId
};
struct tdm_trailer trailer;

#define TEMPTEST 0
// ******************** local variables ******************************
#if TEMPTEST
static bool SetActive;
static uint16_t tickClear,tickSet;
#endif
static enum tdm_state tdm_state;

/// a packet buffer for the TDM code
static uint8_t pbuf[(MAX_PACKET_LENGTH*3)/2];

static uint16_t tdm_end;																												// tdm remaining time will be relative to tick counter
static int32_t tdm_ticks;

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
static uint32_t last_received_tick;

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
int16_t duty_cycle;

// this nodeId
uint8_t nodeId;

/*
  odd/even state. Node1 can only transmit in odd state. Node2 can only
  transmit in even state. Node0 can transmit in either state
*/
uint8_t odd_even;

/// the average duty cycle we have been transmitting
static float average_duty_cycle;

/// duty cycle offset due to temperature
int16_t duty_cycle_offset;

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

// time in ticks of last pkt received from node0 (relay)
static uint32_t last_node0_packet;
static bool relay_node_active;

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

/// buffer to hold a remote AT command before sending
static bool send_at_command;
static uint16_t remote_at_len;
// ******************** local function prototypes ********************
static char remote_at_cmd[((AT_CMD_MAXLEN+1)&0xf0)+16];
static int16_t Set_tdm_state_remaining(int32_t val, uint16_t reltick);
static int32_t tdm_state_remaining(uint16_t tn);
static int16_t transmit_wait(void);
static void Set_transmit_wait(uint16_t val, uint16_t reltick);
// ********************* Implementation ******************************

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
    //uint16_t old_remaining = tdm_state_remaining(timer2_tick());
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

    if (nodeId == NODE_RELAY && tdm_state == TDM_TRANSMIT && old_state != TDM_TRANSMIT) {
        // relay node has changed to transmit state, flip odd/even bit
        odd_even ^= 1;
    }

    if(at_testmode & AT_TEST_TDM) {
        uint16_t remaining = tdm_state_remaining(timer2_tick());
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
    bool StateChanged = false;
    // update the amount of time we are waiting for a preamble
    // to turn into a real packet
    // have we passed the next transition point?
    while(tdm_state_remaining(timer2_tick()) <= 0)
    {
        StateChanged = true;
        // advance the tdm state machine
        tdm_state = (tdm_state + 1) % 4;

        if (tdm_state == TDM_TRANSMIT || tdm_state == TDM_RECEIVE)
        {
            Set_tdm_state_remaining((int32_t)tx_window_width+tdm_state_remaining(timer2_tick()),timer2_tick());
        }
        else
        {
            Set_tdm_state_remaining((int32_t)silence_period+tdm_state_remaining(timer2_tick()),timer2_tick());
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
    }
    if(StateChanged)
    {
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

        if (nodeId == NODE_RELAY && tdm_state == TDM_TRANSMIT) {
            // node0 has changed to transmit state, flip odd/even bit
            odd_even ^= 1;
        }

        if (nodeId != NODE_RELAY && tdm_state == TDM_RECEIVE) {
            // lose our odd/even bit
            odd_even = !(nodeId&1);
#if 0
            printf("force odd_even=%d\n", odd_even);
#endif
        }
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
    if(duty_cycle_offset > 100) {
        duty_cycle_offset = 100;
    }
    if(duty_cycle_offset < -100) {
        duty_cycle_offset = -100;
    }
    // limit to minimum of 20% duty cycle to ensure link stays up OK
    if ((duty_cycle - duty_cycle_offset) < 20)
    {
        duty_cycle_offset = duty_cycle - 20;
    }
#if	TEMPTEST
    pins_user_set_value(PINS_USER_P1_1, 1);
    tickSet = timer2_tick();
    SetActive = true;
    tickClear = (((duty_cycle_offset+100)*250)/4);
#endif
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
    if (unlock_count < 1)
    {
        LED_RADIO(LED_ON);
        blink_state = 0;
    }
    else
    {
#ifdef TDM_SYNC_LOGIC
        TDM_SYNC_PIN = false;
#endif // TDM_SYNC_LOGIC
        LED_RADIO(blink_state);
        blink_state = !blink_state;
        fhop_set_locked(false);
#if 0
        uint32_t now = timer32_tick();
        uint32_t tdiff = now - last_received_tick;
        MAVLink_statustext("%u unlocked diff=%u last=%u now=%u",
                           (unsigned)nodeId,
                           (unsigned)tdiff,
                           (unsigned)last_received_tick,
                           (unsigned)now);
#endif
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
            register int16_t old_remaining = tdm_state_remaining(timer2_tick());
            if (tdm_state_remaining(timer2_tick()) > (int32_t)silence_period)
            {
                Set_tdm_state_remaining(tdm_state_remaining(timer2_tick())-(2*(int16_t)packet_latency),timer2_tick());
            }
            else
            {
                Set_tdm_state_remaining(1,timer2_tick());
            }
            if (at_testmode & AT_TEST_TDM)
            {
                printf("TDM: change timing %u/%u\n", (signed )old_remaining,
                       (unsigned )tdm_state_remaining(timer2_tick()));
            }
        }
        if (at_testmode & AT_TEST_TDM)
        {
            printf("TDM: scanning\n");
        }
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

    if (nodeId != NODE_RELAY && timer32_tick() - last_node0_packet > 0xA0000) {
        // relay has not been seen for 10 seconds, go into normal
        // mode
        if (relay_node_active) {
            MAVLink_statustext("relay node dead");
        }
        relay_node_active = false;
    } else {
        relay_node_active = true;
    }
}

// dispatch an AT command to the remote system
void tdm_remote_at(void)
{
    remote_at_len = strlen(at_cmd)+1;																							// length of string + terminator
    if(aes_get_encryption_level() > 0)																						// if encryption enabled
    {   // encrypt and copy into remote at buffer
        aes_encrypt((uint8_t*)at_cmd,remote_at_len,(uint8_t*)remote_at_cmd,&remote_at_len,seqNo);
    }
    else																																					// else no encryption
    {
        memcpy(remote_at_cmd, at_cmd, remote_at_len);																// just copy into remote buffer
    }
    send_at_command = true;																												// set flag for serial loop to send
}

// handle an incoming at command from the remote radio
static void handle_at_command(uint8_t *pbuff, uint16_t len,uint16_t bufflen)
{
    if (len < 2 || len > AT_CMD_MAXLEN || (pbuff[0] != (uint8_t) 'R'&&pbuff[0] != (uint8_t)'X')
            || pbuff[1] != (uint8_t) 'T')
    {
        // assume its an AT command reply
        register uint8_t i;
        for (i = 0; i < len; i++)
        {
            putChar(pbuff[i]);
        }
        return;
    }

    // setup the command in the at_cmd buffer
    memcpy(at_cmd, pbuff, len);
    at_cmd[len] = 0;
    at_cmd[0] = 'A'; // replace 'R'
    at_cmd_len = len;
    at_cmd_ready = true;
    uint8_t cmd = pbuff[0];
    // run the AT command, capturing any output to the packet
    // buffer
    // this reply buffer will be sent at the next opportunity
    printf_start_capture(pbuff, bufflen);
    at_command();
    len = printf_end_capture();
    if ((len > 0)&&('X' != cmd))
    {
        packet_inject(pbuff, len);
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
    {   // do a once loop so continue statements work
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
#if TEMPTEST
        if(SetActive)
        {
            if(((uint16_t) (timer2_tick() - tickSet)) >= tickClear)
            {
                pins_user_set_value(PINS_USER_P1_1, 0);
                SetActive = false;
            }
        }
#endif
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
            uint8_t *buffptr = pbuf;
            tdelta = (uint16_t) (timer2_tick() - tickStart);
            maxTicksX = (tdelta > maxTicksX) ? (tdelta) : (maxTicksX);
            LED_ACTIVITY(LED_ON);

            // update the activity indication
            received_packet = true;
            last_received_tick = timer32_tick();
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

            if (trailer.nodeId == NODE_RELAY) {
                last_node0_packet = last_received_tick;
                relay_node_active = true;
            }

            if (relay_node_active && trailer.nodeId != NODE_RELAY && nodeId != NODE_RELAY && !trailer.relayed) {
                // we will send dup packets between GCS and retrieval, special for OBC-2018
                LED_ACTIVITY(LED_OFF);
                if (trailer.window != 0 && (Data_Data == trailer.command) &&
                        len != 0 && !packet_is_duplicate(len, buffptr, trailer.resend)
                        && !at_mode_active) {
                    serial_write_buf(buffptr, len);
                }
                continue;
            }
            if (trailer.relayed && trailer.nodeId == nodeId) {
                // it is a relayed packet from ourselves. discard
                LED_ACTIVITY(LED_OFF);
                continue;
            }

            // when receiving packets from nodeID 0 the other nodes slave their odd_even bit
            if (trailer.nodeId == NODE_RELAY) {
                odd_even = trailer.oddeven;
            }

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
                uint16_t old = tdm_state_remaining(timer2_tick());
                delt = sync_tx_windows(len, RxTick);
                last_t = tnow;
#if 0
                uint16_t remaining = tdm_state_remaining(timer2_tick());
                // need to see tick count when rxed,tx.window value, old&new remaining,
                printf("RX%u rem%u remo%u win%u del%d\n",
                       (unsigned )timer2_tick(),
                       (unsigned )remaining,
                       (unsigned )old,
                       (unsigned )trailer.window,
                       (int )delt);
#else
                (void)delt;
                (void)old;
#endif
                if (trailer.command == Data_PPM)
                {
                    uint8_t pos;
                    PPMWrite(&pbuf[1],pbuf[0]);																						// write ppm data
                    pos = pbuf[0]+1;																											// set position of following data
                    if(pbuf[pos])																													// if there was any other data attached
                    {
                        len = pbuf[pos];																										// set new length
                        trailer.command = pbuf[pos+1];																			// set data type
                        buffptr = &pbuf[pos+2];																							// set new data ptr
                    }
                }

                uint8_t out_len = len;
                if (trailer.command == Data_AT)
                {
                    bool res=true;
                    if(aes_get_encryption_level() > 0)
                    {
                        res = !aes_decrypt(buffptr,len,buffptr,&out_len,lastSeqNo);
                    }
                    if(res) {
                        handle_at_command(buffptr,out_len,sizeof(pbuf)-(buffptr-pbuf));
                    }
                }
                else if((Data_Data == trailer.command) &&
                        len != 0 && !packet_is_duplicate(len, buffptr, trailer.resend)
                        && !at_mode_active)
                {
                    bool send = true;
                    // its user data - send it out
                    // the serial port
                    //printf("rcv(%d,[", len);
                    //LED_ACTIVITY(LED_ON);
                    if(aes_get_encryption_level() > 0)
                    {
                        send = !aes_decrypt(buffptr,len,buffptr,&out_len,lastSeqNo);
                    }
                    if (send) {
                        serial_write_buf(buffptr,out_len);
                        if (nodeId == NODE_RELAY) {
                            // also queue for retransmit
                            relay_store_packet(buffptr, out_len, trailer.nodeId);
                        }
                    }
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

        if (relay_node_active && nodeId != NODE_RELAY && (nodeId&1) != odd_even) {
            // we are not in our allowed odd/even cycle
            continue;
        }

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
        if (tdm_state_remaining(timer2_tick()) < (int32_t)packet_latency)
        {
            // none ....
            continue;
        }
        tickStart = timer2_tick();
        int32_t remaining = tdm_state_remaining(timer2_tick());
        remaining = remaining- (int32_t)(packet_latency+TXSetupTicks+TXDelayTicks);	// how many ticks till we actually get to transmit
        if(remaining <= 0)
        {
            continue;
        }
        max_xmit = (remaining)/ (ticks_per_byte+1);																	// allow extra ticks/byte  for errors in calculations
        if (max_xmit < PACKET_OVERHEAD)
        {
            // can't fit the trailer in with a byte to spare
            continue;
        }
        max_xmit -= PACKET_OVERHEAD;
        if (max_xmit < 2)
        {
            continue;
        }
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
        static uint16_t last_pin_update;
        if ((uint16_t)(timer2_tick() - last_pin_update) > 200000UL/(16UL*PIN_MAX))  // update every 200mS/number of pins
        {
            pins_user_check();
            last_pin_update = timer2_tick();
        }
#endif

        // default to a non-relayed packet
        trailer.relayed = 0;

        // default to packet coming from our own nodeId
        trailer.nodeId = nodeId;

        // ask the packet system for the next packet to send
        if(ReadPPM(&pbuf[1],&len,max_xmit-2))
        {
            uint8_t dlen;
            pbuf[0] = len;
            if(max_xmit > (len+3))
            {
                pbuf[len+1]  = packet_get_next(max_xmit-(len+3), &pbuf[len+3],seqNo);
                pbuf[len+2]  = packet_is_injected();
            }
            else
            {
                pbuf[len+1]  = 0;
            }
            dlen = pbuf[len+1];
            len += (3+dlen);
            trailer.command = Data_PPM;
        }
        else if (send_at_command && max_xmit >= remote_at_len)
        {
            // send a remote AT command
            len = remote_at_len;
            memcpy(pbuf, remote_at_cmd, len);
            trailer.command = Data_AT;
            send_at_command = false;
        }
        else if (nodeId == NODE_RELAY && serial_read_available() <= relay_bytes_pending())
        {
            uint8_t src_nodeId;
            len = relay_get_packet(pbuf, max_xmit, &src_nodeId);
            if (len > 0) {
                trailer.nodeId = src_nodeId;
                trailer.command = Data_Data;
                trailer.relayed = 1;
            }
        }
        else
        {
            // get a packet from the serial port
            len = packet_get_next(max_xmit, pbuf,seqNo);
            trailer.command = packet_is_injected();
        }

        if (len > max_data_packet_length)
        {
            MAVLink_statustext("oversized tdm packet");
            continue;
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
        remaining = tdm_state_remaining(timer2_tick());
        if (trailer.window != 0)
        {
            if (remaining <= (int32_t)val)			// if no time left, whoopsie problem!
            {
                trailer.window = 1;
                MAVLink_statustext("no time left!");
            }
            else
            {
                trailer.window = remaining - val;// set window remaining to estimated time
            }
        }
        trailer.oddeven = odd_even;
#if 0
        printf("nodeId=%u odd_even=%u command=%u\n",
               (unsigned)trailer.nodeId,
               (unsigned)trailer.oddeven,
               (unsigned)trailer.command);
#endif
        memcpy(&pbuf[len], &trailer, sizeof(trailer));
        bool res;
        if (!(res = radio_transmit(len + sizeof(trailer), pbuf,
                                   remaining,&tickEnd)) && len != 0
                && trailer.window != 0 && trailer.command == Data_Data)
        {
            if (trailer.relayed) {
                relay_save_pkt(pbuf, len, trailer.nodeId);
            } else {
                packet_force_resend();
            }
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
        printf("TX%u rem%u remo%u fli%u win%u\n",
               (unsigned )res,
               (unsigned )tdm_state_remaining(timer2_tick()),
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

    // get our nodeId
    nodeId = param_s_get(PARAM_NODEID);

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

    // add one and a half of the channel spacing, to ensure that we are well
    // away from the edges of the allowed range
    freq_min += ((3*channel_spacing )/ 2);

    // add another offset based on network ID. This means that
    // with different network IDs we will have much lower
    // interference
    srand(param_s_get(PARAM_NETID));
    if (num_fh_channels > 5)
    {
        freq_min += ((unsigned long) (rand() * 625)) % channel_spacing;
    }
    debug("freq low=%lu high=%lu spacing=%lu\n",
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
    uint8_t max_xmit_length = param_s_get(PARAM_MAX_XMIT);
    if (feature_golay)
    {
        max_data_packet_length = ((max_xmit_length-12)/2) - sizeof(trailer);

        // and adds 6 bytes
        packet_latency += 6 * ticks_per_byte;
        // golay encoding doubles the cost per byte
        ticks_per_byte *= 2;

    }
    else
    {
        max_data_packet_length = max_xmit_length - sizeof(trailer);
    }

    // set the silence period to two times the packet latency
    silence_period = 2 * packet_latency;
    if(silence_period < 100) {
        silence_period = 100;
    }

    // set the transmit window to allow for 3 full sized packets
    window_width = 3
                   * (packet_latency + (max_data_packet_length * (uint32_t) ticks_per_byte));

    if(window_width < 3125)	// 50mS
        window_width = 3125;

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

    // for PPM a set widow width is needed
    if (param_s_get(PARAM_RCIN)||param_s_get(PARAM_RCOUT))
    {
        window_width = (20000UL/16);																								// fix window to 20mS
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

    MAVLink_statustext("node %u booted", nodeId);
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
static int32_t tdm_state_remaining(uint16_t tn)
{
    uint16_t elapsed = (uint16_t) (tdm_end - tn);
    if ((int32_t)elapsed < tdm_ticks)
    {
        return (elapsed);
    }
    else
    {
        return ((int16_t)(tdm_end - tn));
    }
}

// set the tdm time remaining relative to a specified system tick count
// return the change in ticks
static int16_t Set_tdm_state_remaining(int32_t val, uint16_t reltick)
{
    uint16_t lastend = tdm_end;
    if(val<0)	{
        tdm_end = reltick - (uint16_t)(-val);
    }
    else 			{
        tdm_end = (uint16_t)val + reltick;
    }
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

