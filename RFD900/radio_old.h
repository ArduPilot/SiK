// -*- Mode: C; c-basic-offset: 8; -*-
//
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
/// @file	radio.h
///
/// General definitions for the radio application
///

#ifndef _RADIO_OLD_H_
#define _RADIO_OLD_H_

/// @page hardware Notes on Hardware Allocation
///
/// @section timers Timer Allocation
/// @li Timer0 is used by rtPhy for its timeouts.
/// @li Timer1 is used by the UART driver.
/// @li Timer3 is used to generate the 10ms timer tick.

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>
//#include "board_rfd900e.h"

// ***********************typedefs &DEFINES ************************************
// the biggest packet length we will allow. To allow for golay
// encoding this needs to be a multiple of 6
#define MAX_PACKET_LENGTH 252

#ifndef MAX_PA_TEMPERATURE
#define MAX_PA_TEMPERATURE 100
#endif

// Turn on TX/RX Debug pins
//#define DEBUG_PINS_RADIO_TX_RX // TX P1.0 - RX P1.1

#if 0//DEBUG
# define debug(fmt, args...)	printf_small(fmt "\n", ##args)
#else
# define debug(fmt, args...)
#endif

// useful macro for array sizes
#define ARRAY_LENGTH(a) (sizeof(a)/sizeof(a[0]))

// Macro evil for generating strings
#define __stringify(_x)		#_x
#define stringify(_x)		__stringify(_x)

/// System clock frequency
///
/// @todo This is standard for the Si1000 if running off the internal
///       oscillator, but we should have a way to override it.
//#define SYSCLK	48000000UL

/// staticstics maintained by the radio code
typedef struct{
	uint8_t average_rssi;
	uint8_t average_noise;
	uint16_t receive_count;
} statistics_t;


typedef struct {
	uint16_t rx_errors;		///< count of packet receive errors
	uint16_t tx_errors;		///< count of packet transmit errors
	uint16_t serial_tx_overflow;    ///< count of serial transmit overflows
	uint16_t serial_rx_overflow;    ///< count of serial receive overflows
	uint16_t corrected_errors;      ///< count of words corrected by golay code
	uint16_t corrected_packets;     ///< count of packets corrected by golay code
} error_counts_t;

typedef struct{
	uint32_t frequency;
	uint32_t channel_spacing;
	uint8_t air_data_rate;
	uint8_t current_channel;
	uint8_t transmit_power;
	uint8_t preamble_length; // in nibbles
	uint16_t networkID;
} radio_settings_t;

enum BoardFrequency {
        //FREQ_433	= 0x43,
        //FREQ_470	= 0x47,
        FREQ_868	= 0x86,
        FREQ_915	= 0x91,
        FREQ_NONE	= 0xf0,
};

#define BoardFrequencyValid(a) ((a==FREQ_868)||(a==FREQ_915))

// ******************************** externs blurg ******************************

/// optional features
extern bool feature_golay;
extern bool feature_opportunistic_resend;
extern uint8_t feature_mavlink_framing;
extern bool feature_rtscts;

extern statistics_t statistics, remote_statistics;
extern error_counts_t errors;

// Board infop
extern const char 		g_version_string[];	///< printable version string
extern const char 		g_banner_string[];	///< printable startup banner string
extern enum BoardFrequency	g_board_frequency;	///< board RF frequency from the bootloader
extern uint8_t			g_board_bl_version;	///< bootloader version

extern radio_settings_t settings;

// ******************************** function prototypes ************************

/// Print a message and halt, largely for debug purposes
/// @param	fmt		printf-style format string and argments
///				to be printed.
void	panic(char *fmt, ...);


/// receives a packet from the radio
///
/// @param len			Pointer to storage for the length of the packet
/// @param buf			Pointer to storage for the packet
/// @return			True if a packet was received
///
extern bool radio_receive_packet(uint16_t *length, uint8_t *  buf,uint16_t * Tick);

/// test whether the radio has detected a packet preamble
/// @return			True if a preamble has been detected
extern bool radio_preamble_detected(void);

/// begin transmission of a packet
/// @param length		Packet length to be transmitted; assumes
///				the data is already present in the FIFO.
/// @param timeout_ticks	The number of ticks to wait before assiming
///				that transmission has failed.
/// @return			true if packet sent successfully
///
extern bool radio_transmit(uint8_t length, uint8_t *  buf,  uint16_t timeout_ticks,uint16_t *TxTick);

/// switch the radio to receive mode
/// @return			Always true.
extern bool radio_receiver_on(void);

/// reset and intiialise the radio
/// @return			True if the initialisation completed successfully.
extern bool radio_initialise(void);

/// set the nominal radio transmit/receive frequencies
/// This is the frequency of channel zero.
/// @param value		The frequency in Hz
extern bool radio_set_frequency( uint32_t value);

/// set the channel spacing used by the channel offset control
/// @param value		The channel spacing in Hz
extern bool radio_set_channel_spacing( uint32_t value);

/// set the channel for transmit/receive
/// @param value		The channel number to select
extern void radio_set_channel(uint8_t channel, bool RX);

/// get the tx/rx frequency channel
/// @return			the current channel
extern uint8_t radio_get_channel(void);

/// configure the radio for a given air data rate
/// @param air_rate		The air data rate, in bits per second
///				Note that this value is rounded up to
///				the next supported value
/// @return			True if the radio was successfully configured.
///
extern bool radio_configure( uint16_t air_rate);

/// configure the radio network ID
/// The network ID is programmed as header bytes, so that packets for
/// other networks can be rejected at the hardware level.
/// @param id			The network ID to be sent, and to filter
///				on reception
extern void radio_set_network_id(uint16_t id);

/// fetch the signal strength recorded for the most recent preamble
///
/// @return			The RSSI register as reported by the radio
///				the last time a valid preamble was detected.
extern uint8_t radio_last_rssi(void);

/// fetch the current signal strength for LBT
///
/// @return			The RSSI register as reported by the radio
///
extern uint8_t radio_current_rssi(void);

/// return the air data rate
///
/// @return			The value passed to the last successful call
///				to radio_configure
///
extern uint8_t radio_air_rate(void);

/// set the radio transmit power (in dBm)
///
/// @param power		The desired transmit power in dBm
///				
///
extern void radio_set_transmit_power(uint8_t power);

/// set the radio transmit power (in dBm)
///
/// @param increment	change the radio power up (true) or down (false)
/// @param maxPower		The maximum transmit power in dBm
/// @return						The actual transmit power in dBm
///
///
extern uint8_t radio_change_transmit_power(bool increment, uint8_t maxPower);

/// get the currend transmit power (in dBm)
///
/// @return			The actual transmit power in dBm
///				
///
extern uint8_t radio_get_transmit_power(void);

/// check if a packet is coming in
///
/// @return			true if a packet is being received
///
///
extern bool radio_receive_in_progress(void);

/// send a MAVLink status report packet
void MAVLink_report(void);

/// return temperature in degrees C
///
/// @return		temperature in degrees C, from 0 to 127
///
extern int16_t radio_temperature(void);

// maximum temperature we allow the radio to get to before
// we start limiting the duty cycle

extern void radio_set_diversity(bool enable);

void radio_daemon(void);

#endif // _RADIO_H_
