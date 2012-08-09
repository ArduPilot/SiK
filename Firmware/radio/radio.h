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

#ifndef _RADIO_H_
#define _RADIO_H_

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

// the biggest packet length we will allow. To allow for golay
// encoding this needs to be a multiple of 6
#define MAX_PACKET_LENGTH 252


#include "board.h"
#include "serial.h"
#include "board_info.h"
#include "parameters.h"
#include "at.h"
#include "flash.h"
#include "rtc.h"

// canary data for ram wrap
extern __pdata uint8_t pdata_canary;

/// optional features
extern bool feature_golay;
extern bool feature_opportunistic_resend;
extern bool feature_mavlink_framing;
extern bool feature_rtscts;

/// System clock frequency
///
/// @todo This is standard for the Si1000 if running off the internal
///       oscillator, but we should have a way to override it.
#define SYSCLK	24500000UL

#if DEBUG
# define debug(fmt, args...)	printf_small(fmt "\n", ##args)
#else
# define debug(fmt, args...)
#endif

// useful macro for array sizes
#define ARRAY_LENGTH(a) (sizeof(a)/sizeof(a[0]))

/// Print a message and halt, largely for debug purposes
///
/// @param	fmt		printf-style format string and argments
///				to be printed.
///
extern void	panic(char *fmt, ...);


/// Alternate vprintf implementation
///
extern void	vprintfl(const char * fmt, va_list ap) __reentrant;
#define	vprintf(_fmt, _ap)	vprintfl(_fmt, _ap)		///< avoid fighting with the library vprintf() prototype

/// Alternate printf implementation
///
extern void	printfl(const char *fmt, ...) __reentrant;
#define printf(_fmt, args...)	printfl(_fmt, ##args)		///< avoid fighting with the library printf() prototype

/// start a capture of printf data 
extern void printf_start_capture(__xdata uint8_t *buf, uint8_t size);

/// end printf capture, returning number of bytes that have been captured
extern uint8_t printf_end_capture(void);

// Macro evil for generating strings
#define __stringify(_x)		#_x
#define stringify(_x)		__stringify(_x)

// Board infop
extern __code const char 		g_version_string[];	///< printable version string
extern __code const char 		g_banner_string[];	///< printable startup banner string
extern __pdata enum BoardFrequency	g_board_frequency;	///< board RF frequency from the bootloader
extern __pdata uint8_t			g_board_bl_version;	///< bootloader version

/// staticstics maintained by the radio code
struct statistics {
	uint8_t average_rssi;
	uint8_t average_noise;
	uint16_t receive_count;
};
__pdata extern struct statistics statistics, remote_statistics;

struct error_counts {
	uint16_t rx_errors;		///< count of packet receive errors
	uint16_t tx_errors;		///< count of packet transmit errors
	uint16_t serial_tx_overflow;    ///< count of serial transmit overflows
	uint16_t serial_rx_overflow;    ///< count of serial receive overflows
	uint16_t corrected_errors;      ///< count of words corrected by golay code
	uint16_t corrected_packets;     ///< count of packets corrected by golay code
};
__pdata extern struct error_counts errors;

/// receives a packet from the radio
///
/// @param len			Pointer to storage for the length of the packet
/// @param buf			Pointer to storage for the packet
/// @return			True if a packet was received
///
extern bool radio_receive_packet(uint8_t *length, __xdata uint8_t * __pdata buf);

/// test whether the radio has detected a packet preamble
///
/// @return			True if a preamble has been detected
///
extern bool radio_preamble_detected(void);

/// begin transmission of a packet
///
/// @param length		Packet length to be transmitted; assumes
///				the data is already present in the FIFO.
/// @param timeout_ticks	The number of ticks to wait before assiming
///				that transmission has failed.
///
/// @return			true if packet sent successfully
///
extern bool radio_transmit(uint8_t length, __xdata uint8_t * __pdata buf, __pdata uint16_t timeout_ticks);

/// switch the radio to receive mode
///
/// @return			Always true.
///
extern bool radio_receiver_on(void);

/// reset and intiialise the radio
///
/// @return			True if the initialisation completed successfully.
///
extern bool radio_initialise(void);

/// set the nominal radio transmit/receive frequencies
///
/// This is the frequency of channel zero.
///
/// @param value		The frequency in Hz
///
extern bool radio_set_frequency(__pdata uint32_t value);

/// set the channel spacing used by the channel offset control
///
/// @param value		The channel spacing in Hz
///
extern bool radio_set_channel_spacing(__pdata uint32_t value);

/// set the channel for transmit/receive
///
/// @param value		The channel number to select
///
extern void radio_set_channel(uint8_t channel);

/// get the tx/rx frequency channel
///
/// @return			the current channel
///
extern uint8_t radio_get_channel(void);

/// configure the radio for a given air data rate
///
/// @param air_rate		The air data rate, in bits per second
///				Note that this value is rounded up to
///				the next supported value
/// @return			True if the radio was successfully configured.
///
extern bool radio_configure(__pdata uint8_t air_rate);

/// configure the radio network ID
///
/// The network ID is programmed as header bytes, so that packets for
/// other networks can be rejected at the hardware level.
///
/// @param id			The network ID to be sent, and to filter
///				on reception
///
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

struct radio_settings {
	uint32_t frequency;
	uint32_t channel_spacing;
	uint8_t air_data_rate;
	uint8_t current_channel;
	uint8_t transmit_power;
	uint8_t preamble_length; // in nibbles
};

extern __pdata struct radio_settings settings;

/// return temperature in degrees C
///
/// @return		temperature in degrees C, from 0 to 127
///
extern int16_t radio_temperature(void);

// maximum temperature we allow the radio to get to before
// we start limiting the duty cycle
#ifndef MAX_PA_TEMPERATURE
#define MAX_PA_TEMPERATURE 100
#endif

extern void radio_set_diversity(bool enable);

#endif // _RADIO_H_
