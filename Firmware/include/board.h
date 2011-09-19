/* -*- Mode: C; c-basic-offset: 8; -*- */
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

/// @file	board.h
///
/// Board-specific definitions for the bootloader.
///
/// Expected to define:
///
/// BOARD_ID
///	A unique byte identifying this board.
/// LED
///	A __bit controlling an LED.
/// LED_ON
///	The value to write to LED to turn the LED on.
/// LED_OFF
///	The value to write to LED to turn the LED off.
///
/// BUTTON
///	A __bit corresponding to a button or strap that will cause the
///	bootloader to stop and wait for a download.
/// BUTTON_ACTIVE
///	The value that BUTTON will have when the bootloader should stop.
///
/// HW_INIT
///	A code fragment called at early startup time that configures
///	the GPIOs for the LED and button.
///

/// @file	board.h
///		Board definitions for the HopeRF RF50 evaluation board

#ifndef _BOARD_H
#define _BOARD_H

#include <compiler_defs.h>
#include <Si1000_defs.h>

#define BOARD_ID	0x4d	// unique board ID used to connect bootloader and upload tools

// GPIO definitions (not exported)
SBIT(LED_RED,	   SFR_P2, 0);
SBIT(LED_GREEN,	   SFR_P2, 5);
SBIT(BUTTON_ENTER, SFR_P0, 6);
SBIT(BUTTON_UP,	   SFR_P1, 5);
SBIT(BUTTON_DOWN,  SFR_P1, 6);

// Signal polarity definitions
#define LED_ON		0
#define LED_OFF		1
#define BUTTON_ACTIVE	0

// bootloader definitions
#define LED	LED_RED
#define BUTTON	BUTTON_ENTER

// board-specific hardware config
#define HW_INIT						\
do {							\
	P0SKIP	|=  0x40;		/* button */	\
	P1SKIP  |=  0x60;		/* buttons */	\
	P2SKIP	|=  0x21;		/* LEDs */	\
	SFRPAGE	 =  CONFIG_PAGE;			\
	P2DRV	|=  0x21;		/* LED */	\
	SFRPAGE	 =  LEGACY_PAGE;			\
	LED_RED  = 1;					\
	LED_GREEN = 1;					\
} while(0)

// EzRadio / rtPhy definitions
#define EZRADIOPRO_OSC_CAP_VALUE 0xb4	// Per HRF demo code
#define ENABLE_RF_SWITCH		// Per HRF demo code, verified presence of RF switch on the RFM50 module
SBIT(IRQ, SFR_P0, 7);			// Per HRF demo code & schematic
SBIT(NSS1, SFR_P1, 4);			// SI100x Internal Connection
SBIT(SDN, SFR_P2, 6);			// XXX not actually the case on the RFM50... HRF set it this way though

#endif // _BOARD_H
