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
/// @file	board_hm_trp.h
///
/// Board-specific definitions for the HM-TRP board.
///
/// Expected to define:
///
/// BOARD_ID
///	A unique byte identifying this board.
///
/// LED_BOOTLOADER
///	A __bit controlling an LED for the bootloader.
///
/// LED_RADIO
///     A __bit controlling an LED for the radio.
///
/// LED_ACTIVITY
///	A __bit controlling an LED that can be blinked to show activity.
///
/// LED_ON
///	The value to write to LED to turn LEDs on.
///
/// LED_OFF
///	The value to write to LED to turn LEDs off.
///
/// BUTTON_BOOTLOAD
///	A __bit corresponding to a button or strap that will cause the
///	bootloader to stop and wait for a download.
///
/// BUTTON_ACTIVE
///	The value that BUTTON_bootload will have when the bootloader should stop.
///
/// SYSCLK
///	Frequency of the system clock in MHz.
///
/// HW_INIT
///	A code fragment called at early startup time that configures
///	the SoC for board-specific operation.
///	- configures LED GPIO(s)
///	- configures button GPIO(s)
///	- configures INT0 for the radio interrupt
///

#ifndef _BOARD_HM_TRP_H_
#define _BOARD_HM_TRP_H_

#include <compiler_defs.h>
#include <Si1000_defs.h>

// GPIO definitions (not exported)
SBIT(LED_RED,	   SFR_P1, 6);
SBIT(LED_GREEN,	   SFR_P1, 5);
SBIT(PIN_CONFIG,   SFR_P0, 2);
SBIT(PIN_ENABLE,   SFR_P0, 3);

// Signal polarity definitions
#define LED_ON		0
#define LED_OFF		1
#define BUTTON_ACTIVE	0

// UI definitions
#define LED_BOOTLOADER	LED_RED
#define LED_RADIO	LED_GREEN
#define LED_ACTIVITY	LED_RED
#define BUTTON_BOOTLOAD	PIN_CONFIG

// board-specific hardware config
#define HW_INIT						\
	do {							\
		/* GPIO config */				\
		P0SKIP	|=  0x0c;		/* pins */	\
		P1SKIP  |=  0x60;		/* LEDs */	\
		SFRPAGE	 =  CONFIG_PAGE;			\
		P1DRV	|=  0x60;		/* LEDs */	\
		SFRPAGE	 =  LEGACY_PAGE;			\
		/* INT0 is the radio interrupt, on P0.7 */	\
		IT01CF   =  (IT01CF & 0xf) | 0x7;		\
		IT0	 = 0;	/* INT0 level triggered */	\
	} while(0)


// EzRadio / rtPhy definitions
// Note that the HM-TRP deviates from SiLabs' appnotes in the wiring of the RF switch
//
#define EZRADIOPRO_OSC_CAP_VALUE 0xb4	// XXX cribbed from RF50, probably wrong
#define ENABLE_RFM50_SWITCH	1	// verified presence of RF switch on the module
SBIT(IRQ,  SFR_P0, 7);			// Per board inspection
SBIT(NSS1, SFR_P1, 4);			// SI100x Internal Connection
SBIT(SDN,  SFR_P2, 6);			// XXX not actually the case on the HM-TRP... HRF set it this way on the RF50

#endif // _BOARD_HM_TRP_H_
