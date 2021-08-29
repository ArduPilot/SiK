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
/// @file	board_hb1060.h
///
/// Board-specific definitions for the RF50-DEMO board.
///

/// @file	board.h
///		Board definitions for the HB1060 board
 
#ifndef _BOARD_HB1060_H
#define _BOARD_HB1060_H

#include <compiler_defs.h>
#include <Si1060_defs.h>

#define BOARD_ID	0x50
#define BOARD_NAME	"HB1060"
#define CPU_SI1060

#define BOARD_MINTXPOWER 0		// Minimum transmit power level
#define BOARD_MAXTXPOWER 0x7f	// Maximum transmit power level

// GPIO definitions (not exported)
SBIT(LED_RED,	   SFR_P1, 4);
SBIT(LED_GREEN,	   SFR_P1, 5);
SBIT(PIN_CONFIG,   SFR_P0, 2);
SBIT(PIN_ENABLE,   SFR_P0, 3);

// Serial flow control
#define SERIAL_RTS	PIN_ENABLE	// always an input
#define SERIAL_CTS	PIN_CONFIG	// input in bootloader, output in app

// Signal polarity definitions
#define LED_ON		0
#define LED_OFF		1
#define BUTTON_ACTIVE	0

// UI definitions
#define LED_BOOTLOADER	LED_RED
#define LED_RADIO		LED_GREEN
#define LED_ACTIVITY	LED_RED
#define BUTTON_BOOTLOAD	PIN_CONFIG

// board-specific hardware config
#define HW_INIT						\
	do {							\
		/* GPIO config */			\
		P0SKIP	|= 0x0E; /* input pins & nIRQ */ \
		P1SKIP  |= 0x30; /* LEDs */ 			\
		P0MDOUT |= 0x80; /* radio SDN */		\
		P1MDOUT |= 0x3D; /* LEDs & radio SPI */ \
												\
		SFRPAGE  = CONFIG_PAGE; 				\
		P1DRV   |= 0x0D;						\
		SFRPAGE  = LEGACY_PAGE;					\
												\
		/* INT0 is the radio interrupt, on P0.1 */ \
		IT01CF = 0x01; /* active low, P0.1 */	\
		IT0	= 0;	/* INT0 level triggered */	\
	} while(0)


#define EZRADIOPRO_OSC_CAP_VALUE 100

SBIT(NSS1, SFR_SPI1CN, 2); // SI106x Internal Connection
SBIT(SDN,  SFR_P0, 7);

#define GPIO_0_CONFIG 33 // RX_STATE
#define GPIO_1_CONFIG 32 // TX_STATE

#endif // _BOARD_HB1060_H
