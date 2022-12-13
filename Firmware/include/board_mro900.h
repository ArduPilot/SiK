// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2011 Michael Smith, All Rights Reserved
// Copyright (c) 2012 Seppo Saario, All Rights Reserved
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
//
///
/// @file	board_mro900.h
///
/// Board-specific definitions and documentation for the MRO900

#ifndef _BOARD_mro900
#define _BOARD_mro900

#include <compiler_defs.h>
#include <Si1020_defs.h>

// Ensure that the BoardID has the upper most bit set
// This tells the tool chain we are dealing with a CPU_SI1030 device
#define BOARD_ID	 0x80 | 0x11
#define BOARD_NAME	"MRO900"
#define CPU_SI1030

#define BOARD_MINTXPOWER 20		// Minimum transmit power level
#define BOARD_MAXTXPOWER 30		// Maximum transmit power level

//#define WATCH_DOG_ENABLE

// GPIO definitions (not exported)
SBIT(LED_RED,      SFR_P3, 6);
SBIT(LED_GREEN,    SFR_P3, 7);
SBIT(PIN_CONFIG,   SFR_P0, 2);
SBIT(PIN_ENABLE,   SFR_P0, 3);
SBIT(PA_ENABLE,    SFR_P2, 5);         // Power Amplifier Enable
SBIT(LNA_ENABLE,   SFR_P2, 6);

// Signal polarity definitions
#define LED_ON		1				// LED Sense inverted when compared to HM_TRP
#define LED_OFF		0
#define BUTTON_ACTIVE	0
#define LNA_ON      0
#define LNA_OFF     1

// UI definitions
#define LED_BOOTLOADER	LED_RED
#define LED_RADIO		LED_GREEN
#define LED_ACTIVITY	LED_RED
#define BUTTON_BOOTLOAD	PIN_CONFIG

// Serial flow control
#define SERIAL_RTS	PIN_ENABLE	// always an input
#define SERIAL_CTS	PIN_CONFIG	// input in bootloader, output in app

// board-specific hardware config
#define HW_INIT							\
	do { \
		/* GPIO config */				\
		P2MDOUT |= 0x60;        /* PA_ENABLE (P2.5) & LNA_ENABLE output */           \
		SFRPAGE  = CONFIG_PAGE; \
		P3MDOUT |= 0x40;		/* Led Red */ \
		P3DRV   |= 0x40;		/* Led Red */ \
		P2DRV 	|= 0x60;	    /* PA_ENABLE (P2.5) & LNA_ENABLE high current drive*/ \
		SFRPAGE  = LEGACY_PAGE;	\
		/* Setup Timers */ \
		TMOD	 = (TMOD & ~0xf0) | 0x20; /* TMOD: timer 1 in 8-bit autoreload */ \
		TR1		 = 1;			/* START Timer1 */ \
		TI0		 = 1;			/* Indicate TX0 ready */ \
		/* INT0 is the radio interrupt, on P0.1 */ \
		IT01CF	 = (IT01CF & 0xf) | 0x01;\
		IT0		 = 0;			/* INT0 level triggered */ \
		P2		 = 0xFF;		/* P2 bug fix for SDCC and Raisonance*/ \
	} while(0)

// application/board-specific hardware config
#define HW_INIT_APPLICATION					\
	do {							\
		SFRPAGE	 =  CONFIG_PAGE;			\
		P0DRV	|=    0x04;		/* CTS */	\
		SFRPAGE	 =  LEGACY_PAGE;			\
	} while(0)

// Radio Definitions

#define EZRADIOPRO_OSC_CAP_VALUE 0x6A // Measured on MRO900 V1.1
#define MRO900_DIVERSITY 1            // Enable/Disable diversity on MRO900
#define TEMP_OFFSET 45                // Use the internal offset register with this extra cal offset
SBIT(IRQ,  SFR_P0, 1);                // Connection within MRO900 module, P0.1 is connected to nIRQ
SBIT(NSS1, SFR_P2, 3);                // SI1020 Internal Connection

#endif // _BOARD_MRO900