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
/// @file	board_rfd900u.h
///
/// Board-specific definitions and documentation for the RFD900U,
/// Version 1.2 onwards.
/// 
/// The RFD900 board provides pads for programming
/// the Si1000 via the debug port.
/// The pads are on the single horizontal header on the bottom of the board.
/// Pin 1 = GND, Pin 2 = +5V, Pin 3 = C2D, Pin 4 = C2CK
///
/// The SiLabs programmer has a 10-pin ribbon cable, for which you will
/// need to make an adapter.  The User Guide, linked from the page here:
///
/// http://www.silabs.com/products/mcu/Pages/USBDebug.aspx
/// describes the pinout of the cable.
///
/// Connect the SiLabs debug adapter to the RFD900 V1.2+ as follows:
///
/// Debug Adapter Pin:                 	RFD900A V1.2+ 9W header pin (J2)
///
///        2 <--------------------------> GND 	(Pin 2)
///        4 <--------------------------> C2D 	(Pin 4)
///        7 <--------------------------> C2CK	(Pin 5)
///       10 <--------------------------> +5V	(Pin 3)
///
///
/// If you are making your own adapter board for the RFD900, note that
/// whilst the stock firmware requires the ENABLE pin be tied low,
/// it is a flow control input to the SiK radio firmware.


#ifndef _BOARD_RFD900U
#define _BOARD_RFD900U

#include <compiler_defs.h>
#include <Si1020_defs.h>

#define BOARD_ID	  0x44
#define BOARD_NAME	"RFD900U"
#define CPU_SI1030

#define BOARD_MINTXPOWER 0		// Minimum transmit power level
#define BOARD_MAXTXPOWER 30		// Maximum transmit power level

//#define WATCH_DOG_ENABLE

// GPIO definitions (not exported)
SBIT(LED_RED,	   SFR_P3, 6);
SBIT(LED_GREEN,	   SFR_P3, 7);
SBIT(PIN_CONFIG,   SFR_P0, 2);
SBIT(PIN_ENABLE,   SFR_P0, 3);
//SBIT(PA_ENABLE,    SFR_P2, 5);         // Power Amplifier Enable


// Signal polarity definitions
#define LED_ON		1				// LED Sense inverted when compared to HM_TRP
#define LED_OFF		0
#define BUTTON_ACTIVE	0

// UI definitions
#define LED_BOOTLOADER	LED_RED
#define LED_RADIO		LED_GREEN
#define LED_ACTIVITY	LED_RED
#define BUTTON_BOOTLOAD	PIN_CONFIG

// Serial flow control
//#define SERIAL_RTS	PIN_ENABLE	// always an input
//#define SERIAL_CTS	PIN_CONFIG	// input in bootloader, output in app

// board-specific hardware config
#define HW_INIT							\
	do { \
		SFRPAGE  = CONFIG_PAGE; \
		P3MDOUT |= 0x40;		/* Led Red */ \
		P3DRV   |= 0x40;		/* Led Red */ \
		SFRPAGE  = LEGACY_PAGE;	\
		/* Setup Timers */ \
		TMOD	 = (TMOD & ~0xf0) | 0x20;		/* TMOD: timer 1 in 8-bit autoreload */ \
		TR1		 = 1;			/* START Timer1 */ \
		TI0		 = 1;			/* Indicate TX0 ready */ \
		\
		P2       = 0xFF;                    /* P2 bug fix for SDCC and Raisonance*/ \
	} while(0)

// application/board-specific hardware config
#define HW_INIT_APPLICATION					\
	do {							\
		SFRPAGE	 =  CONFIG_PAGE;			\
		P0DRV	|=    0x04;		/* CTS */	\
		SFRPAGE	 =  LEGACY_PAGE;			\
	} while(0)

// Radio Definitions

#define EZRADIOPRO_OSC_CAP_VALUE 0xB6 // Measured on RFD900 V1.1
#define ENABLE_RFD900_SWITCH 1        // Define RF switches on the module (V1.1 are V1.2 the same)
#define RFD900_DIVERSITY 1            // Enable/Disable diversity on RFD900 (V1.1 are V1.2 the same)
SBIT(IRQ,  SFR_P0, 7);                // Connection within RFD900 module, P0.7 is connected to nIRQ
SBIT(NSS1, SFR_P2, 3);                // SI1020 Internal Connection

#endif // _BOARD_RFD900U
