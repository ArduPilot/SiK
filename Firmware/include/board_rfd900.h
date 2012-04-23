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
/// @file	board_rfd900.h
///
/// Board-specific definitions and documentation for the RFD900 board.
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
/// Connect the SiLabs debug adapter to the RFD900 as follows:
///
/// Debug Adapter Pin:                 		RFD900 pin
///
///        2 <--------------------------> GND 	(Pin 1)
///        4 <--------------------------> C2D 	(Pin 3)
///        7 <--------------------------> C2CK	(Pin 4)
///       10 <--------------------------> +5V		(Pin 2)
///
///
///
/// If you are making your own adapter board for the RFD900, note that
/// whilst the stock firmware requires the ENABLE pin be tied low,
/// it is a flow control input to the SiK radio firmware.
///
/// Also, the CONFIG pin is a flow control output, and a series resistor
/// of at least 33 ohms must be placed in series with it to avoid damage
/// to the Si1000 when it is jumpered low to force bootloader entry.
///

#ifndef _BOARD_RFD900
#define _BOARD_RFD900

#include <compiler_defs.h>
#include <Si1000_defs.h>

#define BOARD_ID	  0x42
#define BOARD_NAME	"RFD900"

#define BOARD_MINTXPOWER 17		// Minimum transmit power level
#define BOARD_MAXTXPOWER 30		// Maximum transmit power level

// GPIO definitions (not exported)
SBIT(LED_RED,	     SFR_P1, 6);
SBIT(LED_GREEN,	   SFR_P1, 5);
SBIT(PIN_CONFIG,   SFR_P0, 2);
SBIT(PIN_ENABLE,   SFR_P0, 3);

// Signal polarity definitions
#define LED_ON		1				// LED Sense inverted when compared to HM_TRP
#define LED_OFF		0
#define BUTTON_ACTIVE	0

// UI definitions
#define LED_BOOTLOADER	LED_RED
#define LED_RADIO	      LED_GREEN
#define LED_ACTIVITY	  LED_RED
#define BUTTON_BOOTLOAD	PIN_CONFIG

// Serial flow control
#define SERIAL_RTS	PIN_ENABLE	// always an input
#define SERIAL_CTS	PIN_CONFIG	// input in bootloader, output in app

// board-specific hardware config
#define HW_INIT							\
	do {							\
		/* GPIO config */				\
		P0SKIP	|=  0x0C;		/* pins */	\
		P1SKIP  |=  0xE0;		/* LEDs 60*/	\
		SFRPAGE	 =  CONFIG_PAGE;			  \
		P1MDOUT	|= 0xF5;	/* SCK1, MOSI1, MISO1 push-pull was 60 */ \
		P1DRV	|= 0xF5;	/* SPI signals use high-current mode, LEDs and PAEN High current drive was 60 */ \
		SFRPAGE	 =  LEGACY_PAGE;			  \
		/* INT0 is the radio interrupt, on P0.7 */	\
		IT01CF   =  (IT01CF & 0xf) | 0x7;		\
		IT0	 = 0;	/* INT0 level triggered */	\
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
#define ENABLE_RFD900_SWITCH 1        // Define RF switches on the module
#define RFD900_DIVERSITY 1            // Enable/Disable diversity on RFD900
SBIT(IRQ,  SFR_P0, 7);                // Connection within RFD900 module, P0.7 is connected to nIRQ
SBIT(NSS1, SFR_P1, 4);                // SI100x Internal Connection

#endif // _BOARD_RFD900
