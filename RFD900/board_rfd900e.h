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
/// @file	board_rfd900p.h
///
/// Board-specific definitions and documentation for the RFD900P,
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


#ifndef _BOARD_RFD900E
#define _BOARD_RFD900E

#include "em_gpio.h"

// Ensure that the BoardID has the upper most bit set
// This tells the tool chain we are dealing with a CPU_SI1030 device
#define BOARD_ID	 0x80 | 0x3
#define BOARD_NAME	"RFD900E"
#define CPU_EFM32

#define BOARD_MINTXPOWER 0		// Minimum transmit power level
#define BOARD_MAXTXPOWER 30		// Maximum transmit power level


// GPIO definitions (not exported)
typedef enum {
    GPIO_LED_RED		,
    GPIO_LED_GREEN	,
    GPIO_PIN_CONFIG,
    GPIO_PIN_ENABLE,
    GPIO_PIN_CTS,
    GPIO_PIN_RTS,
    GPIO_LAST
} GPIO_SELECT_TypeDef;

typedef struct {
    GPIO_Port_TypeDef Port;
    unsigned int Pin;
} GPIO_Port_Pin_TypeDef;

#define LED_RED				{gpioPortF,11}
#define LED_GREEN			{gpioPortF,10}
#define PIN_CONFIG		{gpioPortC, 6}
#define PIN_ENABLE		{gpioPortD, 8}

#define SERIAL_CTS				{gpioPortC,6}
#define SERIAL_RTS        {gpioPortD,8}

#define P1_0				  {gpioPortD, 2}
#define P1_1				  {gpioPortD, 3}
#define P1_2				  {gpioPortD, 5}
#define P1_3				  {gpioPortD, 4}
#define P3_3				  {gpioPortD, 1}
#define P3_4				  {gpioPortD, 0}


// Signal polarity definitions
#define LED_ON		1				// LED Sense inverted when compared to HM_TRP
#define LED_OFF		0

// UI definitions
#define GPIO_LED_BOOTLOADER	GPIO_LED_RED
#define GPIO_LED_RADIO		GPIO_LED_GREEN
#define GPIO_LED_ACTIVITY	GPIO_LED_RED


#endif // _BOARD_RFD900E
