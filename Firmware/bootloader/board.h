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

#ifndef _BOARD_H
#define _BOARD_H

#include <compiler_defs.h>
#include <Si1000_defs.h>

#define BOARD_ID	0x4d	// unique board ID

__at (SFR_P2+0)	LED;		// Bootloader LED
#define LED_ON		0
#define LED_OFF		1

__at (SFR_P0+6)	BUTTON;		// start-in-BL button
#define BUTTON_ACTIVE	0

#define HW_INIT						\
do {							\
	P0SKIP	|=  0x40;		/* button */	\
	P2SKIP	|=  0x01;		/* LED */	\
	SFRPAGE	 =  CONFIG_PAGE;			\
	P2DRV	|=  0x01;		/* LED */	\
	SFRPAGE	 =  LEGACY_PAGE;			\
} while(0)

#endif // _BOARD_H
