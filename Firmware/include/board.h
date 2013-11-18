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
/// @file	board.h
///
/// Includes the board-specific configuration header.
///
/// The following options may (or if required, should) be defined
///
/// BOARD_ID		[required]
///	A unique byte identifying the board.
///
/// BOARD_NAME		[required]
///	A short text string giving the name of the board.
///
/// LED_BOOTLOADER	[required]
///	A __bit controlling an LED for the bootloader.  The bootloader
///	will light this LED while waiting for input.
///
/// LED_RADIO		[required]
///     A __bit controlling an LED for the radio.  The radio will light
///	this LED while running.
///
/// LED_ACTIVITY	[required]
///	A __bit controlling an LED that can be blinked to show activity.
///
/// LED_ON		[required]
///	The value to write to an LED to turn the LED on.
///
/// LED_OFF		[required]
///	The value to write to an LED to turn the LED off.
///
/// BUTTON_BOOTLOAD	[required]
///	A __bit corresponding to a button or strap that will cause the
///	bootloader to stop and wait for a download.
///
/// BUTTON_ACTIVE	[required]
///	The value that BUTTON will have when the bootloader should stop.
///
/// HW_INIT		[required]
///	A code fragment called at early bootloader startup that configures
///	the SoC for board-specific operation.
///	- configures LED GPIO(s)
///	- configures button GPIO(s)
///	- configures INT0 for the radio interrupt
///
/// HW_INIT_APPLICATION	[optional]
///	A code fragment called at application startup time to adjust any
///	settings that vary between the bootloader and application.
///
/// EZRADIOPRO_OSC_CAP_VALUE	[required]
///     Radio oscillator load capacitor value.
///
/// ENABLE_RFM50_SWITCH		[optional]
///	Assume the RF switch is configured like the HopeRF RFM50
///	XXX this should be generalised
///
/// SERIAL_RTS		[optional]
///	A __bit for output flow control.  If this bit is set, the
///	serial device at the other end is ready for data.
///
/// SERIAL_CTS		[optional]
///	A __bit for input flow control.  This bit can be cleared
///	to indicate to the serial device at the other end that
///	the local serial buffer is nearly full.
///

#ifndef _BOARD_H_
#define _BOARD_H_

#if   defined(BOARD_rf50)
# include "board_rf50.h"
#elif defined(BOARD_hm_trp)
# include "board_hm_trp.h"
#elif defined(BOARD_rfd900)
# include "board_rfd900.h"
#elif defined(BOARD_rfd900a)
# include "board_rfd900a.h"
#elif defined(BOARD_rfd900u)
# include "board_rfd900u.h"
#else
# error Must define a BOARD_ value before including this file.
#endif

#endif // _BOARD_H_
