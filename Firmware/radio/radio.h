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

// Notes on hardware allocation:
//
// Timer0 is used by rtPhy for its timeouts.
// Timer1 is used by the UART driver.
// Timer3 is used to generate the 10ms timer tick.

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>

#include "board.h"
#include "serial.h"
#include "rtPhy.h"
#include "board_info.h"
#include "parameters.h"
#include "at.h"

// System clock frequency 
#define SYSCLK	24500000UL

// supported serial speeds
enum SerialSpeed
{
    B9600,
    B38400,
    B57600,
    B115200,
    B230400,
    BMAX,
    BNOCHANGE
};

#if DEBUG
# define debug(fmt, args...)	printf_small(fmt "\n", ##args)
#else
# define debug(fmt, args...)
#endif

/// Print a message and halt, largely for debug purposes
///
extern void	panic(char *fmt, ...);

/// Disable interrupts and save their current state
///
#define interrupt_disable(_save)	do { _save = EA; EA = 0; } while(0)

/// Restore saved interrupt state
///
#define interrupt_restore(_save)	do { EA = _save; } while(0)

/// Alternate vprintf implementation
///
extern void	vprintfl(char *fmt, va_list ap) __reentrant;
#define	vprintf(_fmt, _ap)	vprintfl(_fmt, _ap)

/// Alternate printf implementation
///
extern void	printfl(char * fmt, ... ) __reentrant;
#define printf(_fmt, args...)	printfl(_fmt, ##args)

// Macro evil for generating strings
#define __stringify(_x)		#_x
#define stringify(_x)		__stringify(_x)

extern __code const char g_version_string[];			///< printable version string
extern __code const char g_banner_string[];			///< printable startup banner string

extern __pdata enum BoardFrequency	g_board_frequency;	///< board RF frequency from the bootloader
extern __pdata uint8_t			g_board_bl_version;	///< bootloader version

#endif // _RADIO_H_
