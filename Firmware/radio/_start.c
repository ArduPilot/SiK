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

/// @file	_start.c
///
/// Early startup code.
///
/// This file *must* be linked first for interrupt vector generation and main() to work.
///

#include "radio.h"
#include "board.h"
#include "uart.h"

// Interrupt vector prototypes
extern  void uartIsr(void) __interrupt(INTERRUPT_UART0) __using(1);

// Local prototypes
static void hardware_init(void);

void
main(void)
{
	hardware_init();
	puts("SiK radio");

	for (;;)
		;
}

/// Additional hardware intialisation beyond the basic operating conditions
/// set up by the bootloader.
///
static void
hardware_init(void)
{
	// clocking - bootloader uses internal oscillator, prescaled by 1, missing clock
	//            detector is already enabled.
	// 		This is already max speed, and should be good enough precision for UART.

	// timer1 - bootloader has already configured for 115200, leave it alone for now

	// brownout detector - bootloader has enabled

	// crossbar - buttons and LEDs already configured by bootloader

	// uart - leave the baud rate alone
	uartInitUart(BAUD_RATE_NO_CHANGE);

	// global interrupt enable
	EA = 1;
}
