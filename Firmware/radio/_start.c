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

#include <stdarg.h>

#include "radio.h"

// Interrupt vector prototypes
extern  void uartIsr(void) __interrupt(INTERRUPT_UART0) __using(1);

// Local prototypes
static void hardware_init(void);

void
main(void)
{
	PHY_STATUS	s;

	hardware_init();
	puts("SiK radio starting");

	// Init the radio driver
	s = rtPhyInit();
	if (s != PHY_STATUS_SUCCESS)
		panic("rtPhyInit failed: %u", s);
	puts("radio config done");

	panic("bored, bored bored...");
}

/// Panic and stop the system
///
/// This is not terribly solid - printf is large and it uses putchar which
/// depends on interrupts.  Consider hacking printf_tiny into a panic handler.
///
void
panic(const char *reason, ...)
{
	va_list	arg;

	puts("**PANIC**");
	va_start(arg, reason);
	vprintf(reason, arg);
	va_end(arg);
	puts("");
	for(;;)
		;
}

/// Additional basic hardware intialisation beyond the basic operating conditions
/// set up by the bootloader.
///
static void
hardware_init(void)
{

	// SPI
	XBR1	|= 0x40;	// enable SPI in 3-wire mode
	P1MDOUT	|= 0x15;	// SCK1, MOSI1, MISO1 push-pull
	SFRPAGE	 = CONFIG_PAGE;
	P1DRV	|= 0x15;	// SPI signals use high-current mode
	SFRPAGE	 = LEGACY_PAGE;
	SPI1CFG	 = 0x40;	// master mode
	SPI1CN	 = 0x00;	// 3 wire master mode
	SPI1CKR	 = 0x00;	// initialize SPI prescaler
	SPI1CN	|= 0x01;	// enable SPI
	NSS1 = 1;		// set NSS high

	// Clear the radio interrupt state
	IE0	 = 0;

	// uart - leave the baud rate alone
	uartInitUart(BAUD_RATE_NO_CHANGE);

	// global interrupt enable
	EA = 1;

	// Turn on the 'radio running' LED and turn off the bootloader LED
	LED_RADIO = LED_ON;
	LED_BOOTLOADER = LED_OFF;
}
