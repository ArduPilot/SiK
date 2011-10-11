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
//
// Note that these *must* be placed in this file for SDCC to generate the
// interrupt vector table correctly.
//
extern void	uartIsr(void)		__interrupt(INTERRUPT_UART0) __using(1);
extern void	Receiver_ISR(void)	__interrupt(INTERRUPT_INT0);
extern void	T0_ISR(void)		__interrupt(INTERRUPT_TIMER0);
static void	T3_ISR(void)		__interrupt(INTERRUPT_TIMER3);

__code const char g_banner_string[] = "SiK " stringify(APP_VERSION_HIGH) "." stringify(APP_VERSION_LOW) " on " BOARD_NAME;
__code const char g_version_string[] = stringify(APP_VERSION_HIGH) "." stringify(APP_VERSION_LOW);

// board info from the bootloader
__pdata enum BoardFrequency	g_board_frequency;
__pdata uint8_t			g_board_bl_version;

// Local prototypes
static void hardware_init(void);
static void radio_init(void);

void
main(void)
{
	PHY_STATUS	s;

	// Stash board info from the bootloader before we let anything touch
	// the SFRs.
	//
	g_board_frequency = BOARD_FREQUENCY_REG;
	g_board_bl_version = BOARD_BL_VERSION_REG;

	// Do hardware initialisation
	hardware_init();

	// try to load parameters; set them to defaults if that fails
	// XXX default parameter selection should be based on strapping
	// options
	if (!param_load())
		param_default();

	// do radio initialisation
	radio_init();

	puts(g_banner_string);

	// turn on the receiver
	s = rtPhyRxOn();
	if (s != PHY_STATUS_SUCCESS)
		panic("rtPhyRxOn failed: %u", s);

	for (;;) {
		uint8_t		rlen;
		__xdata uint8_t	rbuf[64];

		if (rtPhyGetRxPacket(&rlen, rbuf) == PHY_STATUS_SUCCESS) {
			LED_ACTIVITY = LED_ON;
			rtPhyTx(rlen, rbuf);
			LED_ACTIVITY = LED_OFF;
			rtPhyRxOn();
			printf("pkt %d 0x%02x\n", rlen, rbuf[0]);
		}

		if (iskey()) {
			uint8_t	c = getchar();
			at_input(c);
		}
	}
}

/// Panic and stop the system
///
void
_panic()
{
	puts("\n**PANIC**");
	for(;;)
		;
}

/// Do basic hardware initialisation.
///
static void
hardware_init(void)
{
#if 0
	uint16_t	i;

	// Disable the watchdog timer
	PCA0MD	&= ~0x40;

	// Select the internal oscillator, prescale by 1
	FLSCL	 =  0x40;
	OSCICN	 =  0x8F;
	CLKSEL	 =  0x00;

	// Configure the VDD brown out detector
	VDM0CN	 =  0x80;
	for (i = 0; i < 350; i++);	// Wait 100us for initialization
	RSTSRC	 =  0x06;		// enable brown out and missing clock reset sources

	// Configure crossbar for UART
	P0MDOUT	 =  0x10;		// UART Tx push-pull
	SFRPAGE	 =  CONFIG_PAGE;
	P0DRV	 =  0x10;		// UART TX
	SFRPAGE	 =  LEGACY_PAGE;
	XBR0	 =  0x01;		// UART enable
#endif

	// SPI
	XBR1	|= 0x40;	// enable SPI in 3-wire mode
	P1MDOUT	|= 0x15;	// SCK1, MOSI1, MISO1 push-pull
	SFRPAGE	 = CONFIG_PAGE;
	P1DRV	|= 0x15;	// SPI signals use high-current mode
	SFRPAGE	 = LEGACY_PAGE;
	SPI1CFG	 = 0x40;	// master mode
	SPI1CN	 = 0x00;	// 3 wire master mode
	SPI1CKR	 = 0x00;	// Initialise SPI prescaler to divide-by-2 (12.25MHz, technically out of spec)
	SPI1CN	|= 0x01;	// enable SPI
	NSS1	 = 1;		// set NSS high

	// Clear the radio interrupt state
	IE0	 = 0;

	// 100Hz timer tick using timer3
	// Derive timer values from SYSCLK
	TMR3RLL	 = 0x40; /* (65536 - ((SYSCLK / 12) / 100)) & 0xff */
	TMR3RLH	 = 0xb0; /* ((65536 - ((SYSCLK / 12) / 100)) >> 8) & 0xff; */
	TMR3CN	 = 0x04;	// count at SYSCLK / 12 and start
	EIE1	|= 0x80;

	// uart - leave the baud rate alone
	uartInitUart(BAUD_RATE_NO_CHANGE);
	uartSetUartOption(UART_TRANSLATE_EOL, 1);

	// global interrupt enable
	EA = 1;

	// Turn on the 'radio running' LED and turn off the bootloader LED
	LED_RADIO = LED_ON;
	LED_BOOTLOADER = LED_OFF;

//	XBR2	 =  0x40;		// Crossbar (GPIO) enable
}

static void
radio_init(void) __reentrant
{
	PHY_STATUS	s;
	uint32_t	freq;

	// Do generic PHY initialisation
	//
	s = rtPhyInit();
	if (s != PHY_STATUS_SUCCESS)
		panic("rtPhyInit failed: %u", s);

	switch (g_board_frequency) {
	case FREQ_433:
		freq = 433000000UL;
		break;
	case FREQ_470:
		freq = 470000000UL;
		break;
	case FREQ_868:
		freq = 868000000UL;
		break;
	case FREQ_915:
		freq = 915000000UL;
		break;
	default:
		panic("bad board frequency %d", g_board_frequency);
		freq = 0;	// keep the compiler happy
		break;
	}

	// Set PHY parameters for the initial operational state
	//
	rtPhySet(TRX_FREQUENCY,		freq);
	rtPhySet(TRX_CHANNEL_SPACING,	100000UL);	// XXX
	rtPhySet(TRX_DEVIATION,		 35000UL);	// XXX
	rtPhySet(TRX_DATA_RATE,		 40000UL);	// XXX
	rtPhySet(RX_BAND_WIDTH,		 10000UL);	// XXX

	// And intilise the radio with them.
	s = rtPhyInitRadio();
	if (s != PHY_STATUS_SUCCESS)
		panic("rtPhyInitRadio failed: %u", s);
}

/// Timer tick interrupt handler
///
/// XXX Could switch this and everything it calls to use another register bank?
///
static void
T3_ISR(void) __interrupt(INTERRUPT_TIMER3)
{
	/* re-arm the interrupt */
	TMR3CN = 0x04;

	/* call the AT parser tick */
	at_timer();
}
