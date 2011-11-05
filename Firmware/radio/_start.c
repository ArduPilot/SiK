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
/// @file	_start.c
///
/// Early startup code.
/// This file *must* be linked first for interrupt vector generation and main() to work.
///

#include <stdarg.h>

#include "radio.h"

////////////////////////////////////////////////////////////////////////////////
/// @name	Interrupt vector prototypes
///
/// @note these *must* be placed in this file for SDCC to generate the
/// interrupt vector table correctly.
//@{

/// Serial rx/tx interrupt handler.
///
extern void	serial_interrupt(void)	__interrupt(INTERRUPT_UART0) __using(1);

/// Radio event interrupt handler.
///
extern void	Receiver_ISR(void)	__interrupt(INTERRUPT_INT0);

/// Timer tick interrupt handler
///
/// @todo switch this and everything it calls to use another register bank?
///
static void	T3_ISR(void)		__interrupt(INTERRUPT_TIMER3);

//@}

__code const char g_banner_string[] = "SiK " stringify(APP_VERSION_HIGH) "." stringify(APP_VERSION_LOW) " on " BOARD_NAME;
__code const char g_version_string[] = stringify(APP_VERSION_HIGH) "." stringify(APP_VERSION_LOW);

__pdata enum BoardFrequency	g_board_frequency;	///< board info from the bootloader
__pdata uint8_t			g_board_bl_version;	///< from the bootloader

/// Counter used by delay_msec
///
uint8_t		delay_counter;

/// Configure the Si1000 for operation.
///
static void hardware_init(void);

/// Initialise the radio and bring it online.
///
static void radio_init(void) __reentrant;

void
main(void)
{
	PHY_STATUS	s;

	// Stash board info from the bootloader before we let anything touch
	// the SFRs.
	//
	g_board_frequency = BOARD_FREQUENCY_REG;
	g_board_bl_version = BOARD_BL_VERSION_REG;

	// Do hardware initialisation.
	hardware_init();

	// try to load parameters; set them to defaults if that fails
	// XXX default parameter selection should be based on board info
	//
	if (!param_load())
		param_default();

	// do radio initialisation
	radio_init();

	// turn on the receiver
	s = rtPhyRxOn();
	if (s != PHY_STATUS_SUCCESS)
		panic("rtPhyRxOn failed: %u", s);

	// just a simple transparent serial implementation
	for (;;) {
		uint8_t		rlen;
		__xdata uint8_t	rbuf[64];

		// if we received something via the radio, turn around and send it out the serial port
		if (rtPhyGetRxPacket(&rlen, rbuf) == PHY_STATUS_SUCCESS) {	
			LED_ACTIVITY = LED_ON;
			serial_write_buf(rbuf, rlen);
			LED_ACTIVITY = LED_OFF;
		}

		// if we have received something via serial, transmit it
		rlen = serial_read_available();
		if (rlen > sizeof(rbuf))
			rlen = sizeof(rbuf);
		if (rlen > 0) {
			LED_ACTIVITY = LED_ON;
			serial_read_buf(rbuf, rlen);
			rtPhyTx(rlen, rbuf);
			rtPhyRxOn();
			LED_ACTIVITY = LED_OFF;
		}

		// give the AT command processor a chance to handle a command
		at_command();
	}
}

void
panic(char *fmt, ...)
{
	va_list ap;

	puts("\n**PANIC**");
	va_start(ap, fmt);
	vprintf(fmt, ap);
	puts("");
	for (;;)
		;
}

static void
hardware_init(void)
{
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

	// SPI1
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
	// Derive timer values from SYSCLK, just for laughs.
	TMR3RLL	 = (65536UL - ((SYSCLK / 12) / 100)) & 0xff;
	TMR3RLH	 = ((65536UL - ((SYSCLK / 12) / 100)) >> 8) & 0xff;
	TMR3CN	 = 0x04;	// count at SYSCLK / 12 and start
	EIE1	|= 0x80;

	// UART - set the configured speed
	serial_init(param_get(PARAM_SERIAL_SPEED));

	// global interrupt enable
	EA = 1;

	// Turn on the 'radio running' LED and turn off the bootloader LED
	LED_RADIO = LED_ON;
	LED_BOOTLOADER = LED_OFF;

	XBR2	 =  0x40;		// Crossbar (GPIO) enable
}

static void
radio_init(void)
{
	PHY_STATUS	s;
	uint32_t	freq;

	// Do generic PHY initialisation
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
	rtPhySet(TRX_FREQUENCY,		freq);
	rtPhySet(TRX_CHANNEL_SPACING,	100000UL);	// XXX
	rtPhySet(TRX_DATA_RATE,		 40000UL);	// XXX
	rtPhySet(TRX_DEVIATION,		 param_get(PARAM_DEVIATION)*1000UL);
	rtPhySet(RX_BAND_WIDTH,		 param_get(PARAM_BANDWIDTH)*1000UL);

	// And intilise the radio with them.
	s = rtPhyInitRadio();
	if (s != PHY_STATUS_SUCCESS)
		panic("rtPhyInitRadio failed: %u", s);
}

///
/// Table of supported serial speed settings.
///
const __code U8 serial_baud_rates[][2] = {
	// TH1, CKCON
	{0x96, 0x00},	// B9600
	{0x60, 0x01},	// B19200
	{0xb0, 0x01},	// B38400
	{0x2b, 0x08},	// B57600
	{0x96, 0x08},	// B115200
	{0xcb, 0x08},	// B230400
};

void
serial_device_set_speed(uint8_t speed)
{
	if (speed < BMAX) {
		TH1 = serial_baud_rates[speed][0];
		CKCON = (CKCON & ~0x0b) | serial_baud_rates[speed][1];
	}
}

static void
T3_ISR(void) __interrupt(INTERRUPT_TIMER3)
{

	// re-arm the interrupt
	TMR3CN = 0x04;

	// call the AT parser tick
	at_timer();

	// update the delay counter
	if (delay_counter > 0)
		delay_counter--;
}

void
delay_set(uint16_t msec)
{
	// note that this limits the delay to ~2550ms.
	delay_counter = (msec + 9) / 10;
}

bool
delay_expired()
{
	return delay_counter == 0;
}

void
delay_msec(uint16_t msec)
{
	delay_set(msec);
	while (!delay_expired())
		;
}
