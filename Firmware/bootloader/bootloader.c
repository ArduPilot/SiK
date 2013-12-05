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
/// @file	bootloader.c
///
/// A UART bootloader for the SiLabs Si1000 SoC.
///
/// Protocol inspired by the STK500 protocol by way of Arduino.
///

#pragma codeseg HOME

#include <compiler_defs.h>
#include <stdio.h>
#include <stdint.h>

#include "board.h"
#include "bootloader.h"
#include "board_info.h"
#include "flash.h"
#include "util.h"

// Send the default 'in sync' response.
//
static void	sync_response(void);

// Initialise the Si1000 and board hardware
//
static void	hardware_init(void);

// Bootloader protocol handler
//
static void	bootloader(void);


uint8_t __data	buf[PROTO_PROG_MULTI_MAX];

uint8_t		reset_source;
uint8_t		debounce_count;
bool		app_valid;

// Bootloader entry logic
//
void
bl_main(void)
{
	uint8_t		i;
#ifdef FLASH_BANKS
	uint8_t		bank_state = PSBANK;
#endif
	
	// Do early hardware init
	hardware_init();

	// Switch the page to bank 3
#ifdef FLASH_BANKS
	PSBANK = 0x33;
#endif
	
	// Work out why we reset
	//
	// Note that if PORSF (bit 1) is set, all other bits are invalid.
	reset_source = RSTSRC;
	if (reset_source & (1 << 1))
		reset_source = 1 << 1;

	// Check for app validity
	app_valid = flash_app_valid();

	// Set the button to have a pullup on this pin
	BUTTON_BOOTLOAD = ~BUTTON_ACTIVE;
	
	// Do some simple debouncing on the bootloader-entry
	// strap/button.
	debounce_count = 0;
	for (i = 0; i < 255; i++) {
		if (BUTTON_BOOTLOAD == BUTTON_ACTIVE)
			debounce_count++;
	}

	// Turn on the LED to indicate the bootloader is running
	LED_BOOTLOADER = LED_ON;

	// Boot the application if:
	//
	// - The reset was not due to a flash error (the app uses this to reboot
	//   into the bootloader)
	// - The signature is valid.
	// - The boot-to-bootloader strap/button is not in the active state.
	//
	if (!(reset_source & (1 << 6)) && app_valid) {

		// The button was not entirely pressed - play it safe
		// and boot the app.
		//
		if (debounce_count < 200) {
#ifdef BOARD_rfd900a
			// make sure the calibration is available to the application
			flash_transfer_calibration();
#endif
			// transfer

			// Stash board info in SFRs for the application to find later
			//
			BOARD_FREQUENCY_REG = board_frequency;
			BOARD_BL_VERSION_REG = BL_VERSION;
#ifdef FLASH_BANKS
			// Restore Banking Info
			PSBANK = bank_state;
#endif

			// And jump
			((void (__code *)(void))FLASH_APP_START)();
		}
	}

	// Bootloader loop
	//
	for (;;)
		bootloader();
}

// Bootloader protocol logic
//
static void
bootloader(void)
{
	uint8_t		c;
	uint8_t		count, i;
#ifdef CPU_SI1030
	static uint32_t	address;
#else // FLASH_BANKS
	static uint16_t	address;
#endif

	// Wait for a command byte
	LED_BOOTLOADER = LED_ON;
	c = cin();
	LED_BOOTLOADER = LED_OFF;

	// common tests for EOC
	switch (c) {
	case PROTO_GET_SYNC:
	case PROTO_GET_DEVICE:
	case PROTO_CHIP_ERASE:
	case PROTO_PARAM_ERASE:
	case PROTO_READ_FLASH:
	case PROTO_DEBUG:
		if (cin() != PROTO_EOC)
			goto cmd_bad;
	}

	switch (c) {

	case PROTO_GET_SYNC:		// sync
		break;

	case PROTO_GET_DEVICE:
		cout(BOARD_ID);
		cout(board_frequency);
		break;

	case PROTO_CHIP_ERASE:		// erase the program area
		flash_erase_app();
		break;

	case PROTO_PARAM_ERASE:
		flash_erase_scratch();
		break;

	case PROTO_LOAD_ADDRESS:	// set address
		address = cin();
		address |= (uint16_t)cin() << 8;
#ifdef CPU_SI1030
		address |= (uint32_t)cin() << 16;
#endif // FLASH_BANKS
		if (cin() != PROTO_EOC)
			goto cmd_bad;
		break;

	case PROTO_PROG_FLASH:		// program byte
		c = cin();
		if (cin() != PROTO_EOC)
			goto cmd_bad;
		flash_write_byte(address++, c);
		break;

	case PROTO_READ_FLASH:		// readback byte
		c = flash_read_byte(address++);
		cout(c);
		break;

	case PROTO_PROG_MULTI:
		count = cin();
		if (count > sizeof(buf))
			goto cmd_bad;
		for (i = 0; i < count; i++)
			buf[i] = cin();
		if (cin() != PROTO_EOC)
			goto cmd_bad;
		for (i = 0; i < count; i++)
			flash_write_byte(address++, buf[i]);
		break;

	case PROTO_READ_MULTI:
		count = cin();
		if (cin() != PROTO_EOC)
			goto cmd_bad;
		for (i = 0; i < count; i++) {
			c = flash_read_byte(address++);
			cout(c);
		}
		break;

	case PROTO_REBOOT:
		// generate a software reset, which should boot to the application
		RSTSRC |= (1 << 4);

	case PROTO_DEBUG:
		// XXX reserved for ad-hoc debugging as required
		break;

	default:
		goto cmd_bad;
	}
	sync_response();
cmd_bad:
	return;
}

static void
sync_response(void)
{
	cout(PROTO_INSYNC);	// "in sync"
	cout(PROTO_OK);		// "OK"
}

// Do minimal hardware init required for bootloader
//
static void
hardware_init(void)
{
	int i = 0;

	SFRPAGE = LEGACY_PAGE;
	
	// Disable interrupts - we run with them off permanently
	// as all interrupts vector to the application.
	EA	 =  0x00;

	// Disable the watchdog timer
	PCA0MD	&= ~0x40;

	// Select the internal oscillator, prescale by 1
	FLSCL	 =  0x40;
#ifdef CPU_SI1030
	OSCICN	 |=	0x80;
#else
	OSCICN	 =	0x8F;
#endif
	CLKSEL	 =  0x00;

	// Configure Timers
	TCON	 =  0x40;		// Timer1 on
	TMOD	 =  0x20;		// Timer1 8-bit auto-reload
	CKCON	 =  0x08;		// Timer1 from SYSCLK
	TH1		 =  0x96;		// 115200 bps

	// Configure UART
	SCON0	 =  0x12;		// enable receiver, set TX ready

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

	// Hardware-specific init for LED and button
	HW_INIT;

	XBR2	 =  0x40;		// Crossbar (GPIO) enable
}
