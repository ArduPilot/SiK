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
/// @file	flash.c
///
/// Flash memory handling
///

#include <compiler_defs.h>
#include <Si1000_defs.h>
#include <board_info.h>

#include <stdint.h>

#include "flash.h"
#include "util.h"

#if 0
# define trace(_x)	cout(_x);
#else
# define trace(_x)
#endif

// Place all code in the high page
//
#pragma codeseg HIGHCSEG

/// Signature bytes at the very end of the application space.
///
/// These must be supplied by the application as part of the uploaded image,
/// they should be programmed last.
///
__at(FLASH_SIGNATURE_BYTES) __code uint8_t flash_signature[2];

/// Lock byte
///
/// We explicitly initialise the lock byte to prevent code in the high
/// page from overwriting it.
///
__at(FLASH_LOCK_BYTE) __code uint8_t flash_lock_byte = 0xff;

/// Patchbay for the board frequency byte.
/// This is patched in the hex file(s) after building.
///
__at(FLASH_FREQUENCY_BYTE) __code uint8_t board_frequency = FREQ_NONE;

char
flash_app_valid(void)
{
	trace(flash_signature[0]);
	trace(flash_signature[1]);
	return (flash_signature[0] == FLASH_SIG0) && (flash_signature[1] == FLASH_SIG1);
}

/// Tests whether an address is in the visible (app) range
///
/// @param	address		The address to be tested
/// @returns			True if the address is visible
///
static bool
flash_address_visible(uint16_t address)
{
	if ((address < FLASH_APP_START) || (address >= FLASH_INFO_PAGE))
		return false;
	return true;
}

/// Load the write-enable keys into the hardware in order to enable
/// one write or erase operation.
///
static void
flash_load_keys(void)
{
	FLKEY = 0xa5;
	FLKEY = 0xf1;
}

void
flash_erase_app(void)
{
	uint16_t	address;

	// start with the signature so that a partial erase will fail the signature check on startup
	for (address = FLASH_INFO_PAGE - FLASH_PAGE_SIZE; address >= FLASH_APP_START; address -= FLASH_PAGE_SIZE) {
		flash_load_keys();
		PSCTL = 0x03;				// set PSWE and PSEE
		*(uint8_t __xdata *)address = 0xff;	// do the page erase
		PSCTL = 0x00;				// disable PSWE/PSEE
	}
}

void
flash_erase_scratch(void)
{
	// erase the scratch page
	flash_load_keys();		// unlock flash for one operation
	PSCTL = 0x07;			// enable flash erase of the scratch page
	*(uint8_t __xdata *)0 = 0xff;	// trigger the erase
	PSCTL = 0x00;			// disable flash write & scratch access
}

void
flash_write_byte(uint16_t address, uint8_t c)
{
	if (flash_address_visible(address)) {
		flash_load_keys();
		PSCTL = 0x01;				// set PSWE, clear PSEE
		*(uint8_t __xdata *)address = c;	// write the byte
		PSCTL = 0x00;				// disable PSWE/PSEE
	}
}

uint8_t
flash_read_byte(uint16_t address)
{
	if (flash_address_visible(address)) {
		return *(uint8_t __code *)address;
	}
	return 0xff;
}

