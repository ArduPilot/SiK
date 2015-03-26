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
#include <board_info.h>

#include <stdint.h>

#include "board.h"
#include "flash.h"
#include "util.h"

// Place all code in the high page, for banking this would be in Bank 3
#pragma codeseg HIGHCSEG

/// Signature bytes at the very end of the application space.
///
/// These must be supplied by the application as part of the uploaded image,
/// they should be programmed last.
///
__at(FLASH_SIGNATURE_BYTES) __code uint8_t flash_signature[2];

/// Patchbay for the board frequency byte.
/// This is patched in the hex file(s) after building.
///
__at(FLASH_FREQUENCY_BYTE) __code uint8_t board_frequency = FREQ_NONE;

/// Lock byte
///
/// We explicitly initialise the lock byte to prevent code in the high
/// page from overwriting it, and clear the LSB to lock the first page
/// of flash.  This has the side-effect of locking the high page as well;
/// combined this means that the bootloader code cannot be overwritten.
/// RFD900A locks the bootloader as a separate step after calibration instead.
///
#if !defined BOARD_rfd900a && !defined BOARD_rfd900p
volatile __at(FLASH_LOCK_BYTE) __code uint8_t flash_lock_byte = 0xfe;
#endif

char
flash_app_valid(void)
{
	return (flash_signature[0] == FLASH_SIG0) && (flash_signature[1] == FLASH_SIG1);
}

/// Tests whether an address is in the visible (app) range
///
/// @param	address		The address to be tested
/// @returns			True if the address is visible
///
static bool
#ifdef FLASH_BANKS
flash_address_visible(uint32_t address)
{
	switch (address >> 16) {
		// Make sure we dont address the top of the flash
		case 3:
			if ((address & 0xFFFF) >= FLASH_SCRATCH)
				return false;
		// Banks 1,2 are always good above 0x7FFF
		case 2:
		case 1:
			if ((address & 0xFFFF) < 0x8000)
				return false;
			break;
		// Home Bank shouldn't be writing to the bootloader..
		case 0:
			if ((address & 0xFFFF) < FLASH_APP_START || (address & 0xFFFF) > 0x7FFF)
				return false;
			break;
		default:
			return false;
	}
	return true;
}
#else // FLASH_BANKS
flash_address_visible(uint16_t address)
{
	if ((address < FLASH_APP_START) || (address >= FLASH_INFO_PAGE))
		return false;
	return true;
}
#endif // FLASH_BANKS

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
#ifdef FLASH_BANKS
	uint16_t	greaterAddress;
	uint8_t		bank;
	uint8_t		bank_state = PSBANK;
	for (bank=FLASH_BANKS; bank>0; bank--) {
		// Change to the correct Bank..
		PSBANK = ((bank_state & 0x03) | (bank<<4)); // Set IFBANK to current value and COBANK to the erase page..
		
		switch (bank) {
			case 3:
				address = FLASH_SCRATCH - FLASH_PAGE_SIZE;
				greaterAddress = 0x8000;
				break;
			case 2:
				address = 0xFFFF;
				greaterAddress = 0x8000;
				break;
			case 1:
				address = 0xFFFF;
				greaterAddress = FLASH_APP_START;
				break;
			default:
				address = 1;
				greaterAddress = 0;
				break;
		}
		
		// start with the signature so that a partial erase will fail the signature check on startup
		for (; address >= greaterAddress; address -= FLASH_PAGE_SIZE) {
			flash_load_keys();
			PSCTL = 0x03;				// set PSWE and PSEE
			*(uint8_t __xdata *)address = 0xff;	// do the page erase
			PSCTL = 0x00;				// disable PSWE/PSEE
		}
	}
	// Restore Prev State
	PSBANK = bank_state;
#else // FLASH_BANKS
	// start with the signature so that a partial erase will fail the signature check on startup
	for (address = FLASH_INFO_PAGE - FLASH_PAGE_SIZE; address >= FLASH_APP_START; address -= FLASH_PAGE_SIZE) {
		flash_load_keys();
		PSCTL = 0x03;				// set PSWE and PSEE
		*(uint8_t __xdata *)address = 0xff;	// do the page erase
		PSCTL = 0x00;				// disable PSWE/PSEE
	}
#endif // FLASH_BANKS
}

void
flash_erase_scratch(void)
{
#ifdef FLASH_BANKS // The Banked version doesn't have a scratch page
	flash_load_keys();		// unlock flash for one operation
	PSCTL = 0x03;			// set PSWE and PSEE
	*(uint8_t __xdata *)FLASH_SCRATCH = 0xff;	// do the page erase
	PSCTL = 0x00;			// disable PSWE/PSEE
#else // FLASH_BANKS
	// erase the scratch page
	flash_load_keys();		// unlock flash for one operation
	PSCTL = 0x07;			// enable flash erase of the scratch page
	*(uint8_t __xdata *)0 = 0xff;	// trigger the erase
	PSCTL = 0x00;			// disable flash write & scratch access
#endif // FLASH_BANKS
}

// If we have banking support, change to 32bit addressing..

void
#ifdef FLASH_BANKS
flash_write_byte(uint32_t address, uint8_t c)
{
	uint8_t	bank_state = PSBANK;

	// If Address is above the bondary for a normal page
	// and is on the home page (page 0) move to bank 3 to prevent code corruption
	// Used when not compiling with banking support
	if(((address & 0xFFFF) > 0x7FFF) && ((address & 0xFFFF) < 0xFFFF))
	{
		address |= 0x30000;
	}

	if (flash_address_visible(address)) {
		// Set IFBANK to current value and COBANK to the erase page.
		if((address>>16) == 0)
			PSBANK = ((bank_state & 0x03) | 0x10);
		else
			// We need to place the new address into the second byte, thus moving it down 16 then up 4 and masking
			PSBANK = ((bank_state & 0x03) | ((address>>12) & 0xF0));
		flash_load_keys();
		PSCTL = 0x01;				// set PSWE, clear PSEE
		*(uint8_t __xdata *)((uint16_t)address) = c;	// write the byte
		PSCTL = 0x00;				// disable PSWE/PSEE
		
		// Restore Prev State
		PSBANK = bank_state;
	}
}
#else // FLASH_BANKS
flash_write_byte(uint16_t address, uint8_t c)
{
	if (flash_address_visible(address)) {
		flash_load_keys();
		PSCTL = 0x01;				// set PSWE, clear PSEE
		*(uint8_t __xdata *)address = c;	// write the byte
		PSCTL = 0x00;				// disable PSWE/PSEE
	}
}
#endif // FLASH_BANKS

uint8_t
#ifdef FLASH_BANKS
flash_read_byte(uint32_t address)
{
	uint8_t	bank_state = PSBANK;
	uint8_t c;
	
	// Used when not compiling with banking support
	if(((address & 0xFFFF) > 0x7FFF) && ((address & 0xFFFF) < 0xFFFF))
	{
		address |= 0x30000;
	}
	
	if ((address>>16) <= FLASH_BANKS) {
	//if(flash_address_visible(address)) {
		// Set IFBANK to current value and COBANK to the erase page.
		if((address>>16) == 0)
			PSBANK = ((bank_state & 0x03) | 0x10);
		else
			// We need to place the new address into the second byte, thus moving it down 16 then up 4 and masking
			PSBANK = ((bank_state & 0x03) | ((address>>12) & 0xF0));
		c = *(uint8_t __code *)((uint16_t)address);
		
		// Restore Prev State
		PSBANK = bank_state;
		return c;
	}
	return 0xFF;
}
#else // FLASH_BANKS
flash_read_byte(uint16_t address)
{
	return *(uint8_t __code *)address;
}
#endif // FLASH_BANKS


#if defined BOARD_rfd900a || defined BOARD_rfd900p
__at(FLASH_CALIBRATION_AREA_HIGH) uint8_t __code calibration[FLASH_CALIBRATION_AREA_SIZE];
__at(FLASH_CALIBRATION_CRC_HIGH)	uint8_t __code calibration_crc;

void
flash_transfer_calibration()
{
	uint8_t idx, crc = 0;

	// ensure the user area (plus crc byte) is all 0xFF
	for (idx = 0; idx < FLASH_CALIBRATION_AREA_SIZE; idx++)
	{
		if (flash_read_byte(FLASH_CALIBRATION_AREA + idx) != 0xFF)
		{
			return;
		}
	}
	if (flash_read_byte(FLASH_CALIBRATION_CRC) != 0xFF)
	{
		return;
	}

	// ensure valid data is available
	for (idx = 0; idx < FLASH_CALIBRATION_AREA_SIZE; idx++)
	{
		crc ^= calibration[idx];
	}
	if (crc != calibration_crc)
	{
		return;
	}

	// transfer calibration data from bootloader area to user area
	for (idx = 0; idx < FLASH_CALIBRATION_AREA_SIZE; idx++)
	{
		flash_write_byte((FLASH_CALIBRATION_AREA + idx), calibration[idx]);
	}
	flash_write_byte(FLASH_CALIBRATION_CRC, calibration_crc);
}
#endif //BOARD_rfd900a/p
