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
/// Flash-related data structures and functions, including the application
/// signature for the bootloader.
///

#include <stdint.h>
#include <flash_layout.h>

#include "radio.h"

// The application signature block.
//
// The presence of this block, which is the first thing to be erased and the
// last thing to be programmed during an update, tells the bootloader that
// a valid application is installed.
//
__at(FLASH_SIGNATURE_BYTES) uint8_t __code app_signature[2] = { FLASH_SIG0, FLASH_SIG1 };

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
flash_erase_scratch(void)
__critical {
	// erase the scratch page
#ifdef CPU_SI1030 // Part of memory no seperate scratch section
	__pdata uint8_t	bank_state = PSBANK;
	PSBANK = ((bank_state & 0x03) | 0x30); // Select Bank 3
#endif // CPU_SI1030
	
	flash_load_keys();				// unlock flash for one operation
	PSCTL = FLASH_ERASE_SCRATCH;	// enable flash erase of the scratch page
	*(uint8_t __xdata *)FLASH_SCRATCH = 0xff;	// trigger the erase
	PSCTL = FLASH_DISABLE;			// disable flash write & scratch access
	
#ifdef CPU_SI1030
	PSBANK = bank_state;
#endif // CPU_SI1030
}

uint8_t
flash_read_scratch(__pdata uint16_t address)
__critical {
	uint8_t	d;

#ifdef CPU_SI1030 // Part of memory no seperate scratch section
	__pdata uint8_t	bank_state = PSBANK;
	PSBANK = ((bank_state & 0x03) | 0x30); // Select Bank 3
#endif // CPU_SI1030
	
	PSCTL = FLASH_READ_SCRATCH;
	d = *(uint8_t __code *)(FLASH_SCRATCH | address);
	PSCTL = FLASH_DISABLE;
	
#ifdef CPU_SI1030
	PSBANK = bank_state;
#endif // CPU_SI1030
	return d;
}

void
flash_write_scratch(__pdata uint16_t address, __pdata uint8_t c)
__critical {
#ifdef CPU_SI1030 // Part of memory no seperate scratch section
	__pdata uint8_t	bank_state = PSBANK;
	PSBANK = ((bank_state & 0x03) | 0x30); // Select Bank 3
#endif // CPU_SI1030
	
	flash_load_keys();
	PSCTL = 0x05;
	*(uint8_t __xdata *)(FLASH_SCRATCH | address) = c;
	
#ifdef CPU_SI1030
	PSBANK = bank_state;
#endif // CPU_SI1030
	PSCTL = FLASH_DISABLE;
}
