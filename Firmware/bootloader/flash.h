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
/// @file	flash.h
///
/// Flash memory related defines and prototypes.
///

#ifndef _FLASH_H_
#define _FLASH_H_

#include <stdint.h>
#include <stdbool.h>

#include <flash_layout.h>

/// Board frequency code, patched into the bootloader at build time.
///
extern __at(FLASH_FREQUENCY_BYTE) __code uint8_t board_frequency;

/// Checks to see whether the flash contains a valid application.
///
/// @returns	Nonzero if there is a valid application loaded.
///
char	flash_app_valid(void);

/// Erases the application from flash, starting with the page containing
/// the signature.
///
void	flash_erase_app(void);

/// Erases the scratch page, where applications keep their parameters.
///
void	flash_erase_scratch(void);

/// Writes a byte to flash.
///
/// @param	address		The address at which to write the byte
/// @param	c		The byte to write
/// @returns			True if the byte can be written
///
void	flash_write_byte(uint16_t address, uint8_t c);

/// Reads a byte from flash
///
/// @param	address		The address from which to read the byte.
/// @returns			The byte that was read.
///
uint8_t	flash_read_byte(uint16_t address);

#ifdef BOARD_rfd900a
void flash_transfer_calibration();
#endif

#endif // _FLASH_H_
