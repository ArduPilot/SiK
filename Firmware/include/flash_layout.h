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
/// @file	flash_layout.h
///
/// Layout definitions for the SiK bootloader and conforming applications.
///

#ifndef _FLASH_LAYOUT_H
#define _FLASH_LAYOUT_H

#include <stdint.h>

// Flash memory map
// It would be nice to derive all these numbers from some
// simpler values, but life is easier if we just spell them all out.
//
#define FLASH_PAGE_SIZE		0x0400		// 1KiB
#define FLASH_PAGE_SHIFT	10
#define FLASH_APP_START		0x0400		// 1 page reserved for bootloader
#define FLASH_INFO_PAGE		0xf800		// 1 page reserved for bootloader
#define FLASH_LOCK_BYTE		0xfbff

// Anticipated flash signature bytes
//
#define FLASH_SIG0	0x3d
#define FLASH_SIG1	0xc2

// Location of the flash signature
//
#define FLASH_SIGNATURE_BYTES	(FLASH_INFO_PAGE - 2)

// Bootloader patchbay for frequency code
//
#define FLASH_FREQUENCY_BYTE	(FLASH_LOCK_BYTE - 1)

#ifdef BOARD_rfd900a
// locked and unlocked areas to store power calibration info
#define FLASH_CALIBRATION_AREA_SIZE (31)
#define FLASH_CALIBRATION_CRC_HIGH  (FLASH_FREQUENCY_BYTE - 1)
#define FLASH_CALIBRATION_AREA_HIGH  (FLASH_CALIBRATION_CRC_HIGH - FLASH_CALIBRATION_AREA_SIZE)
#define FLASH_CALIBRATION_CRC  (FLASH_SIGNATURE_BYTES - 1)
#define FLASH_CALIBRATION_AREA  (FLASH_CALIBRATION_CRC - FLASH_CALIBRATION_AREA_SIZE)
#endif

#endif	// _FLASH_LAYOUT_H
