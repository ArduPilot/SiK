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
#include "em_device.h" // need to know flash sizes

// Flash memory map
#define FLASH_APP_START		0x4000					// 2 pages reserved for bootloader

//#if defined BOARD_rfd900u || defined BOARD_rfd900p
// WARNING You need to select Bank 3 for these address to work. (p125)
//#define FLASH_INFO_PAGE   0xFC00    // 1 page reserved for bootloader (In Bank3)
//#define FLASH_LOCK_BYTE   0xFFFF    // Bank3
//#define FLASH_SCRATCH     0xF800    // We don't have a scratch page so lets define one in code space (bank3).
//#define FLASH_BANKS            3    // 0-Home, 1-Bank1, 2-Bank2, 3-Bank3

// pages are always 2048 bytes, size can be 64,128,or 256K
// we will be using 128K/2 = 64 pages
#define FLASH_NUM_PAGES     (FLASH_SIZE/FLASH_PAGE_SIZE)
#define FLASH_CAL_PAGE		  (FLASH_NUM_PAGES-1)
#define FLASH_CAL_START     (FLASH_CAL_PAGE*FLASH_PAGE_SIZE)
#define FLASH_CAL_END       ((FLASH_CAL_PAGE+1)*FLASH_PAGE_SIZE)
#define FLASH_SCRATCH_PAGE	(FLASH_CAL_PAGE-1)
#define FLASH_SCRATCH       (FLASH_SCRATCH_PAGE*FLASH_PAGE_SIZE)
#define FLASH_SIG_PAGE      (FLASH_SCRATCH_PAGE-1)
#define FLASH_SIG_START     (FLASH_SIG_PAGE*FLASH_PAGE_SIZE)
#define FLASH_SIG_END       ((FLASH_SIG_PAGE+1)*FLASH_PAGE_SIZE)
#define FLASH_SIGNATURE_ADDR (FLASH_SIG_END - 2) // Location of the flash signature


// Anticipated flash signature bytes
#define FLASH_SIG0	0x3d
#define FLASH_SIG1	0xc2


// locked and unlocked areas to store power calibration info
#define FLASH_CALIBRATION_AREA_SIZE   (31)
#define FLASH_CALIBRATION_CRC         (FLASH_CAL_END - 1)
#define FLASH_CALIBRATION_AREA        (FLASH_CALIBRATION_CRC - FLASH_CALIBRATION_AREA_SIZE)

#endif	// _FLASH_LAYOUT_H
