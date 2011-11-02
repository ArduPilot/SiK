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
/// @file	board_info.h
///
/// Board information passed from the bootloader to the
/// application.
///

#ifndef _BOARD_INFO_H_
#define _BOARD_INFO_H_

/// Possible board RF configurations.
///
/// These bytes are patched into the last byte of the first
/// page of flash.
///
enum BoardFrequency {
        FREQ_433	= 0x43,
        FREQ_470	= 0x47,
        FREQ_868	= 0x86,
        FREQ_915	= 0x91,
        FREQ_NONE	= 0xf0,
};

// SFRs used to temporarily save board information during handoff
// between the bootloader and the application.
//
#define BOARD_FREQUENCY_REG	ADC0GTH		// board frequency
#define BOARD_BL_VERSION_REG	ADC0GTL		// bootloader version
#define BOARD_UNUSED1_REG	ADC0LTH		// spare
#define BOARD_UNUSED2_REG	ADC0LTL		// spare

#endif // _BOARD_INFO_H
