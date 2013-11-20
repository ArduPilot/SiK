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
///		Prototypes for the flash interface.
///

// The Si1030 doesn't have a flash scratch page
// so the read and write's are slightly diffrent
#ifdef CPU_SI1030
#define FLASH_ERASE_SCRATCH 0x03
#define FLASH_WRITE_SCRATCH 0x01
#define FLASH_READ_SCRATCH  0x00
#define FLASH_DISABLE		0x00
#else
#define FLASH_ERASE_SCRATCH 0x07
#define FLASH_WRITE_SCRATCH 0x05
#define FLASH_READ_SCRATCH  0x04
#define FLASH_DISABLE		0x00
#endif

extern void	flash_erase_scratch(void);
extern uint8_t	flash_read_scratch(__pdata uint16_t address);
extern void	flash_write_scratch(__pdata uint16_t address, __pdata uint8_t c);
