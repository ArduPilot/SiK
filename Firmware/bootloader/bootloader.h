/* -*- Mode: C; c-basic-offset: 8; -*- */
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

/// @file	bootloader.h
///
/// Bootloader structures and defines.
///

#ifndef _BOOTLOADER_H
#define _BOOTLOADER_H

#include <stdint.h>

#ifdef __CDT_PARSER__
# define __data
# define __xdata
# define __pdata
# define __code
# define __at(_x)
#endif

// Basic flash protocol, originally inspired by the STK500 protocol
//
// Packet is command byte, command data, EOC.  Reply is [data,] OK or FAILED.
//
#define PROTO_OK		0x10	// 'ok' response
#define PROTO_FAILED		0x11	// 'fail' response
#define PROTO_INSYNC		0x12	// 'in sync' response to GET_SYNC

#define PROTO_EOC		0x20	// end of command
#define PROTO_GET_SYNC		0x21	// NOP for re-establishing sync
#define PROTO_GET_DEVICE	0x22	// get device ID byte
#define PROTO_CHIP_ERASE	0x23	// erase program area
#define PROTO_LOAD_ADDRESS	0x24	// set next programming address
#define PROTO_PROG_FLASH	0x25	// write byte at address + increment
#define PROTO_READ_FLASH	0x26	// read byte at address + increment
#define PROTO_PROG_MULTI	0x27	// write up to PROTO_PROG_MULTI_MAX bytes at address + increment
#define PROTO_READ_MULTI	0x28	// read up to 255 bytes at address + increment

#define PROTO_PROG_MULTI_MAX	64	// maximum PROG_MULTI size - must fit in DSEG

#endif // _BOOTLOADER_H
