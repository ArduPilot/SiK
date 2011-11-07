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
/// @file	bootloader.h
///
/// Bootloader structures and defines.
///

#ifndef _BOOTLOADER_H_
#define _BOOTLOADER_H_

#include <stdint.h>
#include "cdt.h"

// SiK bootloader flash update protocol.
//
// Command format:
//
// 	<opcode>[<command_data>]<EOC>
//
// Reply format:
//
//	[<reply_data>]<INSYNC><status>
//
// The <opcode> and <status> values come from the PROTO_ defines below,
// the <*_data> fields is described only for opcodes that transfer data;
// in all other cases the field is omitted.
//
// Expected workflow is:
//
// GET_SYNC			verify that the board is present
// GET_DEVICE			determine which board (select firmware to upload)
// CHIP_ERASE			clear the program area
// loop:
//	LOAD_ADDRESS		set address for programming fragment
//	loop:
//		PROG_MULTI	program portion of fragment
// loop:
//	LOAD_ADDRESS		set address for verifying program fragment
//	loop:
//		READ_MULTI	verify portion of fragment
// PARAM_ERASE			optional - clear flash scratch/parameter page
// RESET			resets chip and starts application
//
#define PROTO_OK		0x10	// 'ok' response
#define PROTO_FAILED		0x11	// 'fail' response
#define PROTO_INSYNC		0x12	// 'in sync' byte sent before status

#define PROTO_EOC		0x20	// end of command
#define PROTO_GET_SYNC		0x21	// NOP for re-establishing sync
#define PROTO_GET_DEVICE	0x22	// get device ID bytes,                       <reply_data>: <board ID><frequency code>
#define PROTO_CHIP_ERASE	0x23	// erase program area
#define PROTO_LOAD_ADDRESS	0x24	// set next programming address               <command_data>: <lowbyte><highbyte>
#define PROTO_PROG_FLASH	0x25	// write byte at address + increment address  <command_data>: <databyte>
#define PROTO_READ_FLASH	0x26	// read byte at address + increment address   <reply_data>: <databyte>
#define PROTO_PROG_MULTI	0x27	// write bytes at address + increment         <command_data>: <count><databytes>
#define PROTO_READ_MULTI	0x28	// read bytes at address + increment          <command_data>: <count>,  <reply_data>: <databytes>
#define PROTO_PARAM_ERASE	0x29	// erase the parameter flash

#define PROTO_REBOOT		0x30	// reboot the board & start the app

#define PROTO_DEBUG		0x31	// emit debug information - format not defined

#define PROTO_PROG_MULTI_MAX	64	// maximum PROG_MULTI size
#define PROTO_READ_MULTI_MAX	255	// size of the size field

#endif // _BOOTLOADER_H_
