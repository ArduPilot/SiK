// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2013 Luke Hovington, All Rights Reserved
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
/// @file	pins.h
///
/// Prototypes for the PINS command parser
///

#ifndef _PINS_H_
#define _PINS_H_

#include <stdint.h>
#include <stdbool.h>

// Pin rfd900a  Mapping
#if defined BOARD_rfd900a || defined BOARD_rfd900p 
#define PINS_USER_MAX 6
#elif defined BOARD_rfd900u
#define PINS_USER_MAX 2
#else
#define PINS_USER_MAX 0
#endif

#define PINS_ABS_MAX 10

#ifdef INCLUDE_AES
#define PIN_MAX 0
#else
#define PIN_MAX (PINS_USER_MAX < PINS_ABS_MAX ? PINS_USER_MAX : PINS_ABS_MAX)
#endif // INCLUDE_AES

enum pin_state { PIN_OUTPUT=true, PIN_INPUT=false,
				 PIN_HIGH=true,   PIN_LOW=false,
				 PIN_NULL=0xFF,   PIN_MIRROR_NULL=0xFFFFFFFF,
				 PIN_ERROR=0x7F };

/// In-ROM parameter info table. Changed by ATP commands
/// When changing this structure, PINS_USER_INFO_DEFAULT and param_default() need updating
///
typedef struct pins_user_info {
	uint16_t   output:4;
	uint16_t   pin_dir:4;
	uint16_t   pin_mirror:8;
} pins_user_info_t;

#define PINS_USER_INFO_DEFAULT {PIN_OUTPUT, PIN_LOW, PIN_NULL}

#if PIN_MAX > 0

extern pins_user_info_t pin_values[];

extern void pins_user_init(void);
extern bool pins_user_set_io(__pdata uint8_t pin, bool in_out);
extern bool pins_user_get_io(__pdata uint8_t pin);
extern bool pins_user_set_value(__pdata uint8_t pin, bool high_low);
extern bool pins_user_get_value(__pdata uint8_t pin);
extern uint8_t pins_user_get_adc(__pdata uint8_t pin);
extern void pins_user_check();
#endif // #if PIN_MAX > 0

#endif	// _PINS_H_
