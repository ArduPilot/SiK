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
/// @file	at.h
///
/// Prototypes for the AT command parser
///

#ifndef _AT_H_
#define _AT_H_

extern bool	at_mode_active;	///< if true, the AT interpreter is in command mode
extern bool	at_cmd_ready;	///< if true, at_cmd / at_cmd_len contain valid data

/// Timer tick handler for the AT command interpreter
///
extern void	at_timer(void);

/// +++ detector.  Handles the state machine for detecting the AT escape
/// sequence.
///
/// Call this at interrupt time for every incoming character when at_mode_active
/// is false.
///
/// @param	c		Received character.
///
extern void	at_plus_detector(register uint8_t c);

/// AT command character input method.
///
/// Call this at interrupt time for every incoming character when at_mode_active
/// is true, and don't buffer those characters.
///
///  @param	c		Received character.
///
extern void	at_input(register uint8_t c);

/// Check for and execute AT commands
///
/// Call this from non-interrupt context when it's safe for an AT command
/// to be executed.  It's cheap if at_mode_active is false.
///
extern void	at_command(void);

/// AT_TEST_* test modes
extern __pdata uint8_t  at_testmode;    ///< AT_TEST_* bits

#define AT_TEST_RSSI 1
#define AT_TEST_TDM  2

// max size of an AT command
#define AT_CMD_MAXLEN	16

// AT command buffer
extern __pdata char at_cmd[AT_CMD_MAXLEN + 1];
extern __pdata uint8_t at_cmd_len;

#endif	// _AT_H_
