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

/// @file	parameters.h
///
/// Definitions for program parameter storage.

/// Parameter IDs.
///
/// Note that the implementation does not support more than 64 parameters
/// total.
///
enum ParamID {
	PARAM_FORMAT = 0,
	PARAM_NODE_ID,			// this node's ID
	PARAM_PEER_ID,			// peer node's ID (if paired)
	PARAM_TRX_FREQUENCY,
	PARAM_TRX_DEVIATION,
	PARAM_TRX_DATA_RATE,
	PARAM_RX_BAND_WIDTH,
	PARAM_SERIAL_SPEED,

	PARAM_MAX			// must be last
};

#define PARAM_FORMAT_CURRENT	0x10UL

extern uint8_t	param_get8 (enum ParamID param);
extern uint16_t	param_get16(enum ParamID param);
extern uint32_t	param_get32(enum ParamID param);

extern void param_set8 (enum ParamID param, uint8_t value);
extern void param_set16(enum ParamID param, uint16_t value);
extern void param_set32(enum ParamID param, uint32_t value);

extern bool param_load();
extern void param_save();
