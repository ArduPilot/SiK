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
/// This enumeration should be updated
/// if/when we switch to using the hop register and fixed channel spacing, etc.
///
/// Parameter IDs here match AT S-register numbers, so change them with extreme
/// care.  Parameter zero cannot be written by AT commands.
///
enum ParamID {
	PARAM_FORMAT = 0,		// Must always be parameter 0
	PARAM_NODE_ID,			// this node's ID
	PARAM_PEER_ID,			// peer node's ID (if paired)
	PARAM_SERIAL_SPEED,		// BAUD_RATE_* constant

	PARAM_MAX			// must be last
};

#define PARAM_FORMAT_CURRENT	0x14UL				///< current parameter format ID

/// Returns a parameter as an 8-bit value
///
/// @note Passing a parameter ID that is out of range will return garbage.
///
/// @param	param		The parameter to return.
/// @return			The low 8 bits of the saved parameter.
///
extern uint8_t	param_get8 (enum ParamID param);

/// Returns a parameter as a 16-bit value
///
/// @note Passing a parameter ID that is out of range will return garbage.
///
/// @param	param		The parameter to return.
/// @return			The parameter.
///
extern uint16_t	param_get16(enum ParamID param);

/// Set a parameter to an 8-bit value
///
/// @note Parameters are not saved until param_save is called.
///
/// @param	param		The parameter to set.
/// @param	value		The value to assign to the parameter.
/// @return			True if the parameter's value is valid.
///
extern bool param_set8 (enum ParamID param, uint8_t value);

/// Set a parameter to a 16-bit value
///
/// @note Parameters are not saved until param_save is called.
///
/// @param	param		The parameter to set.
/// @param	value		The value to assign to the parameter.
/// @return			True if the parameter's value is valid.
///
extern bool param_set16(enum ParamID param, uint16_t value);

/// Load parameters from the flash scratchpad.
///
/// @return			True if parameters were successfully loaded.
///
extern bool param_load(void);

/// Save parameters to the flash scratchpad.
///
extern void param_save(void);

/// Reset parameters to default.
///
extern void param_default(void);

/// Validates that a parameter has a valid value
///
/// @param	param		The parameter to validate.
/// @param	val		The value to validate.
///
extern bool param_check(enum ParamID id, uint16_t val);
