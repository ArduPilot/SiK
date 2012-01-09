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
/// @file	parameters.c
///
/// Storage for program parameters.
///
/// Parameters are held in a contiguous array of 16-bit values.
/// It is up to the caller to decide how large a parameter is and to
/// access it accordingly.
///
/// When saved, parameters are copied bitwise to the flash scratch page,
/// with an 8-bit XOR checksum appended to the end of the flash range.
///


#include "radio.h"

/// In-ROM parameter info table.
///
__code const struct parameter_info {
	const char	*name;
	param_t		default_value;
} parameter_info[PARAM_MAX] = {
	{"FORMAT", 		PARAM_FORMAT_CURRENT},
	{"SERIAL_SPEED",	B57600}, // match APM default
	{"AIR_SPEED",		128},
	{"NETID",		0},
};

/// In-RAM parameter store.
///
/// It seems painful to have to do this, but we need somewhere to
/// hold all the parameters when we're rewriting the scratchpad
/// page anyway.
///
union param_private {
	param_t		val;
	uint8_t		bytes[2];
};
__xdata union param_private	parameter_values[PARAM_MAX];

static bool
param_check(enum ParamID id, uint16_t val)
{
	// parameter value out of range - fail
	if (id >= PARAM_MAX)
		return false;

	switch (id) {
	case PARAM_FORMAT:
		return false;

	case PARAM_SERIAL_SPEED:
		if (val > BMAX)
			return false;
		break;

	default:
		// no sanity check for this value
		break;
	}
	return true;
}

bool
param_set(enum ParamID param, param_t value)
{
	// Sanity-check the parameter value first.
	if (!param_check(param, value))
		return false;

	parameter_values[param].val = value;
	return true;
}

param_t
param_get(enum ParamID param)
{
	if (param >= PARAM_MAX)
		return 0;
	return parameter_values[param].val;
}

bool
param_load()
{
	__pdata uint8_t		d;
	__pdata uint8_t		i;
	__pdata uint8_t		sum;

	// initialise checksum
	sum = 0;

	// loop reading the parameters array
	for (i = 0; i < sizeof(parameter_values); i ++) {
		d = flash_read_scratch(i);
		parameter_values[0].bytes[i] = d;
		sum ^= d;
	}

	// verify checksum
	d = flash_read_scratch(i);
	if (sum != d)
		return false;

	// decide whether we read a supported version of the structure
	if (param_get(PARAM_FORMAT) != PARAM_FORMAT_CURRENT) {
		debug("parameter format %lu expecting %lu", parameters[PARAM_FORMAT], PARAM_FORMAT_CURRENT);
		return false;
	}
	return true;
}

void
param_save()
{
	__pdata uint8_t		d;
	__pdata uint8_t		i;
	__pdata uint8_t		sum;

	// tag parameters with the current format
	parameter_values[PARAM_FORMAT].val = PARAM_FORMAT_CURRENT;

	// erase the scratch space
	flash_erase_scratch();

	// initialise checksum
	sum = 0;

	// save parameters to the scratch page
	for (i = 0; i < sizeof(parameter_values); i++) {
		d = parameter_values[0].bytes[i];	// byte we are going to write
		sum ^= d;
		flash_write_scratch(i, d);
	}

	// write checksum
	flash_write_scratch(i, sum);
}

void
param_default(void)
{
	__pdata uint8_t	i;

	// set all parameters to their default values
	for (i = 0; i < PARAM_MAX; i++) {
		parameter_values[i].val = parameter_info[i].default_value;
	}
}

enum ParamID
param_id(char *name)
{
	__pdata uint8_t i;

	for (i = 0; i < PARAM_MAX; i++) {
		if (!strcmp(name, parameter_info[i].name))
			break;
	}
	return i;
}

const char *__code
param_name(enum ParamID param)
{
	if (param < PARAM_MAX) {
		return parameter_info[param].name;
	}
	return 0;
}
