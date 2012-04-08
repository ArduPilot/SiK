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
#include "tdm.h"

/// In-ROM parameter info table.
///
__code const struct parameter_info {
	const char	*name;
	param_t		default_value;
} parameter_info[PARAM_MAX] = {
	{"FORMAT", 		PARAM_FORMAT_CURRENT},
	{"SERIAL_SPEED",	57}, // match APM default of 57600
	{"AIR_SPEED",		64}, // relies on MAVLink flow control
	{"NETID",		25},
	{"TXPOWER",		0},
	{"ECC",			1},
	{"MAVLINK",		1},
	{"OPPRESEND",		1},
	{"MIN_FREQ",		0},
	{"MAX_FREQ",		0},
	{"NUM_CHANNELS",	0},
	{"DUTY_CYCLE",		100},
	{"LBT_RSSI",		0},
	{"MANCHESTER",		0}
};

/// In-RAM parameter store.
///
/// It seems painful to have to do this, but we need somewhere to
/// hold all the parameters when we're rewriting the scratchpad
/// page anyway.
///
union param_private {
	param_t		val;
	uint8_t		bytes[4];
};
__xdata union param_private	parameter_values[PARAM_MAX];

static bool
param_check(enum ParamID id, __pdata uint32_t val)
{
	// parameter value out of range - fail
	if (id >= PARAM_MAX)
		return false;

	switch (id) {
	case PARAM_FORMAT:
		return false;

	case PARAM_SERIAL_SPEED:
		return serial_device_valid_speed(val);

	case PARAM_AIR_SPEED:
		if (val > 256)
			return false;
		break;

	case PARAM_NETID:
		// all values are OK
		return true;

	case PARAM_TXPOWER:
		if (val > BOARD_MAXTXPOWER)
			return false;
		break;

	case PARAM_ECC:
	case PARAM_MAVLINK:
	case PARAM_OPPRESEND:
		// boolean 0/1 only
		if (val > 1)
			return false;
		break;

	default:
		// no sanity check for this value
		break;
	}
	return true;
}

bool
param_set(__data enum ParamID param, __pdata param_t value)
{
	// Sanity-check the parameter value first.
	if (!param_check(param, value))
		return false;

	parameter_values[param].val = value;

	// some parameters we update immediately
	switch (param) {
	case PARAM_TXPOWER:
		// useful to update power level immediately when range
		// testing in RSSI mode		
		radio_set_transmit_power(value);
		break;
	case PARAM_DUTY_CYCLE:
		// update duty cycle immediately
		duty_cycle = constrain(value, 0, 100);
		break;

	default:
		break;
	}

	return true;
}

param_t
param_get(__data enum ParamID param)
{
	if (param >= PARAM_MAX)
		return 0;
	return parameter_values[param].val;
}

bool
param_load(void)
__critical {
	__pdata uint8_t		d;
	__pdata uint8_t		i;
	__pdata uint8_t		sum;
	__pdata uint8_t         count;

	// start with defaults
	for (i = 0; i < sizeof(parameter_values); i++) {
		parameter_values[i].val = parameter_info[i].default_value;
	}

	// initialise checksum
	sum = 0;
	count = flash_read_scratch(0);
	if (count > sizeof(parameter_values) ||
	    count < 12*sizeof(param_t)) {
		return false;
	}

	// loop reading the parameters array
	for (i = 0; i < count; i ++) {
		d = flash_read_scratch(i+1);
		parameter_values[0].bytes[i] = d;
		sum ^= d;
	}

	// verify checksum
	d = flash_read_scratch(i+1);
	if (sum != d)
		return false;

	// decide whether we read a supported version of the structure
	if (param_get(PARAM_FORMAT) != PARAM_FORMAT_CURRENT) {
		debug("parameter format %lu expecting %lu", parameters[PARAM_FORMAT], PARAM_FORMAT_CURRENT);
		return false;
	}

	for (i = 0; i < sizeof(parameter_values); i++) {
		if (!param_check(i, parameter_values[i].val)) {
			parameter_values[i].val = parameter_info[i].default_value;
		}
	}

	return true;
}

void
param_save(void)
__critical {
	__pdata uint8_t		d;
	__pdata uint8_t		i;
	__pdata uint8_t		sum;

	// tag parameters with the current format
	parameter_values[PARAM_FORMAT].val = PARAM_FORMAT_CURRENT;

	// erase the scratch space
	flash_erase_scratch();

	// initialise checksum
	sum = 0;
	flash_write_scratch(0, sizeof(parameter_values));

	// save parameters to the scratch page
	for (i = 0; i < sizeof(parameter_values); i++) {
		d = parameter_values[0].bytes[i];	// byte we are going to write
		sum ^= d;
		flash_write_scratch(i+1, d);
	}

	// write checksum
	flash_write_scratch(i+1, sum);
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
param_id(__data char * __pdata name)
{
	__pdata uint8_t i;

	for (i = 0; i < PARAM_MAX; i++) {
		if (!strcmp(name, parameter_info[i].name))
			break;
	}
	return i;
}

const char *__code
param_name(__data enum ParamID param)
{
	if (param < PARAM_MAX) {
		return parameter_info[param].name;
	}
	return 0;
}

// constraint for parameter values
uint32_t constrain(__pdata uint32_t v, __pdata uint32_t min, __pdata uint32_t max)
{
	if (v < min) v = min;
	if (v > max) v = max;
	return v;
}

