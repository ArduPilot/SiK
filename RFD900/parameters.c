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


#include "radio_old.h"
#include "tdm.h"
#include "flash_layout.h"
#include "pins_user.h"
#include "parameters.h"
#include "serial.h"
#include "board_rfd900e.h"
#include "printfl.h"
#include "flash.h"
#include "crc.h"

//#define NO_FLASH_TEST 1
#if NO_FLASH_TEST
#define FLASH_CALIBRATION_AREA_SIZE   (31)
#endif
/// In-ROM parameter info table.
///
const struct parameter_s_info {
	const char	*name;
	param_t		default_value;
} parameter_s_info[PARAM_S_MAX] = {
	{"FORMAT",         PARAM_FORMAT_CURRENT},
	{"SERIAL_SPEED",   57}, // match APM default of 57600
	{"AIR_SPEED",      64}, // relies on MAVLink flow control
	{"NETID",          25},
	{"TXPOWER",        20},
	{"ECC",             0},
	{"MAVLINK",         0},
	{"OPPRESEND",       0},
	{"MIN_FREQ",        0},
	{"MAX_FREQ",        0},
	{"NUM_CHANNELS",    0},
	{"DUTY_CYCLE",    100},
	{"LBT_RSSI",        0},
	{"MANCHESTER",      0},
	{"RTSCTS",          1},
	{"MAX_WINDOW",    131},
};

const struct parameter_r_info {
	const char	*name;
	param_t		default_value;
} parameter_r_info[PARAM_R_MAX] = {
	{"TARGET_RSSI",     255},
	{"HYSTERESIS_RSSI", 50},
};

/// In-RAM parameter store.
///
/// It seems painful to have to do this, but we need somewhere to
/// hold all the parameters when we're rewriting the scratchpad
/// page anyway.
///
param_t	parameter_s_values[PARAM_S_MAX];
param_t	parameter_r_values[PARAM_R_MAX];

// Three extra bytes, 1 for the number of params and 2 for the checksum
#define PARAM_S_FLASH_START   0
#define PARAM_S_FLASH_END     (PARAM_S_FLASH_START + sizeof(parameter_s_values) + 3)

// Three extra bytes, 1 for the number of params and 2 for the checksum, starts at position 128
#define PARAM_R_FLASH_START   (2<<6)
#define PARAM_R_FLASH_END     (PARAM_R_FLASH_START + sizeof(parameter_r_values) + 3)

#if PIN_MAX > 0
const pins_user_info_t pins_defaults = PINS_USER_INFO_DEFAULT;
pins_user_info_t pin_values[PIN_MAX];

// Place the start away from the other params to allow for expantion 2<<7 = 256
#define PIN_FLASH_START       (2<<7)
#define PIN_FLASH_END         (PIN_FLASH_START + sizeof(pin_values) + 2)
#endif



static bool param_s_check(enum Param_S_ID id, uint32_t val)
{
	// parameter value out of range - fail
	if (id >= PARAM_S_MAX)
		return false;

	switch (id) {
	case PARAM_FORMAT:
		if(PARAM_FORMAT_CURRENT != val)
			return false;
		break;

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
	case PARAM_OPPRESEND:
		// boolean 0/1 only
		if (val > 1)
			return false;
		break;

	case PARAM_MAVLINK:
		if (val > 2)
			return false;
		break;

	case PARAM_MAX_WINDOW:
		// 131 milliseconds == 0x1FFF 16 usec ticks,
		// which is the maximum we can handle with a 13
		// bit trailer for window remaining
		if (val > 131)
			return false;
		break;

	default:
		// no sanity check for this value
		break;
	}
	return true;
}

bool param_s_set(enum Param_S_ID param, param_t value)
{
	// Sanity-check the parameter value first.
	if (!param_s_check(param, value))
		return false;

	// some parameters we update immediately
	switch (param) {
	case PARAM_TXPOWER:
		// useful to update power level immediately when range
		// testing in RSSI mode		
		radio_set_transmit_power(value);
		value = radio_get_transmit_power();
		break;

	case PARAM_DUTY_CYCLE:
		// update duty cycle immediately
		value = constrain(value, 0, 100);
		duty_cycle = value;
		break;

	case PARAM_LBT_RSSI:
		// update LBT RSSI immediately
		if (value != 0) {
			value = constrain(value, 25, 220);
		}
		lbt_rssi = value;
		break;

	case PARAM_MAVLINK:
		feature_mavlink_framing = (uint8_t) value;
		value = feature_mavlink_framing;
		break;

	case PARAM_OPPRESEND:
		feature_opportunistic_resend = value?true:false;
		value = feature_opportunistic_resend?1:0;
		break;

	case PARAM_RTSCTS:
		feature_rtscts = value?true:false;
		value = feature_rtscts?1:0;
		break;

	default:
		break;
	}

	parameter_s_values[param] = value;

	return true;
}

param_t param_s_get(enum Param_S_ID param)
{
	if (param >= PARAM_S_MAX)
		return 0;
	return parameter_s_values[param];
}

static bool param_r_check(enum Param_R_ID id, uint32_t val)
{
	// parameter value out of range - fail
	if (id >= PARAM_R_MAX)
		return false;
	
	switch (id) {
		case PARAM_R_TARGET_RSSI:
			if (val < 50 || 255 < val)
				return false;
			break;

		case PARAM_R_HYSTERESIS_RSSI:
			if (val < 20 || 50 < val)
				return false;
			break;
			
		default:
			// no sanity check for this value
			break;
	}
	return true;
}

bool param_r_set(enum Param_R_ID param, param_t value)
{
	// Sanity-check the parameter value first.
	if (!param_r_check(param, value))
		return false;
	
	// some parameters we update immediately
	switch (param) {
//		case PARAM_R_TARGET_RSSI:
//			break;
		default:
			break;
	}
	
	parameter_r_values[param] = value;
	
	return true;
}

param_t param_r_get(enum Param_R_ID param)
{
	if (param >= PARAM_R_MAX)
		return 0;
	return parameter_r_values[param];
}

#if !NO_FLASH_TEST
static bool read_params(uint8_t * input, uint16_t start, uint8_t size)
{
	uint16_t		i;
	
	for (i = start; i < start+size; i ++){
		input[i-start] = flash_read_scratch(i);
		printf("%d-%d\n",i,input[i-start]);
	}
	
	// verify checksum
	if (crc16(size, input) != ((uint16_t) flash_read_scratch(i+1)<<8 | flash_read_scratch(i)))
		return false;
	return true;
}

static void write_params(uint8_t * input, uint16_t start, uint8_t size)
{
	uint16_t	i, checksum;

	// We cannot address greater than one page (1023 bytes)
	if((start + size + 2) > 1023)
		return;
	
	// save parameters to the scratch page
	for (i = start; i < start+size; i ++)
	{
		flash_write_scratch(i, input[i-start]);
	}
	
	// write checksum
	checksum = crc16(size, input);
	flash_write_scratch(i, checksum&0xFF);
	flash_write_scratch(i+1, checksum>>8);
}
#endif

bool param_load(void)
{
	uint8_t	i;

	// Start with default values
	param_default();
#if !NO_FLASH_TEST
	uint8_t	expected;
	
	// read and verify params
	expected = flash_read_scratch(PARAM_S_FLASH_START);
	if (expected > sizeof(parameter_s_values) || expected < 12*sizeof(param_t))
		return false;
	
	if(!read_params((uint8_t *)parameter_s_values, PARAM_S_FLASH_START+1, expected))
		return false;

	// read and verify params
	expected = flash_read_scratch(PARAM_R_FLASH_START);
	if (expected > sizeof(parameter_r_values))
		return false;
	if(!read_params((uint8_t *)parameter_r_values, PARAM_R_FLASH_START+1, expected))
		return false;
#endif
	
	// decide whether we read a supported version of the structure
	if ((param_t) PARAM_FORMAT_CURRENT != parameter_s_values[PARAM_FORMAT]) {
		debug("parameter format %lu expecting %lu", parameter_s_values[PARAM_FORMAT], PARAM_FORMAT_CURRENT);
		return false;
	}
	for (i = 0; i < PARAM_S_MAX; i++) {
		if (!param_s_check(i, parameter_s_values[i])) {
			parameter_s_values[i] = parameter_s_info[i].default_value;
		}
	}
	
	for (i = 0; i < PARAM_R_MAX; i++) {
		if (!param_r_check(i, parameter_r_values[i])) {
			parameter_r_values[i] = parameter_r_info[i].default_value;
		}
	}
	
	// read and verify pin params
#if !NO_FLASH_TEST
#if PIN_MAX > 0
	expected = flash_read_scratch(PIN_FLASH_START);
	if (expected != sizeof(pin_values))
		return false;
	if(!read_params((uint8_t *)pin_values, PIN_FLASH_START+1, sizeof(pin_values)))
		return false;
#endif
#endif
	return true;
}

void param_save(void)
{
	// tag parameters with the current format
	parameter_s_values[PARAM_FORMAT] = PARAM_FORMAT_CURRENT;
#if !NO_FLASH_TEST

	// erase the scratch space
	flash_erase_scratch();

	// write S params
	flash_write_scratch(PARAM_S_FLASH_START, sizeof(parameter_s_values));
	write_params((uint8_t *)parameter_s_values, PARAM_S_FLASH_START+1, sizeof(parameter_s_values));

	// write R params
	flash_write_scratch(PARAM_R_FLASH_START, sizeof(parameter_r_values));
	write_params((uint8_t *)parameter_r_values, PARAM_R_FLASH_START+1, sizeof(parameter_r_values));
	
	// write pin params
#if PIN_MAX > 0
	flash_write_scratch(PIN_FLASH_START, sizeof(pin_values));
	write_params((uint8_t *)pin_values, PIN_FLASH_START+1, sizeof(pin_values));
#endif
#endif
}

void param_default(void)
{
	uint8_t	i;

	// set all parameters to their default values
	for (i = 0; i < PARAM_S_MAX; i++) {
		parameter_s_values[i] = parameter_s_info[i].default_value;
	}

	// set all parameters to their default values
	for (i = 0; i < PARAM_R_MAX; i++) {
		parameter_r_values[i] = parameter_r_info[i].default_value;
	}
	
#if PIN_MAX > 0
	for (i = 0; i < PIN_MAX; i ++) {
		pin_values[i].output = pins_defaults.output;
		pin_values[i].pin_dir = pins_defaults.pin_dir;
		pin_values[i].pin_mirror = pins_defaults.pin_mirror;
	}
#endif
}

enum Param_S_ID param_s_id(char * name)
{
	uint8_t i;

	for (i = 0; i < PARAM_S_MAX; i++) {
		if (!strcmp(name, parameter_s_info[i].name))
			break;
	}
	return i;
}

const char *param_s_name(enum Param_S_ID param)
{
	if (param < PARAM_S_MAX) {
		return parameter_s_info[param].name;
	}
	return 0;
}

enum Param_R_ID param_r_id(char * name)
{
	uint8_t i;
	
	for (i = 0; i < PARAM_R_MAX; i++) {
		if (!strcmp(name, parameter_r_info[i].name))
			break;
	}
	return i;
}

const char *param_r_name(enum Param_R_ID param)
{
	if (param < PARAM_R_MAX) {
		return parameter_r_info[param].name;
	}
	return 0;
}

// constraint for parameter values
uint32_t constrain(uint32_t v, uint32_t min, uint32_t max)
{
	if (v < min) v = min;
	if (v > max) v = max;
	return v;
}

#define flash_read_byte(addr) (*(uint8_t*)(addr))

bool calibration_set(uint8_t idx, uint8_t value)
{
#if !NO_FLASH_TEST
	// if level is valid
	if (idx <= BOARD_MAXTXPOWER && value != 0xFF)
	{
		// if the target byte isn't yet written
		if (flash_read_byte(FLASH_CALIBRATION_AREA + idx) == 0xFF)
		{
			flash_write_byte(FLASH_CALIBRATION_AREA + idx, value);
      //flash_write_byte(FLASH_CALIBRATION_AREA + idx,      value);
			return true;
		}
	}
	return false;
#else
	return true;
#endif
}

uint8_t calibration_get(uint8_t level)
{
	uint8_t idx;
	uint8_t crc = 0;
	uint8_t *calibration = (uint8_t *)(FLASH_CALIBRATION_AREA);
	uint8_t calibration_crc = flash_read_byte(FLASH_CALIBRATION_CRC);

	// Change for next board revision
	for (idx = 0; idx < FLASH_CALIBRATION_AREA_SIZE; idx++)
	{
		crc ^= calibration[idx];
	}

	if (calibration_crc != 0xFF && calibration_crc == crc && level <= BOARD_MAXTXPOWER)
	{
		return calibration[level];
	}
	return 0xFF;
}

bool calibration_lock(void)
{
#if !NO_FLASH_TEST
	uint8_t idx;
	uint8_t crc = 0;

	// check that all entries are written
	if (flash_read_byte(FLASH_CALIBRATION_CRC) == 0xFF)
	{
		for (idx=0; idx < FLASH_CALIBRATION_AREA_SIZE; idx++)
		{
			uint8_t cal = flash_read_byte(FLASH_CALIBRATION_AREA + idx);
			crc ^= cal;
			if (cal == 0xFF)
			{
				printf("dBm level %u not calibrated\n",idx);
				return false;
			}
		}

		// write crc
		flash_write_byte(FLASH_CALIBRATION_CRC, crc);
		// lock the first and last pages
		// can only be reverted by reflashing the bootloader
		FlashLockBlock((uint8_t *)FLASH_CALIBRATION_AREA);
		return true;
	}
	return false;
#else
	return true;
#endif

}
