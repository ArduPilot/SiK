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
#include "crc.h"
#include <flash_layout.h>

#ifdef INCLUDE_AES
#include "AES/aes.h"
#endif // INCLUDE_AES

/// In-ROM parameter info table.
///
__code const struct parameter_info {
	const char	*name;
	param_t		default_value;
} parameter_info[PARAM_MAX] = {
	{"FORMAT",         PARAM_FORMAT_CURRENT},
	{"SERIAL_SPEED",   57}, // match APM default of 57600
	{"AIR_SPEED",      64}, // relies on MAVLink flow control
	{"NETID",          25},
	{"TXPOWER",        20},
	{"ECC",             0},
	{"MAVLINK",         1},
	{"OPPRESEND",       0},
	{"MIN_FREQ",        0},
	{"MAX_FREQ",        0},
	{"NUM_CHANNELS",    0},
	{"DUTY_CYCLE",    100},
	{"LBT_RSSI",        0},
	{"MANCHESTER",      0},
	{"RTSCTS",          0},
	{"MAX_WINDOW",    131},
#ifdef INCLUDE_AES
	{"ENCRYPTION_LEVEL", 0}, // no Enycryption (0), 128 or 256 bit key
#endif
};

/// In-RAM parameter store.
///
/// It seems painful to have to do this, but we need somewhere to
/// hold all the parameters when we're rewriting the scratchpad
/// page anyway.
///
__xdata param_t	parameter_values[PARAM_MAX];

// Three extra bytes, 1 for the number of params and 2 for the checksum
#define PARAM_FLASH_START   0
#define PARAM_FLASH_END     (PARAM_FLASH_START + sizeof(parameter_values) + 3)

#if PIN_MAX > 0
__code const pins_user_info_t pins_defaults = PINS_USER_INFO_DEFAULT;
__xdata pins_user_info_t pin_values[PIN_MAX];

// Place the start away from the other params to allow for expantion 2<<7 = 256
#define PIN_FLASH_START       (2<<7)
#define PIN_FLASH_END         (PIN_FLASH_START + sizeof(pin_values) + 3)

// Check to make sure the End of the r and the beginning of pins dont overlap
typedef char r2pCheck[(PARAM_FLASH_END < PIN_FLASH_START) ? 0 : -1];
#else // PIN_MAX
#define PIN_FLASH_END PARAM_FLASH_END
#endif // PIN_MAX

// Place the start away from the other params to allow for expantion 2<<7 +128 = 384
#ifdef INCLUDE_AES
// Holds the encrpytion string
__xdata uint8_t encryption_key[32];

#define PARAM_E_FLASH_START   (2<<7) + 128
#define PARAM_E_FLASH_END     (PARAM_E_FLASH_START + sizeof(encryption_key) + 3)

// Check to make sure the End of the pins and the beginning of encryption dont overlap
typedef char p2eCheck[(PIN_FLASH_END < PARAM_E_FLASH_START) ? 0 : -1];
#else
#define PARAM_E_FLASH_END PIN_FLASH_END
#endif // INCLUDE_AES


// Check to make sure we dont overflow off the page
typedef char endCheck[(PARAM_E_FLASH_END < 1023) ? 0 : -1];

static bool
param_check(__pdata enum ParamID id, __data uint32_t val)
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

bool
param_set(__data enum ParamID param, __pdata param_t value)
{
	// Sanity-check the parameter value first.
	if (!param_check(param, value))
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
		break;

	case PARAM_RTSCTS:
		feature_rtscts = value?true:false;
		value = feature_rtscts?1:0;
		break;

	default:
		break;
	}

	parameter_values[param] = value;

	return true;
}

param_t
param_get(__data enum ParamID param)
{
	if (param >= PARAM_MAX)
		return 0;
	return parameter_values[param];
}

bool read_params(__xdata uint8_t * __data input, uint16_t start, uint8_t size)
{
	uint16_t		i;
	
	for (i = start; i < start+size; i ++)
		input[i-start] = flash_read_scratch(i);
	
	// verify checksum
	if (crc16(size, input) != ((uint16_t) flash_read_scratch(i+1)<<8 | flash_read_scratch(i)))
		return false;
	return true;
}

void write_params(__xdata uint8_t * __data input, uint16_t start, uint8_t size)
{
	uint16_t	i, checksum;

	// save parameters to the scratch page
	for (i = start; i < start+size; i ++)
		flash_write_scratch(i, input[i-start]);
	
	// write checksum
	checksum = crc16(size, input);
	flash_write_scratch(i, checksum&0xFF);
	flash_write_scratch(i+1, checksum>>8);
}

bool
param_load(void)
__critical {
	__pdata uint8_t	i, expected;

	// Start with default values
	param_default();
	
	// loop reading the parameters array
	expected = flash_read_scratch(PARAM_FLASH_START);
	if (expected > sizeof(parameter_values) || expected < 12*sizeof(param_t))
		return false;
	
	// read and verify params
	if(!read_params((__xdata uint8_t *)parameter_values, PARAM_FLASH_START+1, expected))
		return false;
	
	// decide whether we read a supported version of the structure
	if (param_get(PARAM_FORMAT) != PARAM_FORMAT_CURRENT) {
		debug("parameter format %lu expecting %lu", parameters[PARAM_FORMAT], PARAM_FORMAT_CURRENT);
		return false;
	}

	for (i = 0; i < sizeof(parameter_values); i++) {
		if (!param_check(i, parameter_values[i])) {
			parameter_values[i] = parameter_info[i].default_value;
		}
	}
	
	// read and verify pin params
#if PIN_MAX > 0
	if(!read_params((__xdata uint8_t *)pin_values, PIN_FLASH_START+1, sizeof(pin_values)))
		return false;
#endif

  // read and verify encryption params
#ifdef INCLUDE_AES
  if(!read_params((__xdata uint8_t *)encryption_key, PARAM_E_FLASH_START+1, sizeof(encryption_key)))
    return false;
#endif // INCLUDE_AES
	return true;
}

void
param_save(void)
__critical {

	// tag parameters with the current format
	parameter_values[PARAM_FORMAT] = PARAM_FORMAT_CURRENT;

	// erase the scratch space
	flash_erase_scratch();

	// write param array length
	flash_write_scratch(PARAM_FLASH_START, sizeof(parameter_values));

	// write params
	write_params((__xdata uint8_t *)parameter_values, PARAM_FLASH_START+1, sizeof(parameter_values));

	// write pin params
#if PIN_MAX > 0
  flash_write_scratch(PIN_FLASH_START, sizeof(pin_values));
	write_params((__xdata uint8_t *)pin_values, PIN_FLASH_START+1, sizeof(pin_values));
#endif

  // write encryption params
#ifdef INCLUDE_AES
  flash_write_scratch(PARAM_E_FLASH_START, sizeof(encryption_key));
  write_params((__xdata uint8_t *)encryption_key, PARAM_E_FLASH_START+1, sizeof(encryption_key));
#endif // INCLUDE_AES

}

void
param_default(void)
{
	__pdata uint8_t	i;

	// set all parameters to their default values
	for (i = 0; i < PARAM_MAX; i++) {
		parameter_values[i] = parameter_info[i].default_value;
	}
	
#if PIN_MAX > 0
	for (i = 0; i < PIN_MAX; i ++) {
		pin_values[i].output = pins_defaults.output;
		pin_values[i].pin_dir = pins_defaults.pin_dir;
		pin_values[i].pin_mirror = pins_defaults.pin_mirror;
	}
#endif // PIN_MAX
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

// rfd900a calibration stuff
// Change for next rfd900 revision
#if defined BOARD_rfd900a || defined BOARD_rfd900p
static __at(FLASH_CALIBRATION_AREA) uint8_t __code calibration[FLASH_CALIBRATION_AREA_SIZE];
static __at(FLASH_CALIBRATION_CRC) uint8_t __code calibration_crc;

static void
flash_write_byte(uint16_t address, uint8_t c) __reentrant __critical
{
	PSCTL = 0x01;				// set PSWE, clear PSEE
	FLKEY = 0xa5;
	FLKEY = 0xf1;
	*(uint8_t __xdata *)address = c;	// write the byte
	PSCTL = 0x00;				// disable PSWE/PSEE
}

static uint8_t
flash_read_byte(uint16_t address) __reentrant
{
	// will cause reset if the byte is in a locked page
	return *(uint8_t __code *)address;
}

bool
calibration_set(uint8_t idx, uint8_t value) __reentrant
{
#ifdef CPU_SI1030
  PSBANK = 0x33;
#endif
  
	// if level is valid
	if (idx <= BOARD_MAXTXPOWER && value != 0xFF)
	{
		// if the target byte isn't yet written
		if (flash_read_byte(FLASH_CALIBRATION_AREA_HIGH + idx) == 0xFF)
		{
			flash_write_byte(FLASH_CALIBRATION_AREA_HIGH + idx, value);
			return true;
		}
	}
	return false;
}

uint8_t
calibration_get(uint8_t level) __reentrant
{
	uint8_t idx;
	uint8_t crc = 0;

#ifdef CPU_SI1030
  PSBANK = 0x33;
#endif
  
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

uint8_t
calibration_force_get(uint8_t idx) __reentrant
{
  return flash_read_byte(FLASH_CALIBRATION_AREA_HIGH + idx);
}

bool
calibration_lock() __reentrant
{
	uint8_t idx;
	uint8_t crc = 0;

#ifdef CPU_SI1030
  PSBANK = 0x33;
#endif
  
	// check that all entries are written
	if (flash_read_byte(FLASH_CALIBRATION_CRC_HIGH) == 0xFF)
	{
		for (idx=0; idx < FLASH_CALIBRATION_AREA_SIZE; idx++)
		{
			uint8_t cal = flash_read_byte(FLASH_CALIBRATION_AREA_HIGH + idx);
			crc ^= cal;
			if (cal == 0xFF)
			{
				printf("dBm level %u not calibrated\n",idx);
				return false;
			}
		}

		// write crc
		flash_write_byte(FLASH_CALIBRATION_CRC_HIGH, crc);
		// lock the first and last pages
		// can only be reverted by reflashing the bootloader
		flash_write_byte(FLASH_LOCK_BYTE, 0xFE);
		return true;
	}
	return false;
}
#endif // BOARD_rfd900a/p

#ifdef INCLUDE_AES
// Used to convert individial Hex digits into Integers
//
uint8_t read_hex_nibble(const uint8_t c) __reentrant
{
	if ((c >='0') && (c <= '9'))
	{
		return c - '0';
	}
	else if ((c >='A') && (c <= 'F'))
	{
		return c - 'A' + 10;
	}
	else if ((c >='a') && (c <= 'f'))
	{
		return c - 'a' + 10;
	}
	else
	{
		// printf("[%u] read_hex_nibble: Error char not in supported range",nodeId);
		return 0;
	}
}


/// Convert string to hex codes
///
void convert_to_hex(__xdata unsigned char *str_in, __xdata unsigned char *str_out,	__pdata uint8_t key_length)
{
	__pdata uint8_t i, num;

	for (i=0;i<key_length;i++) {
		num = read_hex_nibble(str_in[2 * i])<<4;
		num += read_hex_nibble(str_in[2 * i + 1]);
		str_out[i] = num;
	}
}

/// Set default encryption key
//
void param_set_default_encryption_key(__pdata uint8_t key_length)
{
	__pdata uint8_t i;
	__xdata uint8_t b[] = {0x62};

	for (i=0;i< key_length;i++) {
		// Set default key to b's
		memcpy(&encryption_key[i], &b, 1);
	}
}

/// set the encryption key
///
/// Note: There is a reliance on the encryption level as this determines
///       how many characters we need. So we need to set ATS16 first, THEN
///       save and then Set the encryption key.
///
bool
param_set_encryption_key(__xdata unsigned char *key)
{
	__pdata uint8_t len, key_length;

  // Use the new encryption level to help with key changes before reboot
  // Deduce key length (bytes) from level 1 -> 16, 2 -> 24, 3 -> 32
  key_length = AES_KEY_LENGTH(param_get(PARAM_ENCRYPTION));
  len = strlen(key);
  // If not enough characters (2 char per byte), then set default
  if (len < 2 * key_length ) {
    param_set_default_encryption_key(key_length);
    //printf("%s\n",key);
    printf("ERROR - Key length:%u, Required %u\n",len, 2 * key_length);
    return true;
  } else {
    // We have sufficient characters for the encryption key.
    // If too many characters, then it will just ignore extra ones
    printf("key len %d\n",key_length);
    convert_to_hex(key, encryption_key, key_length);
  }
  
  return true;
}

/// Print hex codes for given string
///
void
print_encryption_key()
{
  __pdata uint8_t i;
  __pdata uint8_t key_length = AES_KEY_LENGTH(param_get(PARAM_ENCRYPTION));
  
  for (i=0; i<key_length; i++) {
    if (0xF >= encryption_key[i]) {
      printf("0");
    }
		printf("%x",encryption_key[i]);
	}
	printf("\n");
}

/// get the encryption key
///
__xdata uint8_t* param_get_encryption_key()
{
	return encryption_key;
}
#endif // INCLUDE_AES
