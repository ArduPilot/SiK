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

#include "em_cmu.h"
#include "radio_old.h"
#include "tdm.h"
#include "flash_layout.h"
#include "pins_user.h"
#include "parameters.h"
#include "serial.h"
#include "printfl.h"
#include "flash.h"
#include "crc.h"
#include "aes.h"
#include "ppm.h"

// ******************** defines and typedefs *************************
typedef enum {
	CountryUnrestrict	= 0,                                                        // any channel or frequency band
	CountryAus,						                                                        // predefined channels and frequency band
	CountryNZ,						                                                        // restrict AT commands for channel number and
	CountryUSA,
	CountryEU,
	CountryLAST
}CountryCode_t;

#define NUM_FIXED_PARAMS 4
enum Param_S_ID FixedParam_S_ID[] = {
PARAM_AIR_SPEED,        // over the air baud rate
PARAM_MIN_FREQ,         // min frequency in MHz
PARAM_MAX_FREQ,         // max frequency in MHz
PARAM_NUM_CHANNELS,     // number of hopping channels
};
// Check to make sure the size matches
typedef char fpCheck[NUM_FIXED_PARAMS==((sizeof(FixedParam_S_ID)/sizeof(FixedParam_S_ID[0]))) ? 0 : -1];
typedef struct{
	uint16_t Min;
	uint16_t Max;
} ParamLimit_t;
// Three extra bytes, 1 for the number of params and 2 for the checksum
#define PARAM_S_FLASH_START   0
#define PARAM_S_FLASH_END     (PARAM_S_FLASH_START + sizeof(parameter_s_values) + 3)

// Three extra bytes, 1 for the number of params and 2 for the checksum, starts at position 128
#define PARAM_R_FLASH_START   (2<<6)
#define PARAM_R_FLASH_END     (PARAM_R_FLASH_START + sizeof(parameter_r_values) + 3)

#if PIN_MAX > 0
pins_user_info_t pin_values[PIN_MAX];
// Place the start away from the other params to allow for expantion 2<<7 = 256
#define PIN_FLASH_START       (2<<7)
#define PIN_FLASH_END         (PIN_FLASH_START + sizeof(pin_values) + 2)
#endif

static uint16_t PPM_Defaults[DMA_BUFF_LEN+1];																		// holds ppm default stream + length
#define PPM_FLASH_START   (2<<7) + 64																						// allow 64 byte for pin config (2*10 max)
#define PPPM_FLASH_END     (PPM_FLASH_START + sizeof(PPM_Defaults) + 3)

// Holds the encrpytion string
static uint8_t encryption_key[32];
// Place the start away from the other params to allow for expantion 2<<7 +128 = 384
#define PARAM_E_FLASH_START   (2<<7) + 128
#define PARAM_E_FLASH_END     (PARAM_E_FLASH_START + sizeof(encryption_key) + 3)
// Check to make sure the End of the pins and the beginning of encryption dont overlap
typedef char p2eCheck[(PIN_FLASH_END < PARAM_E_FLASH_START) ? 0 : -1];
// Check to make sure we dont overflow off the page
typedef char endCheck[(PARAM_E_FLASH_END < 1023) ? 0 : -1];

static const char * SerList[] = {"1200","2400","4800","9600","19200","38400","57600","115200","230400",NULL};
static const char * AirList[] = {"4","64","125","250",NULL};
static const char * EncList[] = {"None","128b",NULL};
static const char * BoolList[] = {"Off","On",NULL};
// ******************** local constants ******************************
/// In-ROM parameter info table.
///
static const struct parameter_s_info {
	const char	*name;
	ParamType_t ParamType;
	uint32_t 		Min;
	uint32_t 		Max;
	param_t		default_value;
	const char	** nameList;
} parameter_s_info[PARAM_S_MAX] = {
	{"FORMAT",          PT_Int ,0     ,0xff  ,PARAM_FORMAT_CURRENT,NULL    },
	{"SERIAL_SPEED",    PT_List,1     ,460   ,57                  ,SerList }, // match APM default of 57600
	{"AIR_SPEED",       PT_List,4     ,500   ,64                  ,AirList }, // relies on MAVLink flow control
	{"NETID",           PT_Int ,0     ,0xff  ,25                  ,NULL    },
	{"TXPOWER",         PT_Int ,0     ,30    ,30                  ,NULL    },
	{"ECC",             PT_Bool,0     ,1     ,0                   ,BoolList},
	{"MAVLINK",         PT_Bool,0     ,1     ,1                   ,BoolList},
	{"OPPRESEND",       PT_Bool,0     ,1     ,0                   ,BoolList},
	{"MIN_FREQ",        PT_Int ,902000,929000,915000              ,NULL    },
	{"MAX_FREQ",        PT_Int ,902000,929000,928000              ,NULL    },
	{"NUM_CHANNELS",    PT_Int ,1     ,50    ,20                  ,NULL    },
	{"DUTY_CYCLE",      PT_Int ,1     ,100   ,100                 ,NULL    },
	{"LBT_RSSI",        PT_Int ,0     ,0xff  ,0                   ,NULL    },
	//{"MANCHESTER",      PT_Bool,0     ,1     ,0                   ,BoolList},
	{"RTSCTS",          PT_Bool,0     ,1     ,0                   ,BoolList},
	{"MAX_WINDOW",      PT_Int ,20    ,400   ,131                 ,NULL    },
  {"ENCRYPTION_LEVEL",PT_List,0     ,1     ,0                   ,EncList }, // no Enycryption (0), 128 or 256 bit key
  {"GPI1_1R/CIN",     PT_Bool,0     ,1     ,0                   ,BoolList},
  {"GPO1_1R/COUT",    PT_Bool,0     ,1     ,0                   ,BoolList},
};

static const struct parameter_r_info {
	const char	*name;
	param_t		default_value;
} parameter_r_info[PARAM_R_MAX] = {
	{"TARGET_RSSI",     255},
	{"HYSTERESIS_RSSI", 50},
};

static const ParamLimit_t FixedCountryParams[CountryLAST][NUM_FIXED_PARAMS] =
{ {{ 4,1000},{902,902},{928,928},{1,50}},
	{{ 4,1000},{915,915},{928,928},{20,20}},
	{{ 4,1000},{921,921},{929,929},{20,20}},
	{{ 4,1000},{902,902},{928,928},{50,50}},
	{{ 4,1000},{863,863},{870,870},{50,50}}};

typedef char FCPCheck[(sizeof(FixedCountryParams) ==  (sizeof(ParamLimit_t)*CountryLAST*NUM_FIXED_PARAMS)) ? 0 : -1];

static const uint8_t default_key[32]=
  { 0x60, 0x3D, 0xEB, 0x10, 0x15, 0xCA, 0x71, 0xBE,
    0x2B, 0x73, 0xAE, 0xF0, 0x85, 0x7D, 0x77, 0x81,
    0x1F, 0x35, 0x2C, 0x07, 0x3B, 0x61, 0x08, 0xD7,
    0x2D, 0x98, 0x10, 0xA3, 0x09, 0x14, 0xDF, 0xF4 };

#if PIN_MAX > 0
static const pins_user_info_t pins_defaults = PINS_USER_INFO_DEFAULT;
#endif
// ******************** local variables ******************************
/// In-RAM parameter store.
///
/// It seems painful to have to do this, but we need somewhere to
/// hold all the parameters when we're rewriting the scratchpad
/// page anyway.
///
static param_t	parameter_s_values[PARAM_S_MAX];
static param_t	parameter_r_values[PARAM_R_MAX];

// ******************** global variables *****************************
// ******************** local function prototypes ********************
static void param_set_default_encryption_key(void);
static int16_t GetFixedParamIdx(enum Param_S_ID id);
static const ParamLimit_t * GetFixedParam(enum Param_S_ID id);
// ********************* Implementation ******************************

__STATIC_INLINE uint8_t GetCountry(void)																				// get country, 0 is any country
{
	uint8_t val = calibration_get(CalParam_Country);
	if((val >= CountryUnrestrict)&&(val < CountryLAST))
		return(val);
	return(0);
}

__STATIC_INLINE int16_t FixedParamIdx(enum Param_S_ID id)												// get param idx, -ve is not valid
{
	int16_t i;
	for(i=(sizeof(FixedParam_S_ID)/sizeof(FixedParam_S_ID[0]))-1;i>=0;i--)
	{
		if(id == FixedParam_S_ID[i])
			break;
	}
	return(i);	// will be -ve if not found
}

static int16_t GetFixedParamIdx(enum Param_S_ID id)															// get param idx if country is fixed
{
	if(GetCountry() > 0)
	{
		return(FixedParamIdx(id));
	}
	return(-1);
}

static const ParamLimit_t* GetFixedParam(enum Param_S_ID id)																// get fix parameter -ve is not valid
{
	int16_t idx = GetFixedParamIdx(id);
	if(idx >= 0)
	{
		return(&FixedCountryParams[GetCountry()][idx]);
	}
	return(NULL);
}

static bool param_s_check(enum Param_S_ID id, uint32_t val)
{
	// parameter value out of range - fail
	if (id >= PARAM_S_MAX)
		return false;
	const ParamLimit_t* Limit = GetFixedParam(id);
	if(Limit != NULL)
	{
		if(val < Limit->Min) return false;
		if(val > Limit->Max) return false;
		return true;
	}

	switch (id) {
	case PARAM_FORMAT:
		if(PARAM_FORMAT_CURRENT != val)
			return false;
		break;

	case PARAM_SERIAL_SPEED:
		return serial_device_valid_speed(val);

	case PARAM_AIR_SPEED:
		if(!radio_RateValid(val))
			return false;
		break;

	case PARAM_NETID:
		// all values are OK
		return true;

	case PARAM_TXPOWER:
		if (val > BOARD_MAXTXPOWER)
			return false;
		break;

	case PARAM_RCIN:
	case PARAM_RCOUT:
		if(parameter_s_values[(id==PARAM_RCOUT)?(PARAM_RCIN):(PARAM_RCOUT)])
			return false;
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
	case PARAM_ENCRYPTION:
		if (val >= KEY_SIZE_UNDEFINED)
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

	case PARAM_ENCRYPTION:
		 aes_init(value);
		break;
	default:
		break;
	}

	parameter_s_values[param] = value;

	return true;
}

param_t param_s_get(enum Param_S_ID param)
{
	param_t val;
	const ParamLimit_t *Limit;
	if (param >= PARAM_S_MAX)
		return 0;
	val = parameter_s_values[param];
	if((Limit=GetFixedParam(param)) != NULL)
	{
		if(val < Limit->Min) val = Limit->Min;
		else if(val > Limit->Max) val = Limit->Max;
	}
	return(val);
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

static bool read_params(uint8_t * input, uint16_t start, uint8_t size)
{
	uint16_t		i;
	
	for (i = start; i < start+size; i ++){
		input[i-start] = flash_read_scratch(i);
		//printf("%d-%d\n",i,input[i-start]);
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

bool param_load(void)
{
	uint8_t	i;

	// Start with default values
	param_default();
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
#if PIN_MAX > 0
	expected = flash_read_scratch(PIN_FLASH_START);
	if (expected != sizeof(pin_values))
		return false;
	if(!read_params((uint8_t *)pin_values, PIN_FLASH_START+1, sizeof(pin_values)))
		return false;
#endif
  if(!read_params((uint8_t *)PPM_Defaults, PPM_FLASH_START+1, sizeof(PPM_Defaults)))
    return false;
  // read and verify encryption params
  if(!read_params((uint8_t *)encryption_key, PARAM_E_FLASH_START+1, sizeof(encryption_key)))
    return false;

	return true;
}

void param_save(void)
{
	// tag parameters with the current format
	parameter_s_values[PARAM_FORMAT] = PARAM_FORMAT_CURRENT;

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
  flash_write_scratch(PPM_FLASH_START, sizeof(PPM_Defaults));
  write_params((uint8_t *)PPM_Defaults, PPM_FLASH_START+1, sizeof(PPM_Defaults));
  // write encryption params
  flash_write_scratch(PARAM_E_FLASH_START, sizeof(encryption_key));
  write_params((uint8_t *)encryption_key, PARAM_E_FLASH_START+1, sizeof(encryption_key));

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
	PPM_Read_Defaults(PPM_Defaults);

	param_set_default_encryption_key();
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

const char ** param_s_nameList(enum Param_S_ID param)
{
	if (param < PARAM_S_MAX) {
		return parameter_s_info[param].nameList;
	}
	return 0;
}

ParamType_t param_s_Type(enum Param_S_ID param)
{
	if (param < PARAM_S_MAX) {
		return parameter_s_info[param].ParamType;
	}
	return 0;
}

uint32_t param_s_Min(enum Param_S_ID param)
{
	if (param < PARAM_S_MAX) {
		return parameter_s_info[param].Min;
	}
	return 0;
}

uint32_t param_s_Max(enum Param_S_ID param)
{
	if (param < PARAM_S_MAX) {
		return parameter_s_info[param].Max;
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

	// if level is valid
	if(((idx <= BOARD_MAXTXPOWER) && (value != 0xFF))||
	   ((idx == CalParam_BAND) && BoardFrequencyValid(value))||
	   ((idx == CalParam_Country) && (value < CountryLAST)) )
	{
		// if the target byte isn't yet written
		if (flash_read_byte(FLASH_CALIBRATION_AREA + idx) == 0xFF)
		{
			flash_write_byte(FLASH_CALIBRATION_AREA + idx, value);
			return true;
		}
	}
	return false;
}

uint8_t calibration_get(uint8_t level)
{
	//uint8_t idx;
	//uint8_t crc = 0;
	uint8_t *calibration = (uint8_t *)(FLASH_CALIBRATION_AREA);
#if 0
	uint8_t calibration_crc = flash_read_byte(FLASH_CALIBRATION_CRC);

	// Change for next board revision
	for (idx = 0; idx < FLASH_CALIBRATION_AREA_SIZE; idx++)
	{
		crc ^= calibration[idx];
	}
#endif
	if(((/*calibration_crc != 0xFF && calibration_crc == crc && */(calibration[level] != 0xff) && level <= BOARD_MAXTXPOWER))||
		 ((level >= CalParam_BAND)&&(level < CalParam_LAST)) )
	{
		return calibration[level];
	}
	return 0xFF;
}

bool calibration_lock(void)
{
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
		// can only be reverted by clearing block using bootloader
		FlashLockBlock((uint8_t *)FLASH_CALIBRATION_AREA);
		return true;
	}
	return false;
}

// Used to convert individial Hex digits into Integers
//
uint8_t read_hex_nibble(const uint8_t c)
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
void convert_to_hex(unsigned char *str_in, unsigned char *str_out,	uint8_t key_length)
{
	uint8_t i, num;

	for (i=0;i<key_length;i++) {
		num = read_hex_nibble(str_in[2 * i])<<4;
		num += read_hex_nibble(str_in[2 * i + 1]);
		str_out[i] = num;
	}
}

/// Set default encryption key
//
static void param_set_default_encryption_key(void)
{
	memcpy(encryption_key,default_key,sizeof(encryption_key));
}

/// set the encryption key
///
/// Note: There is a reliance on the encryption level as this determines
///       how many characters we need. So we need to set ATS16 first, THEN
///       save and then Set the encryption key.
///
bool
param_set_encryption_key(unsigned char *key)
{
	uint8_t len, key_length;

  // Use the new encryption level to help with key changes before reboot
  // Deduce key length (bytes) from level 1 -> 16, 2 -> 24, 3 -> 32
  key_length = AES_KEY_LENGTH(param_s_get(PARAM_ENCRYPTION));
  len = strlen((char*)key);
  // If not enough characters (2 char per byte), then set default
  if (len < 2 * key_length ) {
    param_set_default_encryption_key();
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
  uint8_t i;
  uint8_t key_length = AES_KEY_LENGTH(param_s_get(PARAM_ENCRYPTION));
  
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
uint8_t* param_get_encryption_key()
{
	return encryption_key;
}

void param_get_PPMDefaults(uint16_t *Data,uint16_t* Len)
{
	*Len = PPM_Defaults[DMA_BUFF_LEN];
	memcpy(Data,PPM_Defaults,(*Len)<<1);
}

void param_set_PPMDefaults(uint16_t *Data, uint16_t Len)
{
	PPM_Defaults[DMA_BUFF_LEN] = Len;
	memcpy(PPM_Defaults,Data,Len<<1);
}

