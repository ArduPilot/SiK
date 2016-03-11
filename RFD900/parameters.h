#ifndef PARAMETERS_H_
#define PARAMETERS_H_
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
/// @file	parameters.h
///
/// Definitions for program parameter storage.
///

/// Parameter IDs.
///
/// Parameter IDs here match AT S-register numbers, so change them with extreme
/// care.  Parameter zero cannot be written by AT commands.
///
/// Note that this enumeration is used to initialise the parameter_names
/// array, and parameters not listed in that array will not be visible by name.
///
/// When adding or removing a parameter here, you must also update:
///
///   parameters.c:parameter_names[]
///   parameters.c:param_check()
///

#include "board_rfd900e.h"

enum Param_S_ID {
	PARAM_FORMAT = 0,       // Must always be parameter 0
	PARAM_SERIAL_SPEED,     // BAUD_RATE_* constant
	PARAM_AIR_SPEED,        // over the air baud rate
	PARAM_NETID,            // network ID
	PARAM_TXPOWER,          // transmit power (dBm)
	PARAM_ECC,              // ECC using golay encoding
	PARAM_MAVLINK,          // MAVLink framing, 0=ignore, 1=use, 2=rc-override
	PARAM_OPPRESEND,        // opportunistic resend
	PARAM_MIN_FREQ,         // min frequency in MHz
	PARAM_MAX_FREQ,         // max frequency in MHz
	PARAM_NUM_CHANNELS,     // number of hopping channels
	PARAM_DUTY_CYCLE,       // duty cycle (percentage)
	PARAM_LBT_RSSI,         // listen before talk threshold
//	PARAM_MANCHESTER,       // enable manchester encoding
	PARAM_RTSCTS,           // enable hardware flow control
	PARAM_MAX_WINDOW,       // The maximum window size allowed
	PARAM_ENCRYPTION,     	// no Enycryption (0), 128 or 256 bit key
	PARAM_RCIN,     				// enable RC PPM Input
	PARAM_RCOUT,     				// enable RC PPM Output
	PARAM_S_MAX             // must be last
};

enum Param_R_ID {
	PARAM_R_TARGET_RSSI =0, // Change power dynamically to matain target RSSI
	PARAM_R_HYSTERESIS_RSSI,// Hysteresis on the dynamic RSSI
	PARAM_R_MAX             // Must be last
};

typedef enum{
	CalParam_BAND=BOARD_MAXTXPOWER+1,
	CalParam_Country,
	CalParam_LAST,
}	CalParm_t;

typedef enum{
	PT_Int=0,
	PT_Bool,
	PT_List,
	PT_Last,
	PT_SIZEFIX = 0xff
} ParamType_t;


#define PARAM_FORMAT_CURRENT  0x20UL  //< current parameter format ID

/// Parameter type.
///
/// All parameters have this type.
///
typedef uint32_t	param_t;
#if PIN_MAX > 0
extern pins_user_info_t pin_values[];
#endif

/// Set a parameter
///
/// @note Parameters are not saved until param_save is called.
///
/// @param	param		The parameter to set.
/// @param	value		The value to assign to the parameter.
/// @return			True if the parameter's value is valid.
///
extern bool param_s_set(enum Param_S_ID param, param_t value);
extern bool param_r_set(enum Param_R_ID param, param_t value);

/// Get a parameter
///
/// @param	param		The parameter to get.
/// @return			The parameter value, or zero if the param
///				argument is invalid.
///
extern param_t param_s_get(enum Param_S_ID param);
extern param_t param_r_get(enum Param_R_ID param);

/// Look up a parameter by name
///
/// @param	name		The parameter name
/// @return			The parameter ID, or PARAM_MAX if the
///				parameter is not known.
///
extern enum Param_S_ID param_s_id(char * name);
extern enum Param_R_ID param_r_id(char * name);

/// Return the name of a parameter.
///
/// @param	param		The parameter ID to look up.
/// @return			A pointer to the name of the parameter,
///				or NULL if the parameter is not known.
///
extern const char *param_s_name(enum Param_S_ID param);
extern const char *param_r_name(enum Param_R_ID param);
uint32_t param_s_Max(enum Param_S_ID param);
uint32_t param_s_Min(enum Param_S_ID param);
ParamType_t param_s_Type(enum Param_S_ID param);
const char ** param_s_nameList(enum Param_S_ID param);

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
/// Note that this just resets - it does not save.
///
extern void param_default(void);

/// convenient routine to constrain parameter values
uint32_t constrain(uint32_t v, uint32_t min, uint32_t max);

//#if defined BOARD_rfd900a || defined BOARD_rfd900p
extern bool calibration_set(uint8_t idx, uint8_t value);
extern uint8_t calibration_get(uint8_t level);
extern bool calibration_lock();

extern void param_get_PPMDefaults(uint16_t *Data,uint16_t* Len);
extern void param_set_PPMDefaults(uint16_t *Data, uint16_t Len);

/// get the encryption key
///
extern uint8_t *param_get_encryption_key();

/// set the encryption key
///
extern bool param_set_encryption_key(unsigned char *key);

/// Print hex codes
///
extern void print_encryption_key();
#endif
