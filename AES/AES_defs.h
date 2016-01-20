//-----------------------------------------------------------------------------
//AES_defs.h
//-----------------------------------------------------------------------------
// Copyright 2011 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Program Description:
//
//
// Target:         ezr32lg
// Tool chain:     Generic
// Command Line:   None
//
//-----------------------------------------------------------------------------
// Header file applied only if not already defined.
//-----------------------------------------------------------------------------
#ifndef AES_DEFS_H
#define AES_DEFS_H
//-----------------------------------------------------------------------------
// enum used for KEY_SIZE
//-----------------------------------------------------------------------------
typedef enum
{
   KEY_NOENCRYPTION = 0,
	 KEY_SIZE_128_BITS ,                // 0x01
//   KEY_SIZE_192_BITS,                 // 0x02
//   KEY_SIZE_256_BITS,                 // 0x03
   KEY_SIZE_UNDEFINED                 // 0x04
} EncryptionLevel_t;
//-----------------------------------------------------------------------------
// Define SUCCESS and ERROR_INVALID_PARAMETER return codes
//-----------------------------------------------------------------------------
#ifndef SUCCESS
#define SUCCESS 0
#elif(SUCCESS!=0)
#error  "SUCCESS definition conflict!"
#endif
#ifndef ERROR_INVALID_PARAMETER
#define ERROR_INVALID_PARAMETER 1
#elif(ERROR_INVALID_PARAMETER!=1)
#error  "ERROR_INVALID_PARAMETER definition conflict!"
#endif
//-----------------------------------------------------------------------------
// End AES_defs.h
//-----------------------------------------------------------------------------
#endif                                 // AES_DEFS_H
