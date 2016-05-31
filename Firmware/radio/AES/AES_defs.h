//-----------------------------------------------------------------------------
//AES_defs.h
//-----------------------------------------------------------------------------
// Copyright 2011 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Program Description:
//
// DMA definitions for Si102x/3x family.
//
// Target:         Si102x/3x
// Tool chain:     Generic
// Command Line:   None
//
//-----------------------------------------------------------------------------
// Include compiler_defs.h if not already defined.
//-----------------------------------------------------------------------------
#ifndef COMPILER_DEFS_H
#include <compiler_defs.h>
#endif
//-----------------------------------------------------------------------------
// Header file applied only if not already defined.
//-----------------------------------------------------------------------------
#ifndef AES_DEFS_H
#define AES_DEFS_H
#include "../radio.h"
//-----------------------------------------------------------------------------
// enum used for KEY_SIZE
//-----------------------------------------------------------------------------
enum KEY_SIZE_Enum
{
   KEY_SIZE_128_BITS = 0,             // 0x00
   KEY_SIZE_192_BITS,                 // 0x01
   KEY_SIZE_256_BITS,                 // 0x02
   KEY_SIZE_UNDEFINED                 // 0x03
};
//-----------------------------------------------------------------------------
// enum used for ENCRYPT_DECRYPT_AND_SIZE
//-----------------------------------------------------------------------------
enum ENCRYPT_DECRYPT_AND_SIZE_Enum
{
   DECRYPTION_128_BITS = 0,            // 0x00
   DECRYPTION_192_BITS,                // 0x01
   DECRYPTION_256_BITS,                // 0x02
   DECRYPTION_UNDEFINED,               // 0x03
   ENCRYPTION_128_BITS,                // 0x04
   ENCRYPTION_192_BITS,                // 0x05
   ENCRYPTION_256_BITS,                // 0x06
   ENCRYPTION_UNDEFINED                // 0x07
};
//-----------------------------------------------------------------------------
// miscellaneous defines used for AES0BCF
//-----------------------------------------------------------------------------
#define DECRYPTION_MODE          0x00
#define ENCRYPTION_MODE          0x04
#define AES_ENABLE               0x08
//-----------------------------------------------------------------------------
// defines used for AES0DCF
//-----------------------------------------------------------------------------
#define AES_OUTPUT               0x00
#define XOR_ON_INPUT             0x01
#define XOR_ON_OUTPUT            0x02
#define INVERSE_KEY              0x04
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