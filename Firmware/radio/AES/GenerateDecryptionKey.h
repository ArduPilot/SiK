#ifndef GENERATE_DECRYPTION_KEY_H
#define GENERATE_DECRYPTION_KEY_H
//=============================================================================
// GenerateDecryptionKey.h
//=============================================================================
// Copyright 2011 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// C File Description:
//
//
// Target:
//    Si102x/3x
//
// IDE:
//    Silicon Laboratories IDE
//
// Tool Chains:
//    Keil
//    SDCC
//    Raisonance
//
// Project Name:
//    Si102x/3x AES Library
//
// This software must be used in accordance with the End User License Agreement.
//
//=============================================================================
#ifndef COMPILER_DEFS_H
#include <compiler_defs.h>
#endif
#ifndef AES_DEFS_H
#include "AES_defs.h"
#endif
//-----------------------------------------------------------------------------
// typedefs for key size and status
//-----------------------------------------------------------------------------
typedef U8 GENERATE_DECRYPTION_KEY_SIZE;
typedef U8 GENERATE_DECRYPTION_KEY_STATUS;
//-----------------------------------------------------------------------------
// ENCRYPT_DECRYPT_AND_SIZE_Enum defined in AES_defs.h
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// return codes defined in AES_defs.h
//-----------------------------------------------------------------------------
//=============================================================================
// Function Prototypes (API)
//=============================================================================
GENERATE_DECRYPTION_KEY_STATUS
   GenerateDecryptionKey (
   VARIABLE_SEGMENT_POINTER(encryptionKey, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(decryptionKey, U8, SEG_XDATA),
   U8 GENERATE_DECRYPTION_KEY_SIZE);
//=============================================================================
// End of file
//=============================================================================
#endif  // #ifdef GENERATE_DECRYPTION_KEY_H
//=============================================================================