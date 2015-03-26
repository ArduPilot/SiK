#ifndef CBC_ENCRYPT_DECRYPT_H
#define CBC_ENCRYPT_DECRYPT_H
//=============================================================================
// CBC_EncryptDecrypt.h
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
//=============================================================================
//-----------------------------------------------------------------------------
// typedefs for operation and status
//-----------------------------------------------------------------------------
typedef U8 CBC_ENCRYPT_DECRYPT_OPERATION;
typedef U8 CBC_ENCRYPT_DECRYPT_STATUS;
//-----------------------------------------------------------------------------
// ENCRYPT_DECRYPT_AND_SIZE_Enum defined in AES_defs.h
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// return codes defined in AES_defs.h
//-----------------------------------------------------------------------------
//=============================================================================
// Function Prototypes (API)
//=============================================================================
CBC_ENCRYPT_DECRYPT_STATUS
   CBC_EncryptDecrypt (CBC_ENCRYPT_DECRYPT_OPERATION operation,
   VARIABLE_SEGMENT_POINTER(plainText, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(cipherText, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(initialVector, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(key, U8, SEG_XDATA),
   U16 blocks);
//=============================================================================
// End of file
//=============================================================================
#endif  // #ifdef CBC_ENCRYPT_DECRYPT_H
//=============================================================================