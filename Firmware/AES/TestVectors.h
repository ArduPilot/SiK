#ifndef TEST_VECTORS_H
#define TEST_VECTORS_H
//=============================================================================
// TestVectors.h
//-----------------------------------------------------------------------------
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
//    Si102x/3x AES Code Examples
//
// This software must be used in accordance with the End User License Agreement.
//
//=============================================================================
#ifndef COMPILER_DEFS_H
#include <compiler_defs.h>
#endif
//=============================================================================
// extern'ed Public variables
//=============================================================================
extern SEGMENT_VARIABLE (ReferencePlainText[64], U8, SEG_CODE);
extern SEGMENT_VARIABLE (ReferenceEncryptionKey128[16], U8, SEG_CODE);
extern SEGMENT_VARIABLE (ReferenceEncryptionKey192[24] , U8, SEG_CODE);
extern SEGMENT_VARIABLE (ReferenceEncryptionKey256[32] , U8, SEG_CODE);
extern SEGMENT_VARIABLE (ReferenceDecryptionKey128[16], U8, SEG_CODE);
extern SEGMENT_VARIABLE (ReferenceDecryptionKey192[24] , U8, SEG_CODE);
extern SEGMENT_VARIABLE (ReferenceDecryptionKey256[32] , U8, SEG_CODE);
extern SEGMENT_VARIABLE (ReferenceCipherText_ECB_128[64], U8, SEG_CODE);
extern SEGMENT_VARIABLE (ReferenceCipherText_ECB_192[64], U8, SEG_CODE);
extern SEGMENT_VARIABLE (ReferenceCipherText_ECB_256[64], U8, SEG_CODE);
extern SEGMENT_VARIABLE (ReferenceInitialVector[16] , U8, SEG_CODE);
extern SEGMENT_VARIABLE (ReferenceCipherText_CBC_128[64] , U8, SEG_CODE);
extern SEGMENT_VARIABLE (ReferenceCipherText_CBC_192[64] , U8, SEG_CODE);
extern SEGMENT_VARIABLE (ReferenceCipherText_CBC_256[64] , U8, SEG_CODE);
extern SEGMENT_VARIABLE (Nonce[16], U8, SEG_CODE);
extern SEGMENT_VARIABLE (ReferenceCipherText_CTR_128[64], U8, SEG_CODE);
extern SEGMENT_VARIABLE (ReferenceCipherText_CTR_192[64], U8, SEG_CODE);
extern SEGMENT_VARIABLE (ReferenceCipherText_CTR_256[64], U8, SEG_CODE);
//=============================================================================
// End of file
//=============================================================================
#endif  // #ifdef TEST_VECTORS_H
//=============================================================================

