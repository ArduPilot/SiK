//=============================================================================
// AES_BlockCipher.c
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
//    Si102x/3x AES Code Examples
//
// This software must be used in accordance with the End User License Agreement.
//
//=============================================================================
//-----------------------------------------------------------------------------
// uncomment pragma SRC to generate assembly code
//-----------------------------------------------------------------------------
//#pragma SRC
//=============================================================================
// Includes
//=============================================================================
#include <compiler_defs.h>
#include <Si1020_defs.h>
#include "AES_BlockCipher.h"
#include "DMA_defs.h"
#include "AES_defs.h"
//=============================================================================
// API Functions
//=============================================================================
//-----------------------------------------------------------------------------
// AES_BlockCipher()
//
// Parameters:
//    operation       - decryption/encrypt & 128/192/256 options
//    plainText       - xdata pointer to plainText
//    cipherText      - xdata pointer to cipherText
//    key             - xdata pointer to encryption or decryption key
//
// Returns:
//    status         - 0 for success
//                   - 1 for ERROR - Invalid operation parameter.
//
// Description:
//
// This function performs AES Block Cipher Encryption or Decryption of
// a specified number of 16-byte blocks.
//
// The PlainText, and encryption key must be stored in xdata prior to calling
// an encryption operation.
//
// Note that the plainText pointer is a pointer to the source data for an
// encryption operation and a pointer to the destination data for a decryption
// operation.
//
// The cipherText and decryption key must be stored in xdata prior to calling
// an decryption operation.
//
// Note that the cipherText pointer is a pointer to the destination data
// for an encryption operation and a pointer to the source data for a decryption
// operation.
//
// This function will return an error if the operation parameter is invalid.
// Using the error code is optional, but will aid in debugging.
//
// The AES code examples uses four channels as defined in the DMA-defs.h file.
// The channel assignments are static and fixed at compile time. They are
// not assigned dynamically.
//
// This function puts the MCU core into idle mode while encryption/decryption
// is in process. This function is blocking. The function does not return
// until after the encrpytion/decryption operation has completed.
//
// A interrupt is needed to wake up from Idle mode. This requires that the
// global interrupts are enabled. Also, a small DMA ISR that disables EIE2
// bit 5 is required in the main module.
//
//-----------------------------------------------------------------------------
AES_BLOCK_CIPHER_STATUS AES_BlockCipher (AES_BLOCK_CIPHER_OPERATION operation,
   VARIABLE_SEGMENT_POINTER(plainText, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(cipherText, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(key, U8, SEG_XDATA),
    U16 blocks)
{
   // unions used for compiler independent endianness.
   UU16 length;                        // Length in bytes for all blocks.
   UU16 addr;                          // Union used to access pointer bytes.

   U8 keyLength;                       // Used to calculate key length in bytes.

   // check first for valid key type
   if((operation == DECRYPTION_UNDEFINED)||(operation >= ENCRYPTION_UNDEFINED))
   {
      return ERROR_INVALID_PARAMETER;
   }
   else
   {
      // Calculate key length in bytes based on operation parameter.
      keyLength = (((operation & 0x03) + 2) << 3);
   }

   // Calculate plaintext and ciphertext total length.
   length.U16 = (blocks << 4);

   SFRPAGE = DPPE_PAGE;

   AES0BCFG = 0x00;                    // Disable for now
   AES0DCFG = 0x00;                    // Disable for now

   // Disable AES0KIN, AES0BIN, & AES0YOUT channels.
   DMA0EN &= ~AES0_KBY_MASK;

   // Configure AES key input channel using key pointer and calculated key
   // length. Set DMA0NMD to enable Key wrapping. This permits multiple
   // blocks to be encrypted using the same key.

   DMA0SEL = AES0KIN_CHANNEL;
   DMA0NCF = AES0KIN_PERIPHERAL_REQUEST;
   DMA0NMD = WRAPPING;
   addr.U16 = (U16)(key);
   DMA0NBAL = addr.U8[LSB];
   DMA0NBAH = addr.U8[MSB];
   DMA0NSZH = 0;
   DMA0NSZL = keyLength;
   DMA0NAOL = 0;
   DMA0NAOH = 0;

   // AES block input is plaintext for encryption operation or ciphertext
   // for decyption operation.

  if(operation & ENCRYPTION_MODE)
      addr.U16 = (U16)(plainText);
   else
      addr.U16 = (U16)(cipherText);

   // Configure AES block input channel using corresponding address
   // and calculated total plaintext/ciphertext length.
   // Clear DMA0NMD to disable wrapping. Each consecutive block
   // will be encypted/decrypted using the same key.

   DMA0SEL = AES0BIN_CHANNEL;
   DMA0NCF = AES0BIN_PERIPHERAL_REQUEST;
   DMA0NMD = NO_WRAPPING;
   DMA0NBAL = addr.U8[LSB];
   DMA0NBAH = addr.U8[MSB];
   DMA0NSZL = length.U8[LSB];
   DMA0NSZH = length.U8[MSB];
   DMA0NAOL = 0;
   DMA0NAOH = 0;

   // AES Y ouput is ciphertext  for encryption operation or
   // plaintext for decyption operation.

   if(operation & ENCRYPTION_MODE)
      addr.U16 = (U16)(cipherText);
   else
      addr.U16 = (U16)(plainText);

   // Configure AES Y output channel using corresponding address
   // and calculated total plaintext/ciphertext length.
   // Clear DMA0NMD to disable wrapping. Each consecutive block
   // will be encypted/decrypted using the same key.

   DMA0SEL = AES0YOUT_CHANNEL;
   DMA0NCF = AES0YOUT_PERIPHERAL_REQUEST|DMA_INT_EN;
   DMA0NMD = NO_WRAPPING;
   DMA0NBAL = addr.U8[LSB];
   DMA0NBAH = addr.U8[MSB];
   DMA0NSZL = length.U8[LSB];
   DMA0NSZH = length.U8[MSB];
   DMA0NAOH = 0;
   DMA0NAOL = 0;

   // Clear KBY (Key, Block, and Y out) bits in DMA0INT sfr using mask.
   DMA0INT &= ~AES0_KBY_MASK;
   // Set KBY (Key, Block, and Y out) bits in DMA0EN sfr using mask.
   // This enables the DMA. But the encyption/decryption operation
   // won't start until the AES block is enabled.
   DMA0EN  |= AES0_KBY_MASK;

   // Configure AES0DCFG for normal AES Block cipher mode.
   // AES0TOUT is the AES output from the AES core.
   AES0DCFG = AES_OUTPUT;

   // Configure AES0BCFG for encyption or decryption operation according
   // to operation parameter.
   AES0BCFG = operation;

   // Enabled AES module to start encryption/decryption operation.
   AES0BCFG |= AES_ENABLE;

   // enable DMA interrupt to terminate Idle mode
   EIE2 |= 0x20;

   // This do...while loop ensures that the CPU will remain in Idle mode
   // until AES0YOUT DMA channel transfer is complete.

   do
   {
      #ifdef DMA_TRANSFERS_USE_IDLE
      PCON |= 0x01;                    // go to Idle mode
      #endif

   }  while((DMA0INT & AES0YOUT_MASK)==0);

   //Clear AES Block
   AES0BCFG = 0x00;
   AES0DCFG = 0x00;

   // Clear KBY (Key, Block, and Y out) bits in DMA0EN sfr using mask.
   DMA0EN &= ~AES0_KBY_MASK;
   // Clear KBY (Key, Block, and Y out) bits in DMA0INT sfr using mask.
   DMA0INT &= ~AES0_KBY_MASK;

   return SUCCESS;                     // Success!!
}
//=============================================================================
// End of file
//=============================================================================

