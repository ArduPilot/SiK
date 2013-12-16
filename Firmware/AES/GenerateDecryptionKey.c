//=============================================================================
// GenerateDecryptionKey.c
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
#include "GenerateDecryptionKey.h"
#include "AES_defs.h"
#include "DMA_defs.h"
//=============================================================================
// Function Prototypes
//=============================================================================
//-----------------------------------------------------------------------------
// GenerateDecryptionKey()
//
// parameters:
//    encryptionKey     - xdata pointer to encryption key data source
//    decryptionKey     - xdata pointer to decryption key data destination
//    keySize           - enumerated type for key size 0/1/2 = 128/192/256
//
// returns:
//    status            - 0 for success
//                      - 1 for ERROR - Invalid Key size parameter.
//
// description:
//
// This function will generate a decryption key corresponding to a
// given encryption key.
//
// The encryption key must be stored in xdata prior to calling
// the GenerateDecryptionKey function.
//
// The encryption key must be used for all encryption operations and the
// decryption key must be used for all decryption operations.
// (Except CTR Mode which uses the encryption key for both operations.)
//
// The AES code examples uses four channels as defined in the DMA_defs.h file.
// The channel assignments are static and fixed at compile time. They are
// not assigned dynamically.
//
// This function puts the MCU core into idle mode while encryption operation
// is in process. This function is blocking. The function does not return
// until after the encrpytion operation has completed.
//
// A interrupt is needed to wake up from Idle mode. This requires that the
// global interrupts are enabled. Also, a small DMA ISR that disables EIE2
// bit 5 is required in the main module.
//
// Note that the extended portion of the extended keys comes out first,
// before the first 16 bytes. So two DMA operations are used for the
// AESYOUT channel for an extended key.
//
//-----------------------------------------------------------------------------
GENERATE_DECRYPTION_KEY_STATUS
   GenerateDecryptionKey (
   VARIABLE_SEGMENT_POINTER(encryptionKey, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(decryptionKey, U8, SEG_XDATA),
   GENERATE_DECRYPTION_KEY_SIZE keySize)
{
   // unions used for compiler independent endianness.
   UU16 addr;                          // Union used to access pointer bytes.

   U8 keyLength;                       // Used to calculate key length in bytes.

   // check first for valid operation
   if(keySize >= KEY_SIZE_UNDEFINED)
   {
      return ERROR_INVALID_PARAMETER;
   }
   else
   {
      // Calculate key length in bytes based on operation parameter.
      keyLength = ((keySize + 2) << 3);
   }

   SFRPAGE = DPPE_PAGE;

   AES0BCFG = 0x00;                      // disable, for now
   AES0DCFG = 0x00;                      // disable for now

   // Disable AES0KIN, AES0BIN, & AES0YOUT channels.
   DMA0EN &= ~AES0_KBY_MASK;

   // Configure AES key input channel using key pointer.
   // Set length to calculated key length.
   // Clear DMA0NMD to disable Key wrapping.
   // Generating the decryption key only requires encrypting one block.
   DMA0SEL  = AES0KIN_CHANNEL;
   DMA0NCF  = AES0KIN_PERIPHERAL_REQUEST;
   DMA0NMD  = NO_WRAPPING;
   addr.U16 = (U16)(encryptionKey);
   DMA0NBAL = addr.U8[LSB];
   DMA0NBAH = addr.U8[MSB];
   DMA0NSZH = 0;
   DMA0NSZL = keyLength;
   DMA0NAOL = 0;
   DMA0NAOH = 0;

   // Configure AES Block input channel.
   // Since the input data does not matter, the base address is set to
   // 0x0000 and the first 16 bytes of data in xram are used.
   // Set length to 16.
   // Clear DMA0NMD to disable Key wrapping.
   DMA0SEL  = AES0BIN_CHANNEL;
   DMA0NCF  = AES0BIN_PERIPHERAL_REQUEST;
   DMA0NMD  = NO_WRAPPING;
   DMA0NBAL = 0x00;
   DMA0NBAH = 0x00;
   DMA0NSZH = 0;
   DMA0NSZL = 16;                       // one block
   DMA0NAOL = 0;
   DMA0NAOH = 0;

   // Configure AES Y output channel for decryption key.
   // Enable DMA interrupt
   // Set length to calculated key length.
   // Clear DMA0NMD to disable Key wrapping.
   DMA0SEL = AES0YOUT_CHANNEL;
   DMA0NCF = AES0YOUT_PERIPHERAL_REQUEST|DMA_INT_EN;
   DMA0NMD = NO_WRAPPING;
   addr.U16 = (U16)(decryptionKey);
   DMA0NBAL = addr.U8[LSB];
   DMA0NBAH = addr.U8[MSB];
   DMA0NSZH = 0;
   DMA0NSZL = keyLength;
   DMA0NAOH = 0;

   // Configure starting address offset to byte 16 if extended key is used.
   if (keySize> KEY_SIZE_128_BITS)     // if extended key
      DMA0NAOL = 0x10;                 // point to extended portion
   else
      DMA0NAOL = 0x00;                 // else point to start

   // Clear KBY (Key, Block, and Y out) bits in DMA0INT sfr using mask.
   DMA0INT &= ~AES0_KBY_MASK;
   // Set KBY (Key, Block, and Y out) bits in DMA0EN sfr using mask.
   // This enables the DMA. But the encyption/decryption operation
   // won't start until the AES block is enabled.
   DMA0EN  |=  AES0_KBY_MASK;

   // Configure AES0DCFG to send inverse key to AES0YOUT
   AES0DCFG = INVERSE_KEY;

   // Set AES0BCFG according to keySize parameter.
   AES0BCFG = keySize;

   // Generating the decryption key always uses encryption mode.
   AES0BCFG |= ENCRYPTION_MODE;

   // Enabled AES module to start encryption operation.
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

   if (keySize> KEY_SIZE_128_BITS)     // if extended key
   {
      // It is reccomended to pause the AES DMA channels
      // while changing the DMA setup. The AES block cannot
      // be disabled or the rest of the key would be lost.

      // Pause DMA channels used by AES block.
      DMA0EN &= ~AES0_KBY_MASK;

      DMA0SEL = AES0YOUT_CHANNEL;      // select AES0YOUT DMA ch
      DMA0NAOL = 0;                    // reset pointer
      DMA0NSZL = 0x10;                 // set length to 16 (128-bits)

      // Clear KBXY (Key, Block, and Y out) bits in DMA0INT sfr using mask.
      DMA0INT &= ~AES0_KBXY_MASK;

      // Set AES0YOUT DMA enable bit only in DMA0EN sfr using mask.
      // This enables the DMA and the rest of the key will be written out.
      DMA0EN  |=  AES0YOUT_MASK;

      // enable DMA interrupt to terminate Idle mode
      EIE2 |= 0x20;

      // This do...while loop ensures that the CPU will remain in Idle mode
      // until AES0YOUT DMA channel transfer is complete.
      do
      {
         #ifdef DMA_TRANSFERS_USE_IDLE
         PCON |= 0x01;                 // go to Idle mode
         #endif
      }  while((DMA0INT & AES0YOUT_MASK)==0);
   }

   //Clear AES Block
   AES0BCFG = 0x00;
   AES0DCFG = 0x00;

   // Clear KBY (Key, Block, and Y out) bits in DMA0EN sfr using mask.
   DMA0EN &= ~AES0_KBY_MASK;
   // Clear KBY (Key, Block, and Y out) bits in DMA0INT sfr using mask.
   DMA0INT &= ~AES0_KBY_MASK;

   return SUCCESS;
}
//=============================================================================
// End of file
//=============================================================================



