//=============================================================================
// CTR_EncryptDecrypt.c
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
#include "CTR_EncryptDecrypt.h"
#include "DMA_defs.h"
#include "AES_defs.h"
//=============================================================================
// local function prototypes
//=============================================================================
void IncrementCounter (VARIABLE_SEGMENT_POINTER(counter, U8, SEG_XDATA));
//=============================================================================
// API Functions
//=============================================================================
//-----------------------------------------------------------------------------
// CTR_EncryptDecrypt()
//
// parameters:
//    operation       - decryption/encrypt & 128/192/256 options
//    plainText       - xdata pointer to plainText
//    cipherText      - xdata pointer to cipherText
//    counter         - xdata pointer to counter
//    encryptionKey   - xdata pointer to encryption key
//
// returns:
//    status         - 0 for success
//                   - 1 for ERROR - Invalid operation parameter.
//
// description:
//
// This function performs Counter (CTR) Mode Encryption or Decryption of a
// specified number of 16-byte blocks.
//
// The plaintext, and encryption key must be stored in xdata prior to calling
// an encryption operation.
//
// Note that the plainText pointer is a pointer to the source data for an
// encryption operation and a pointer to the destination data for a decryption
// operation.
//
// The ciphertext and encryption key must be stored in xdata prior to calling
// an decryption operation. The CTR mode decryption operation uses the AES
// core in encryption mode. So the encryption key is always used for any CTR
// operation.
//
// The counter should be initialized using a nonce at the start of an
// encryption sequence.
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
// until after the encryption/decryption operation has completed.
//
// A interrupt is needed to wake up from Idle mode. This requires that the
// global interrupts are enabled. Also, a small DMA ISR that disables EIE2
// bit 5 is required in the main module.
//
// Note that CTR encryption/decryption requires one DMA transfer per block.
// It is not possible to encrypt/decrypt multiple blocks using a single DMA
// transfer because the counter must be incremented between blocks.
//
//-----------------------------------------------------------------------------
CTR_ENCRYPT_DECRYPT_STATUS
   CTR_EncryptDecrypt (CTR_ENCRYPT_DECRYPT_OPERATION operation,
   VARIABLE_SEGMENT_POINTER(plainText, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(cipherText, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(counter, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(encryptionKey, U8, SEG_XDATA),
   U16 blocks)
{
   // Unions used for compiler independent endianness.
   UU16 length;                        // Length in bytes for all blocks.
   UU16 addr;                          // Union used to access pointer bytes.

   U8 keyLength;                       // Used to calculate key length in bytes.

   // Check first for valid operation.
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
   // Using <<4 in lieu of * 16 for code efficiency.
   length.U16 = (blocks << 4);

   // From this point on, "blocks" is used to count remaining blocks.
   blocks--;

   SFRPAGE = DPPE_PAGE;

   AES0BCFG = 0x00;                      // disable, for now
   AES0DCFG = 0x00;                      // disable for now

   // Disable AES0KIN, AES0BIN, AES0XIN, & AES0YOUT channels.
   DMA0EN &= ~AES0_KBXY_MASK;

   // Configure AES key input channel using key pointer.
   // Set length to calculated key length.
   // Set DMA0NMD to disable Key wrapping.
   // This is necessary because we want the Key DMA channel
   // to stop after the first block.

   addr.U16 = (U16)(encryptionKey);
   DMA0SEL = AES0KIN_CHANNEL;
   DMA0NCF = AES0KIN_PERIPHERAL_REQUEST;
   DMA0NMD = NO_WRAPPING;
   DMA0NBAL = addr.U8[LSB];
   DMA0NBAH = addr.U8[MSB];
   DMA0NSZH = 0;
   DMA0NSZL = keyLength;
   DMA0NAOL = 0;
   DMA0NAOH = 0;

   // AES block input is always the counter value for both CTR encryption
   // and decryption. Set length to 16 for first block only.
   // Clear DMA0NMD to disable counter wrapping.
   // This is necessary because we want the Counter DMA channel
   // to stop after the first block.

   DMA0SEL = AES0BIN_CHANNEL;
   DMA0NCF = AES0BIN_PERIPHERAL_REQUEST;
   DMA0NMD = NO_WRAPPING;
   addr.U16 = (U16)(counter);
   DMA0NBAL = addr.U8[LSB];
   DMA0NBAH = addr.U8[MSB];
   DMA0NSZL = 16;                      // one block
   DMA0NSZH = 0;
   DMA0NAOL = 0;
   DMA0NAOH = 0;

   // AES XOR input is plaintext for encryption operation or ciphertext
   // for decryption operation.

   if(operation & ENCRYPTION_MODE)
      addr.U16 = (U16)(plainText);
   else
      addr.U16 = (U16)(cipherText);

   // Configure AES XOR input channel using corresponding address.
   // Clear DMA0NMD to disable wrapping.

   DMA0SEL = AES0XIN_CHANNEL;
   DMA0NCF = AES0XIN_PERIPHERAL_REQUEST;
   DMA0NMD = NO_WRAPPING;
   DMA0NBAL = addr.U8[LSB];
   DMA0NBAH = addr.U8[MSB];
   DMA0NSZL = 16;                      // one block
   DMA0NSZH = 0;
   DMA0NAOL = 0;
   DMA0NAOH = 0;

   // AES Y output is ciphertext  for encryption operation or
   // plaintext for decryption operation.

   if(operation & ENCRYPTION_MODE)
      addr.U16 = (U16)(cipherText);
   else
      addr.U16 = (U16)(plainText);

   // Configure AES Y output channel using corresponding address
   // Set length to 16 for first block only.
   // Clear DMA0NMD to disable wrapping.

   DMA0SEL = AES0YOUT_CHANNEL;
   DMA0NCF = AES0YOUT_PERIPHERAL_REQUEST|DMA_INT_EN;
   DMA0NMD = NO_WRAPPING;

   DMA0NBAL = addr.U8[LSB];
   DMA0NBAH = addr.U8[MSB];
   DMA0NSZL = 16;                      // one block
   DMA0NSZH = 0;
   DMA0NAOH = 0;
   DMA0NAOL = 0;

   // Clear KBXY (Key, Block, X in, and Y out) bits in DMA0INT sfr using mask.
   DMA0INT &= ~AES0_KBXY_MASK;


   // Set KBXY (Key, Block, Xin, and Y out) bits in DMA0EN sfr using mask.
   // This enables the DMA. But the encryption/decryption operation
   // won't start until the AES block is enabled.
   DMA0EN  |=  AES0_KBXY_MASK;

   // Configure data path for XOR on output. CTR mode always has the
   // XOR on the output for both encryption and decryption.
   AES0DCFG = XOR_ON_OUTPUT;

   // Configure AES0BCFG for encryption/decryption and requested key size.
   // Note the encryption mode bit is set explicitly.
   AES0BCFG = operation;
   AES0BCFG |= ENCRYPTION_MODE;
   AES0BCFG |= AES_ENABLE;

    // Enable DMA interrupt to terminate Idle mode.
    EIE2 |= 0x20;

   // This do...while loop ensures that the CPU will remain in Idle mode
   // until AES0YOUT DMA channel transfer is complete.
   do
   {
      #ifdef DMA_TRANSFERS_USE_IDLE
      PCON |= 0x01;                    // go to Idle mode
      #endif
   }  while((DMA0INT&AES0YOUT_MASK)==0);

   while(blocks--)                     // if blocks remaining
   {
      // It is necessary to either pause the AES DMA channels
      // or disable the AES block while changing the setup.
      // Both steps are taken in this example to be extra safe.

      // Disable AES block.
      // This also clears the AES contents and resets the state machine.
      // It is not necessary to reset the AES core between blocks.
      // But it is recommended to disable the core while changing the set-up.

      AES0BCFG &= ~AES_ENABLE;

      // Pause DMA channels used by AES block.
      DMA0EN &= ~AES0_KBXY_MASK;

      IncrementCounter(counter);

      SFRPAGE = DPPE_PAGE;

      // AESKIN DMA channel reset address offset
      DMA0SEL = AES0KIN_CHANNEL;
      DMA0NAOL = 0;
      DMA0NAOH = 0;

      // AESBIN DMA channel reset address offset
      DMA0SEL = AES0BIN_CHANNEL;
      DMA0NAOL = 0;
      DMA0NAOH = 0;

      // Ch 2 AES0XIN increment size by 16 bytes
      DMA0SEL = AES0XIN_CHANNEL;
      length.U8[LSB] = DMA0NSZL;
      length.U8[MSB] = DMA0NSZH;
      length.U16 += 16;
      DMA0NSZL = length.U8[LSB];
      DMA0NSZH = length.U8[MSB];

     // Ch 3 AESYOUT increment size by 16 bytes
      DMA0SEL = AES0YOUT_CHANNEL;
      length.U8[LSB] = DMA0NSZL;
      length.U8[MSB] = DMA0NSZH;
      length.U16 += 16;
      DMA0NSZL = length.U8[LSB];
      DMA0NSZH = length.U8[MSB];

      // Clear KBXY (Key, Block, X in, and Y out) bits in DMA0INT sfr using mask.
      DMA0INT &= ~AES0_KBXY_MASK;

      // Set KBXY (Key, Block, Xin, and Y out) bits in DMA0EN sfr using mask.
      // This enables the DMA. But the encryption/decryption operation
      // won't start until the AES block is enabled.
      DMA0EN  |=  AES0_KBXY_MASK;

      // Enabled AES module to start encryption/decryption operation.
      AES0BCFG |= AES_ENABLE;

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

   // Clear KXBY (Key, Block, XOR, and Y out) bits in DMA0EN sfr using mask.
   DMA0EN &= ~AES0_KBXY_MASK;

   // Clear KBY (Key, Block, and Y out) bits in DMA0INT sfr using mask.
   DMA0INT &= ~AES0_KBXY_MASK;

   return SUCCESS;
}
//-----------------------------------------------------------------------------
// IncrementCounter()
//
// This function is used to increment the 16-byte counter.
//
// This version is hand optimized to produce efficient assembler.
// This results in obtuse C code.
//
//-----------------------------------------------------------------------------
void IncrementCounter (VARIABLE_SEGMENT_POINTER(counter, U8, SEG_XDATA))
{
   U8 i;
   U8 x;

   i = 16;
   counter += 16;                      // point to end of data

   do
   {
      counter--;                          // decrement data pointer
      x = *counter;                       // read xdata using data pointer
      x++;                             // increment value
      *counter = x;                       // move to xram
      if(x) break;                     // break if not zero
   }  while(--i);                      // DJNZ
}
//=============================================================================
// End of file
//=============================================================================

