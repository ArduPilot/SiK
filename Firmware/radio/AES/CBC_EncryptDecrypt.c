//=============================================================================
// CBC_EncryptDecrypt.c
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
#include "CBC_EncryptDecrypt.h"
#include "DMA_defs.h"
#include "AES_defs.h"
//=============================================================================
// API Functions
//=============================================================================
//-----------------------------------------------------------------------------
// CBC_EncryptDecrypt()
//
// parameters:
//    operation       - decryption/encrypt & 128/192/256 options
//    plainText       - xdata pointer to plainText
//    cipherText      - xdata pointer to cipherText
//    initialVector   - xdata pointer to initialVector
//    key             - xdata pointer to encryption or decryption key
//
// returns:
//    status         - 0 for success
//                   - 1 for ERROR - Invalid operation parameter.
//
// description:
//
// This function performs Cipher Block Chaining (CBC) Mode Encryption or
// Decryption of a specified number of 16-byte blocks.
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
// The initial vector is used only for the first block.
//
// This function returns an error if the operation parameter is invalid.
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
// This function is written so that the initialVector, plainText, and
// cipherText can be at an arbitrary location in xdata.
//
// Since the initial vector is in an arbitrary place, this function uses
// one DMA transfer to encrypt/decrypt the first block and a second DMA
// transfer to complete any remaining blocks.
//
// Note that it is also possible to perform a CBC encryption/decryption
// operation in one DMA transfer by strategically locating the initial vector
// immediately before the ciphertext. The design choice was made to favor
// input data flexibility over code efficiency.
//
// For an encryption operation, the output ciphertext can replace the input
// plaintext. Both plaintext and ciphertext pointers can point to the same
// location.
//
// For a decryption operation, the plaintext cannot replace the ciphertext,
// because the previous ciphertext block is required for the next chained
// decryption operation. The plaintext and ciphertext pointers cannot point
// to the same location.
//
//-----------------------------------------------------------------------------
CBC_ENCRYPT_DECRYPT_STATUS
   CBC_EncryptDecrypt (CBC_ENCRYPT_DECRYPT_OPERATION operation,
   VARIABLE_SEGMENT_POINTER(plainText, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(cipherText, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(initialVector, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(key, U8, SEG_XDATA),
   U16 blocks)
{
   // Unions used for compiler independent endianness.
   UU16 length;                        // Length in bytes for all blocks.
   UU16 addr;                          // Union used to access pointer bytes.

   U8 keyLength;                       // Used to calculate key length in bytes.

   // check first for valid operation
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

   // From this point on, blocks is used to count remaining blocks.
   blocks--;

   SFRPAGE = DPPE_PAGE;

   AES0BCFG = 0x00;                      // disable for now
   AES0DCFG = 0x00;                      // disable for now

   // Disable AES0KIN, AES0BIN, AES0XIN, & AES0YOUT channels.
   DMA0EN &= ~AES0_KBXY_MASK;

   // Configure AES key input channel using key pointer.
   // Set length to calculated key length.
   // Set DMA0NMD to disable key wrapping.
   // This is necessary because we want the key DMA channel
   // to stop after the first block.

   DMA0SEL = AES0KIN_CHANNEL;
   DMA0NCF = AES0KIN_PERIPHERAL_REQUEST;
   DMA0NMD = NO_WRAPPING;
   addr.U16 = (U16)(key);
   DMA0NBAL = addr.U8[LSB];
   DMA0NBAH = addr.U8[MSB];
   DMA0NSZH = 0;
   DMA0NSZL = keyLength;
   DMA0NAOL = 0;
   DMA0NAOH = 0;

   // AES block input is plaintext for encryption operation or ciphertext
   // for decryption operation.

   if(operation & ENCRYPTION_MODE)
      addr.U16 = (U16)(plainText);
   else
      addr.U16 = (U16)(cipherText);

   // Configure AES block input channel using corresponding address.
   // Set length to 16 for first block only.
   // Clear DMA0NMD to disable wrapping.

   DMA0SEL = AES0BIN_CHANNEL;
   DMA0NCF = AES0BIN_PERIPHERAL_REQUEST;
   DMA0NMD = NO_WRAPPING;
   DMA0NBAL = addr.U8[LSB];
   DMA0NBAH = addr.U8[MSB];
   DMA0NSZL = 16;                      // one block
   DMA0NSZH = 0;
   DMA0NAOL = 0;
   DMA0NAOH = 0;

   // Configure AES X input channel using initialization vector address.
   // Set length to 16 for first block only.
   // Clear DMA0NMD to disable wrapping.

   DMA0SEL = AES0XIN_CHANNEL;
   DMA0NCF = AES0XIN_PERIPHERAL_REQUEST;
   DMA0NMD = NO_WRAPPING;
   addr.U16 = (U16)(initialVector);
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

   // Configure AES0DCFG depending on ENCRYPTION or DECRYPION operation.
   // For CBC Mode, the XOR operation is on the input for encryption or
   // on the output for decryption.
   if(operation & ENCRYPTION_MODE)
      AES0DCFG = XOR_ON_INPUT;          // XOR on input - CBC Encryption
   else
      AES0DCFG = XOR_ON_OUTPUT;         // XOR on output - CBC Decryption

   // Configure AES0BCFG for encryption or decryption operation according
   // to operation parameter.
   AES0BCFG = operation;

   // Enabled AES module to start encryption/decryption operation.
   AES0BCFG |= AES_ENABLE;               // enable AES

   EIE2 |= 0x20;                 // enable DMA interrupt to terminate Idle mode

   // This do...while loop ensures that the CPU will remain in Idle mode
   // until AES0YOUT DMA channel transfer is complete.
   do
   {
      #ifdef DMA_TRANSFERS_USE_IDLE
      PCON |= 0x01;                    // go to Idle mode
      #endif
   }  while((DMA0INT & AES0YOUT_MASK)==0);

   if(blocks)                          // if blocks remaining
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

      // AESKIN DMA reset address offset and enable wrapping.
      DMA0SEL = AES0KIN_CHANNEL;
      DMA0NMD = WRAPPING;
      DMA0NAOL = 0;
      DMA0NAOH = 0;

      // AESBIN DMA channel change length only.
      // Set length to calculated total plaintext/ciphertext length.
      DMA0SEL = AES0BIN_CHANNEL;
      DMA0NSZL = length.U8[LSB];
      DMA0NSZH = length.U8[MSB];

      // AESXIN DMA channel point to ciphertext for
      // both encryption and decryption.
      // Set length to calculated total plaintext/ciphertext length.
      DMA0SEL = AES0XIN_CHANNEL;
      addr.U16 = (U16)(cipherText);
      DMA0NBAL = addr.U8[LSB];
      DMA0NBAH = addr.U8[MSB];
      DMA0NSZL = length.U8[LSB];
      DMA0NSZH = length.U8[MSB];
      DMA0NAOL = 0;
      DMA0NAOH = 0;

      // AESYOUT DMA channel change length only
      // Set length to calculated total plaintext/ciphertext length.
      DMA0SEL = AES0YOUT_CHANNEL;
      DMA0NSZL = length.U8[LSB];
      DMA0NSZH = length.U8[MSB];

      // Clear KBXY (Key, Block, X in, and Y out) bits in DMA0INT sfr using mask.
      DMA0INT &= ~AES0_KBXY_MASK;

      // Set KBXY (Key, Block, Xin, and Y out) bits in DMA0EN sfr using mask.
      // This enables the DMA. But the encryption/decryption operation
      // won't start until the AES block is enabled.
      DMA0EN  |=  AES0_KBXY_MASK;

      // Enabled AES module to start encryption/decryption operation.
      AES0BCFG |= AES_ENABLE;               // enable AES

      // enable DMA interrupt to terminate Idle mode
      EIE2 |= 0x20;                 // enable DMA interrupt to terminate Idle mode

      // This do...while loop ensures that the CPU will remain in Idle mode
      // until AES0YOUT DMA channel transfer is complete.
      do
      {
         #ifdef DMA_TRANSFERS_USE_IDLE
         PCON |= 0x01;                    // go to Idle mode
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
//=============================================================================
// End of file
//=============================================================================
