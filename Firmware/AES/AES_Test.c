//=============================================================================
// Validate_AES-Examples.c
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
// uncomment pragma SRC to generate assemble code
//-----------------------------------------------------------------------------
//#pragma SRC
//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <compiler_defs.h>
#include <Si1020_defs.h>
#include "GenerateDecryptionKey.h"
#include "AES_BlockCipher.h"
#include "CBC_EncryptDecrypt.h"
#include "CTR_EncryptDecrypt.h"
#include "TestVectors.h"
#include <stdio.h>
//=============================================================================
// File global variables
//
// Reference Key and PlainText Copied to global variables located in XRAM.
//
//=============================================================================
SEGMENT_VARIABLE (EncryptionKey[32], U8, SEG_XDATA);
SEGMENT_VARIABLE (DecryptionKey[32], U8, SEG_XDATA);
SEGMENT_VARIABLE (PlainText[64], U8, SEG_XDATA);
SEGMENT_VARIABLE (CipherText[64], U8, SEG_XDATA);
SEGMENT_VARIABLE (InitialVector[16], U8, SEG_XDATA);
SEGMENT_VARIABLE (Counter[16], U8, SEG_XDATA);
//=============================================================================
// Function Prototypes
//=============================================================================
//-----------------------------------------------------------------------------
// Fuctions used to initialize and verify arrays.
//-----------------------------------------------------------------------------
void initKey (VARIABLE_SEGMENT_POINTER(key, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(referenceKey, U8, SEG_CODE), U8 keyLength);

U8 verifyKey (VARIABLE_SEGMENT_POINTER(key, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(referenceKey, U8, SEG_CODE), U8 keyLength);

void initPlainText (VARIABLE_SEGMENT_POINTER(plainText, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(referencePlainText, U8, SEG_CODE), U8 blocks);

U8 verifyPlainText (VARIABLE_SEGMENT_POINTER(plainText, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(referencePlainText, U8, SEG_CODE), U8 blocks);

void initCipherText (VARIABLE_SEGMENT_POINTER(cipherText, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(referenceCipherText, U8, SEG_CODE), U8 blocks);

U8 verifyCipherText (VARIABLE_SEGMENT_POINTER(key, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(referenceCipherText, U8, SEG_CODE), U8 blocks);

void initInitialVector (VARIABLE_SEGMENT_POINTER(initialVector, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(referenceInitialVector, U8, SEG_CODE));

void initCounter (VARIABLE_SEGMENT_POINTER(counter, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(nonce, U8, SEG_CODE));
//-----------------------------------------------------------------------------
// Interrupt proto (for SDCC compatibility)
//-----------------------------------------------------------------------------
INTERRUPT_PROTO(DMA_ISR, INTERRUPT_DMA0);
//=============================================================================
// main
//=============================================================================
//-----------------------------------------------------------------------------
// main()
//
// Parameters: none
// Returns: none
//
// Description:
//
// This main module program serves as a code example for the AES reusable code
// modules and also validates all AES operations and AES cipher block modes.
//
// This reusable AES code modules are:
//
// GenerateDecryptionKey.c
// AES_BlockCipher.c
// CBC_EncryptDecrypt.c
// CTR_EncryptDecrypt.c
//
// The test code in this module will test key inversion for all three key sizes.
// It will also test both encrypytion and decryption for all three key sizes.
//
// The test vectors are located in the test Vectors.c module. The test
// vectors are taken directly from from NIST SP800-38A Appendix F.
//
// Steps to use:
// 1) Compile and download the code.
// 2) Run code.
//
// This simple test code will hang just after the respective test
// if any test fails. If all tests pass, it will run to the end of the
// program.
//
//-----------------------------------------------------------------------------
void  Port_Init (void);
void  SYSCLK_Init (void);
//void  SPI1_Init (void);
void  UART0_Init(void);

void main (void)
{
   U8 status;

   PCA0MD  &= ~0x40;                   // disable watchdog timer

	Port_Init ();
	SYSCLK_Init ();
	UART0_Init();
//	SPI1_Init ();
	
   EA = 1;                             // enable global interrupts

   // Validate 128-bit key inversion
   initKey(EncryptionKey, ReferenceEncryptionKey128, 16);
	printf("128bit - ");
   status = GenerateDecryptionKey(EncryptionKey, DecryptionKey, KEY_SIZE_128_BITS);
	printf("x -");
   while(status);                      // code will hang here on error.
	printf("ok\n");
   status = verifyKey(DecryptionKey, ReferenceDecryptionKey128, 16);

   while(status);                      // code will hang here on error.

   // Validate 192-bit key inversion
   initKey(EncryptionKey, ReferenceEncryptionKey192, 24);

   status = GenerateDecryptionKey(EncryptionKey, DecryptionKey, KEY_SIZE_192_BITS);

   while(status);                      // code will hang here on error.

   status = verifyKey(DecryptionKey, ReferenceDecryptionKey192, 24);

   while(status);                      // code will hang here on error.

   // Validate 256-bit key inversion
   initKey(EncryptionKey, ReferenceEncryptionKey256, 32);

   status = GenerateDecryptionKey (EncryptionKey, DecryptionKey, KEY_SIZE_256_BITS);

   while(status);                      // code will hang here on error.

   status = verifyKey(DecryptionKey, ReferenceDecryptionKey256, 32);

   while(status);                      // code will hang here on error.

   // Validate 128-bit AES block cipher encryption
   initPlainText(PlainText, ReferencePlainText, 4);

   initKey(EncryptionKey, ReferenceEncryptionKey128, 16);

   status = AES_BlockCipher (ENCRYPTION_128_BITS, PlainText, CipherText, EncryptionKey, 4);

   while(status);                      // code will hang here on error.

   status = verifyCipherText(CipherText, ReferenceCipherText_ECB_128, 4);

   while(status);                      // code will hang here on error.

   // Validate 192-bit AES block cipher encryption
   initPlainText(PlainText, ReferencePlainText, 4);

   initKey(EncryptionKey, ReferenceEncryptionKey192, 24);

   status = AES_BlockCipher (ENCRYPTION_192_BITS, PlainText, CipherText, EncryptionKey, 4);

   while(status);                      // code will hang here on error.

   status = verifyCipherText(CipherText, ReferenceCipherText_ECB_192, 4);

   while(status);                      // code will hang here on error.

   // Validate 256-bit AES block cipher encryption
   initPlainText(PlainText, ReferencePlainText, 4);

   initKey(EncryptionKey, ReferenceEncryptionKey256, 32);

   status = AES_BlockCipher (ENCRYPTION_256_BITS, PlainText, CipherText, EncryptionKey, 4);

   while(status);                      // code will hang here on error.

   status = verifyCipherText(CipherText, ReferenceCipherText_ECB_256, 4);

   while(status);                      // code will hang here on error.

   // Validate 128-bit AES block cipher decryption
   initCipherText(CipherText, ReferenceCipherText_ECB_128, 4);

   initKey(DecryptionKey, ReferenceDecryptionKey128, 16);

   status = AES_BlockCipher (DECRYPTION_128_BITS, PlainText, CipherText, DecryptionKey, 4);

   while(status);                      // code will hang here on error.

   status = verifyPlainText(PlainText, ReferencePlainText, 4);

   while(status);                      // code will hang here on error.

   // Validate 192-bit AES block cipher decryption
   initCipherText(CipherText, ReferenceCipherText_ECB_192, 4);

   initKey(DecryptionKey, ReferenceDecryptionKey192, 24);

   status = AES_BlockCipher (DECRYPTION_192_BITS, PlainText, CipherText, DecryptionKey, 4);

   while(status);                      // code will hang here on error.

   status = verifyPlainText(PlainText, ReferencePlainText, 4);

   while(status);                      // code will hang here on error.

   // Validate 256-bit AES block cipher decryption
   initCipherText(CipherText, ReferenceCipherText_ECB_256, 4);

   initKey(DecryptionKey, ReferenceDecryptionKey256, 32);

   status = AES_BlockCipher (DECRYPTION_256_BITS, PlainText, CipherText, DecryptionKey, 4);

   while(status);                      // code will hang here on error.

   status = verifyPlainText(PlainText, ReferencePlainText, 4);

   while(status);                      // code will hang here on error.

   // Validate 128-bit CBC Mode encryption
   initPlainText(PlainText, ReferencePlainText, 4);

   initInitialVector(InitialVector, ReferenceInitialVector);

   initKey(EncryptionKey, ReferenceEncryptionKey128, 16);

   status = CBC_EncryptDecrypt (ENCRYPTION_128_BITS, PlainText, CipherText, InitialVector, EncryptionKey, 4);

   while(status);                      // code will hang here on error.

   status = verifyCipherText(CipherText, ReferenceCipherText_CBC_128, 4);

   while(status);                      // code will hang here on error.

   // Validate 192-bit CBC Mode encryption
   initPlainText(PlainText, ReferencePlainText, 4);

   initInitialVector(InitialVector, ReferenceInitialVector);

   initKey(EncryptionKey, ReferenceEncryptionKey192, 24);

   status = CBC_EncryptDecrypt (ENCRYPTION_192_BITS, PlainText, CipherText, InitialVector, EncryptionKey, 4);

   while(status);                      // code will hang here on error.

   status = verifyCipherText(CipherText, ReferenceCipherText_CBC_192, 4);

   while(status);                      // code will hang here on error.

   // Validate 256-bit CBC Mode encryption
   initPlainText(PlainText, ReferencePlainText, 4);

   initInitialVector(InitialVector, ReferenceInitialVector);

   initKey(EncryptionKey, ReferenceEncryptionKey256, 32);

   status = CBC_EncryptDecrypt (ENCRYPTION_256_BITS, PlainText, CipherText, InitialVector, EncryptionKey, 4);

   while(status);                      // code will hang here on error.

   status = verifyCipherText(CipherText, ReferenceCipherText_CBC_256, 4);

   while(status);                      // code will hang here on error.

   // Validate 128-bit CBC Mode decryption
   initCipherText(CipherText, ReferenceCipherText_CBC_128, 4);

   initInitialVector(InitialVector, ReferenceInitialVector);

   initKey(DecryptionKey, ReferenceDecryptionKey128, 16);

   status = CBC_EncryptDecrypt (DECRYPTION_128_BITS, PlainText, CipherText, InitialVector, DecryptionKey, 4);

   while(status);                      // code will hang here on error.

   status = verifyPlainText(PlainText, ReferencePlainText, 4);

   while(status);                      // code will hang here on error.

   // Validate 192-bit CBC Mode decryption
   initCipherText(CipherText, ReferenceCipherText_CBC_192, 4);

   initInitialVector(InitialVector, ReferenceInitialVector);

   initKey(DecryptionKey, ReferenceDecryptionKey192, 24);

   status = CBC_EncryptDecrypt (DECRYPTION_192_BITS, PlainText, CipherText, InitialVector, DecryptionKey, 4);

   while(status);                      // code will hang here on error.

   status = verifyPlainText(PlainText, ReferencePlainText, 4);

   while(status);                      // code will hang here on error.

   // Validate 256-bit CBC Mode decryption
   initCipherText(CipherText, ReferenceCipherText_CBC_256, 4);

   initInitialVector(InitialVector, ReferenceInitialVector);

   initKey(DecryptionKey, ReferenceDecryptionKey256, 32);

   status = CBC_EncryptDecrypt (DECRYPTION_256_BITS, PlainText, CipherText, InitialVector, DecryptionKey, 4);

   while(status);                      // code will hang here on error.

   status = verifyPlainText(PlainText, ReferencePlainText, 4);

   while(status);                      // code will hang here on error.

   // Validate 128-bit CTR Mode encryption
   initPlainText(PlainText, ReferencePlainText, 4);

   initCounter(Counter, Nonce);

   initKey(EncryptionKey, ReferenceEncryptionKey128, 16);

   status = CTR_EncryptDecrypt (ENCRYPTION_128_BITS, PlainText, CipherText, Counter, EncryptionKey, 4);

   while(status);                      // code will hang here on error.

   status = verifyCipherText(CipherText, ReferenceCipherText_CTR_128, 4);

   while(status);                      // code will hang here on error.

   // Validate 192-bit CTR Mode encryption
   initPlainText(PlainText, ReferencePlainText, 4);

   initCounter(Counter, Nonce);

   initKey(EncryptionKey, ReferenceEncryptionKey192, 24);

   status = CTR_EncryptDecrypt (ENCRYPTION_192_BITS, PlainText, CipherText, Counter, EncryptionKey, 4);

   while(status);                      // code will hang here on error.

   status = verifyCipherText(CipherText, ReferenceCipherText_CTR_192, 4);

   while(status);                      // code will hang here on error.

   // Validate 256-bit CTR Mode encryption
   initPlainText(PlainText, ReferencePlainText, 4);

   initCounter(Counter, Nonce);

   initKey(EncryptionKey, ReferenceEncryptionKey256, 32);

   status = CTR_EncryptDecrypt (ENCRYPTION_256_BITS, PlainText, CipherText, Counter, EncryptionKey, 4);

   while(status);                      // code will hang here on error.

   status = verifyCipherText(CipherText, ReferenceCipherText_CTR_256, 4);

   while(status);                      // code will hang here on error.

   // Validate 128-bit CTR Mode decryption
   initCipherText(CipherText, ReferenceCipherText_CTR_128, 4);

   initCounter(Counter, Nonce);

   initKey(EncryptionKey, ReferenceEncryptionKey128, 16);

   status = CTR_EncryptDecrypt (DECRYPTION_128_BITS, PlainText, CipherText, Counter, EncryptionKey, 4);

   while(status);                      // code will hang here on error.

   status = verifyPlainText(PlainText, ReferencePlainText, 4);

   while(status);                      // code will hang here on error.

   // Validate 192-bit CTR Mode decryption
   initCipherText(CipherText, ReferenceCipherText_CTR_192, 4);

   initCounter(Counter, Nonce);

   initKey(EncryptionKey, ReferenceEncryptionKey192, 24);

   status = CTR_EncryptDecrypt (DECRYPTION_192_BITS, PlainText, CipherText, Counter, EncryptionKey, 4);

   while(status);                      // code will hang here on error.

   status = verifyPlainText(PlainText, ReferencePlainText, 4);

   while(status);                      // code will hang here on error.

   // Validate 256-bit CTR Mode decryption
   initCipherText(CipherText, ReferenceCipherText_CTR_256, 4);

   initCounter(Counter, Nonce);

   initKey(EncryptionKey, ReferenceEncryptionKey256, 32);

   status = CTR_EncryptDecrypt (DECRYPTION_256_BITS, PlainText, CipherText, Counter, EncryptionKey, 4);

   while(status);                      // code will hang here on error.

   status = verifyPlainText(PlainText, ReferencePlainText, 4);

   while(status);                      // code will hang here on error.

	printf("Finished...\n");
   while(1);                           // code will hang here on success.
}
//-----------------------------------------------------------------------------
// DMA_ISR
// description:
//
// This ISR is needed to support the DMA Idle mode wake up, which is used
// in the AES functions. Bit 5 of EIE2 should be enabled before going into
// idle mode. This ISR will disable further interrupts. EA must also be
// enabled.
//
//-----------------------------------------------------------------------------
INTERRUPT(DMA_ISR, INTERRUPT_DMA0)
{
  EIE2 &= ~0x20;                       // disable further interrupts
}
//-----------------------------------------------------------------------------
// initKey()
//
// parameters:
//   key          - xdata pointer to key
//   referenceKey - code pointer to reference key
//   keyLength    - key length in bytes
//
// returns:
//
// description:
//
// Copies reference key in Flash to key in xdata RAM.
// The key must be copied to xdata for use by the DMA.
//
//-----------------------------------------------------------------------------
void initKey (VARIABLE_SEGMENT_POINTER(key, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(referenceKey, U8, SEG_CODE),
   U8 keyLength)
{
   while(keyLength--)
   {
      *key++ = *referenceKey++;
   }
}
//-----------------------------------------------------------------------------
// verifyKey()
//
// parameters:
//   key          - xdata pointer to key
//   referenceKey - code pointer to reference key
//   keyLength    - key length in bytes
//
// returns:
//    badKeyBytes - the number of incorrect key bytes
//
// description:
//
// Compares key in RAM to reference key stored in Flash.
// The return value should be zero if the keys are identical.
//
//-----------------------------------------------------------------------------
U8 verifyKey (VARIABLE_SEGMENT_POINTER(key, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(referenceKey, U8, SEG_CODE),
   U8 keyLength)
{
   U8 badKeyBytes;

   badKeyBytes=0;

   while(keyLength--)
   {
         if(*key++ != *referenceKey++)
            badKeyBytes++;
   }

   return badKeyBytes;
}
//-----------------------------------------------------------------------------
// initPlainText()
//
// parameters:
//   plainText          - xdata pointer to key
//   referencePlainText - code pointer to reference key
//   blocks             - the number of 16-byte blocks to copy
//
// returns:
//
// description:
//
// Copies reference PlainText in Flash to plaintText in xdata RAM.
// The plainText must be copied to xdata for use by the DMA.
//
//-----------------------------------------------------------------------------
void initPlainText (VARIABLE_SEGMENT_POINTER(plainText, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(referencePlainText, U8, SEG_CODE),
   U8 blocks)
{
   U8 i;

   while(blocks--)
   {
      for(i=16;i>0;i--)
      {
         *plainText++ = *referencePlainText++;
      }
   }
}
//-----------------------------------------------------------------------------
// verifyPlainText()
//
// parameters:
//   plainText          - xdata pointer to plainText
//   referencePlainText - code pointer to reference PlainText
//   blocks             - the number of 16-byte blocks to copy
//
// returns:
//    badPlainTextBytes - the number of incorrect PlainText bytes
//
// description:
//
// Compares plainText in RAM to reference PlainText stored in Flash.
// The return value should be zero if the PlainTexts are identical.
//
//
//-----------------------------------------------------------------------------
U8 verifyPlainText (VARIABLE_SEGMENT_POINTER(plainText, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(referencePlainText, U8, SEG_CODE),
   U8 blocks)
{
   U8 badPlainTextBytes;
   U8 i;

   badPlainTextBytes=0;

   while(blocks--)
   {
      for(i=16;i>0;i--)
      {
         if(*plainText++ != *referencePlainText++)
            badPlainTextBytes++;
      }
   }

   return badPlainTextBytes;
}
//-----------------------------------------------------------------------------
// initCipherText()
//
// parameters:
//   cipherText          - xdata pointer to cipherText
//   referenceCipherText - code pointer to CipherText
//   blocks              - the number of 16-byte blocks to copy
//
// returns:
//
// description:
//
// Copies reference CipherText in Flash to cipherText in xdata RAM.
// The cipherText must be copied to xdata for use by the DMA.
//
//-----------------------------------------------------------------------------
void initCipherText (VARIABLE_SEGMENT_POINTER(cipherText, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(referenceCipherText, U8, SEG_CODE),
   U8 blocks)
{
   U8 i;

   while(blocks--)
   {
      for(i=16;i>0;i--)
      {
         *cipherText++ = *referenceCipherText++;
      }
   }
}
//-----------------------------------------------------------------------------
// verifyCipherText()
//
// parameters:
//   cipherText          - xdata pointer to cipherText
//   referenceCipherText - code pointer to reference CipherText
//   blocks             - the number of 16-byte blocks to copy
//
// returns:
//    badCipherTextBytes - the number of incorrect CipherText bytes
//
// description:
//
// Compares cipherText in RAM to reference CipherText stored in Flash.
// The return value should be zero if the CipherTexts are identical.
//
//
//-----------------------------------------------------------------------------
U8 verifyCipherText (VARIABLE_SEGMENT_POINTER(cipherText, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(referenceCipherText, U8, SEG_CODE),
   U8 blocks)
{
   U8 badCipherTextBytes;
   U8 i;

   badCipherTextBytes=0;

   while(blocks--)
   {
      for(i=16;i>0;i--)
      {
         if(*cipherText++ != *referenceCipherText++)
            badCipherTextBytes++;
      }
   }

   return badCipherTextBytes;
}

//-----------------------------------------------------------------------------
// initInitialVector()
//
// parameters:
//   initialVector          - xdata pointer to initialVector
//   referenceInitialVector - code pointer to referenceInitialVector
//
// returns:
//
// description:
//
// Copies reference InitialVector in Flash to InitialVector in xdata RAM.
// The InitialVector must be copied to xdata for use by the DMA.
//
//-----------------------------------------------------------------------------
void initInitialVector (VARIABLE_SEGMENT_POINTER(initialVector, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(referenceInitialVector, U8, SEG_CODE))
{
   U8 i;

   for(i=16;i>0;i--)
   {
      *initialVector++ = *referenceInitialVector++;
   }
}
//-----------------------------------------------------------------------------
// initCounter()
//
// parameters:
//   counter      - xdata pointer to counter
//   nonce        - code pointer to nonce
//
// returns:
//
// description:
//
// Copies nonce in Flash to counter in xdata RAM. Nonce is a crpytographic
// term. The nonce is a number used once. It is used for the initial value of
// the counter. The counter must be stored to xdata for use by the DMA.
//
//-----------------------------------------------------------------------------
void initCounter (VARIABLE_SEGMENT_POINTER(counter, U8, SEG_XDATA),
   VARIABLE_SEGMENT_POINTER(nonce, U8, SEG_CODE))
{
   U8 i;

   for(i=16;i>0;i--)
   {
      *counter++ = *nonce++;
   }
}



//=============================================================================
//
// Init Functions
//
//=============================================================================
//-----------------------------------------------------------------------------
// Port_Init ()
//-----------------------------------------------------------------------------
void Port_Init (void)
{
	U8 restoreSFRPAGE;
	restoreSFRPAGE = SFRPAGE;
	
	// Init UART
	SFRPAGE  = LEGACY_PAGE;
	XBR0    |= 0x01;                    // enable UART
	P0MDOUT |= 0x10;                    // TX push-pull
	SFRPAGE  = CONFIG_PAGE;
	P0DRV    = 0x10;                    // TX high-current mode
	
	// Init SPI
	SFRPAGE   = LEGACY_PAGE;
	XBR1    |= 0x40;                    // Enable SPI1 (3 wire mode)
	P2MDOUT |= 0x0D;                    // SCK1, MOSI1, & NSS1,push-pull
	SFRPAGE  = CONFIG_PAGE;
	P2DRV    = 0x0D;                    // MOSI1, SCK1, NSS1, high-drive mode
	
	P2       = 0xFF;                    // P2 bug fix for SDCC and Raisonance
	
	XBR2    |= 0x40;                    // enable Crossbar
	
	IT01CF   = 0x06;                    // INT0 is assigned to P1.6
	IE0      = 0;
	IT0      = 0;
	EX0      = 1;
	
	SFRPAGE	 =  CONFIG_PAGE;
	P3MDOUT |= 0xC0;		/* Led ports */
	P3DRV   |= 0xC0;		/* Led ports */
	SFRPAGE	 = LEGACY_PAGE;
	
	P3 = 0x80;
	
	SFRPAGE  = restoreSFRPAGE;
}

//-----------------------------------------------------------------------------
// SYSCLK_Init ()
//-----------------------------------------------------------------------------
void SYSCLK_Init (void)
{
	U8 restoreSFRPAGE;
	restoreSFRPAGE = SFRPAGE;
	
	SFRPAGE   = LEGACY_PAGE;
	
	OSCICN |= 0x80;                     // Enable the precision internal osc.
	
	CLKSEL = 0x00;                      // use 24,5 MHz clock
	FLSCL = 0x40; // Bypass one-shot
	
	SFRPAGE  = restoreSFRPAGE;
}

////-----------------------------------------------------------------------------
//// SPI1_Init ()
////-----------------------------------------------------------------------------
//void SPI1_Init()
//{
//	U8 restoreSFRPAGE;
//	restoreSFRPAGE = SFRPAGE;
//	
//	SFRPAGE   = SPI1_PAGE;
//	
//	SPI1CFG  |= 0x40;                   // master mode
//	SPI1CN    = 0x00;                   // 3 wire master mode
//	SPI1CKR   = 0x00;                   // Use SYSWCLK/2r
//	SPI1CN   |= 0x01;                   // enable SPI
//	NSS1      = 1;                      // set NSS high
//	
//	SFRPAGE  = restoreSFRPAGE;
//}

//-----------------------------------------------------------------------------
// UART0_Init ()
//-----------------------------------------------------------------------------
void   UART0_Init (void)
{
	U8 restoreSFRPAGE;
	restoreSFRPAGE = SFRPAGE;
	
	SCON0  = 0x10;                      // SCON0: 8-bit variable bit rate, no receiver
	CKCON |= 0x08;                      // T1 uses SYSCLK
	TH1    = 0x2b;//T1_RELOAD;                 // reload value calculated from BAUD
	TL1   = 0x0; //T1_RELOAD;                 // also load into TL1
	TMOD  &= ~0xF0;                     // clear T1 bits in TMOD
	TMOD  |=  0x20;                     // set TMOD for 8 bit reload
	TR1    = 1;                         // START Timer1
	TI0    = 1;                         // Transciever ready
	
	SFRPAGE  = restoreSFRPAGE;
}

//-----------------------------------------------------------------------------
// putchar
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : character to send to UART
//
// This function outputs a character to the UART.
// Used for SDCC build only.
//-----------------------------------------------------------------------------
#ifdef SDCC
void putchar (char c)
{
	while (!TI0);
	TI0 = 0;
	SBUF0 = c;
}
#endif
//=============================================================================
// End of file
//=============================================================================
