// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2013 Joe Turner, All Rights Reserved
// Copyright 2011 Silicon Laboratories, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  o Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  o Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.
//

#include <stdarg.h>
#include "../radio.h"
#include "GenerateDecryptionKey.h"
#include "AES_BlockCipher.h"
#include "CBC_EncryptDecrypt.h"

SEGMENT_VARIABLE (EncryptionKey[32], U8, SEG_XDATA);
SEGMENT_VARIABLE (DecryptionKey[32], U8, SEG_XDATA);
SEGMENT_VARIABLE (PlainText[64], U8, SEG_XDATA);
SEGMENT_VARIABLE (CipherText[64], U8, SEG_XDATA);
SEGMENT_VARIABLE (InitialVector[16], U8, SEG_XDATA);
SEGMENT_VARIABLE (Counter[16], U8, SEG_XDATA);

// The following four will eventually be provided by user and by other means
// They are here at present, to get the encryption/decryption workin
const SEGMENT_VARIABLE (ReferenceEncryptionKey128[16], U8, SEG_CODE) = {0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};
const SEGMENT_VARIABLE (ReferenceDecryptionKey128[16], U8, SEG_CODE) = {0xD0, 0x14, 0xF9, 0xA8, 0xC9, 0xEE, 0x25, 0x89, 0xE1, 0x3F, 0x0C, 0xC8, 0xB6, 0x63, 0x0C, 0xA6};
const SEGMENT_VARIABLE (ReferencePlainText[16], U8, SEG_CODE) =        {0x48, 0x65, 0x6c, 0x6c, 0x6f, 0x20, 0x77, 0x6F, 0x72, 0x6C, 0x64, 0x21, 0x21, 0x21, 0x21, 0x21};
const SEGMENT_VARIABLE (ReferenceInitialVector[16] , U8, SEG_CODE) = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};



void aesCopyInit1(__xdata unsigned char *dest, __code unsigned char *source, uint8_t length) 
{
   while(length--)
   {
      *dest++ = *source++;
   }
}

void aesCopyInit2(__xdata unsigned char *dest, __code unsigned char *source, uint8_t blocks)
{
   uint8_t i;

   while(blocks--)
   {
      for(i=16;i>0;i--)
      {
         *dest++ = *source++;
      }
   }
}

void aesCopyInit3(__xdata unsigned char *dest, __code unsigned char *source)
{
   uint8_t i;

   for(i=16;i>0;i--)
   {
      *dest++ = *source++;
   }
}


bool aes_init()
{
   uint8_t status;

   // Validate 128-bit key inversion
   aesCopyInit1(EncryptionKey, ReferenceEncryptionKey128, 16);
   status = GenerateDecryptionKey(EncryptionKey, DecryptionKey, KEY_SIZE_128_BITS);
   if (status != 0) {
   	return false;
   }
   aesCopyInit3(InitialVector, ReferenceInitialVector);

   return true;
}

/// Pad out the string to encrypt to a multiple of 16 x bytes
///
__xdata unsigned char *aes_pad(__xdata unsigned char *in_str)
{

	volatile uint8_t  pad_length;
	uint8_t i;
	// __xdata unsigned char padstr[1];

	i = 0;
	pad_length = (strlen(in_str)%16);
	if (pad_length == 0) {
		pad_length = 16;	
	} 

	for (i = 0; i < pad_length;i++) {
		memcpy(&in_str[strlen(in_str)], &pad_length, sizeof(pad_length));
	}
	in_str[strlen(in_str)] = '\0';


	return in_str;
}

uint8_t aes_encrypt(__xdata unsigned char *in_str, __xdata unsigned char *out_str)
{
	uint8_t status;
	uint8_t blocks;
	__xdata unsigned char *pt;

	// Make sure we have something to encrypt
	if (strlen(in_str) == 0) {
		return -1;
	}
	
	// We Always Pad the last 16 bytes.
	// If we don't find a pile of 10 10 10....10 in the last block
	// then we know that the last block was incomplete
	// e.g. 01 02 03 05 06 01 02 03 05 06 06 01 was just 15 bytes long...and the
	// last byte is a 01...is padding

	// Copy String into XDATA
	pt = aes_pad(in_str);  // NOTE...later this might be a padded version of in_str

	// Pad out in_str  to X 16-Byte blocks
	blocks = strlen(pt)>>4; // Number of 16-byte blocks....later we'll calc from in_str 


	// Generate Initial Vector
	// -- assuming that the IV changes from time to time --


	// Copy Initial Vecotr in place
	// -- put here...if we need to move from aes_init --

	// Validate 128-bit CBC Mode encryption
	status = CBC_EncryptDecrypt (ENCRYPTION_128_BITS, pt, out_str, InitialVector, EncryptionKey, blocks);

	return status;
}


uint8_t aes_decrypt(__xdata unsigned char *in_str, __xdata unsigned char *out_str)
{
	uint8_t status;
	uint8_t blocks;
	__xdata unsigned char *ct;

	// Make sure we have something to decrypt
	if (strlen(in_str) == 0) {
		return -1;
	}

	// Pad out in_str  to X 16-Byte blocks
	blocks = strlen(in_str)>>4; // Number of 16-byte blocks....later we'll calc from in_str

	// Initialise CipherText
	ct = in_str; // NOTE...later this might be a padded version of in_str

	// Perform 128-bit CBC Mode decryption
	status = CBC_EncryptDecrypt (DECRYPTION_128_BITS, out_str, ct, InitialVector, DecryptionKey, blocks);

	return status;
}


