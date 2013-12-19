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
// #include <string.h>
// #include <stdio.h>
#include <stdlib.h>

SEGMENT_VARIABLE (EncryptionKey[32], U8, SEG_XDATA);
SEGMENT_VARIABLE (DecryptionKey[32], U8, SEG_XDATA);
SEGMENT_VARIABLE (InitialVector[16], U8, SEG_XDATA);

// The following four will eventually be provided by user and by other means
// They are here at present, to get the encryption/decryption working
const SEGMENT_VARIABLE (ReferenceInitialVector[16] , U8, SEG_CODE) = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

// Hard-coded key for now...will provide via a parameter later on
__pdata unsigned char aes_key_string[] = "aabbccdd112233445566778899121311";


// Default key to use if none provided, or invalid one provided
__pdata unsigned char aes_default_key_string[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};


// Used to convert Hex # into Integers
//
uint8_t read_hex_nibble(const uint8_t c) __reentrant __nonbanked
{
    if ((c >='0') && (c <= '9'))
    {
        return c - '0';
    }
    else if ((c >='A') && (c <= 'F'))
    {
        return c - 'A' + 10;
    }
    else if ((c >='a') && (c <= 'f'))
    {
        return c - 'a' + 10;
    }
    else
    {
        // printf("[%u] read_hex_nibble: Error char not in supported range",nodeId);
        return 0;
    }
}

// Generate EncryptionKey from aes key string provided (or default one if one provided is invalid)
//
void aes_loadkey()
{
	__pdata unsigned char *aes_key;
	uint8_t i, key_length, num;

	key_length = 16;

	// Make sure that the string is valid. If not, use default key.
	if (aes_key_string[strspn(aes_key_string, "0123456789abcdefABCDEF")] != 0)
	{
		aes_key = &aes_default_key_string[0];
	} else {
		aes_key = &aes_key_string[0];
	}

	for (i=0;i<key_length;i++) {
		num = read_hex_nibble(aes_key[2 * i])<<4;
		num += read_hex_nibble(aes_key[2 * i + 1]);
		EncryptionKey[i] = num;
	}
}


// Perform Copying of data, to help prepare for encryption
void aesCopyInit2(__xdata unsigned char *dest, __code unsigned char *source)
{
   uint8_t i;

   for(i=16;i>0;i--)
   {
      *dest++ = *source++;
   }
}


// Initialse variables ready for AES
//
// returns true if successful, or false if not
bool aes_init()
{
   uint8_t status;
//   uint8_t i;  // DEBUGGING

   // Load Key
   aes_loadkey();

// DEBUGGING
//   printf("ENC Key:");
//         for (i=0; i<16; i++) {
//                 printf("%u ",EncryptionKey[i]);
//         }
//         printf("\n");

   // Generate Decryption Key
   status = GenerateDecryptionKey(EncryptionKey, DecryptionKey, KEY_SIZE_128_BITS);
   if (status != 0) {
   	return false;
   }

   // Initialise IV
   aesCopyInit2(InitialVector, ReferenceInitialVector);

   return true;
}

// Pad out the string to encrypt to a multiple of 16 x bytes
//
// returns the padded string
__xdata unsigned char *aes_pad(__xdata unsigned char *in_str, uint8_t len)
{
	volatile uint8_t  pad_length;
	uint8_t i;

	pad_length = 16 - (len%16);

	for (i = 0; i < pad_length;i++) {
		memcpy(&in_str[len+i], &pad_length, sizeof(pad_length));
	}

	return in_str;
}

// encrypt the data pointed to by in_str with length len
//
// returns a number indicate outcome. 0 is success
uint8_t aes_encrypt(__xdata unsigned char *in_str, uint8_t in_len, __xdata unsigned char *out_str,
			uint8_t *out_len)
{
	uint8_t status;
	uint8_t blocks;
	__xdata unsigned char *pt;
//        uint8_t  i;   // FOR DEBUGGING

	// Make sure we have something to encrypt
	if (in_len == 0) {
		return -1;
	}
	
	// We Always Pad the last 16 bytes.
	// If we don't find a pile of 10 10 10....10 in the last block
	// then we know that the last block was incomplete
	// e.g. 01 02 03 05 06 01 02 03 05 06 06 01 was just 15 bytes long...and the
	// last byte is a 01...is padding

	// Copy String into XDATA
	pt = aes_pad(in_str, in_len);  // NOTE...later this might be a padded version of in_str

	// Calculate # of blocks we need to encrypt
	blocks = 1 + (in_len>>4); // Number of 16-byte blocks to encrypt


// DEBUGGING
//   printf("PRE ENC %u:", blocks);
//         for (i=0; i<(16 * blocks); i++) {
//                 printf("%d ",pt[i]);
//         }
//         printf("\n");


	// Validate 128-bit CBC Mode encryption
	status = CBC_EncryptDecrypt (ENCRYPTION_128_BITS, pt, out_str, InitialVector, EncryptionKey, blocks);
	// Set size of encrypted cipher in bytes
	*out_len = 16 * blocks;

	return status;
}



// decrypt the data pointed to by in_str with length in_len
//
// returns a number indicate outcome. 0 is success
uint8_t aes_decrypt(__xdata unsigned char *in_str, uint8_t in_len, __xdata unsigned char *out_str,
			uint8_t *out_len)
{
	uint8_t status;
	uint8_t blocks;
	__xdata unsigned char *ct;
//        uint8_t  i;   // FOR DEBUGGING

	// Make sure we have something to decrypt
	if (in_len == 0) {
		return -1;
	}

	// Pad out in_str  to X 16-Byte blocks
	blocks = in_len>>4; 

	// Initialise CipherText
	ct = in_str; // NOTE...later this might be a padded version of in_str

// DEBUGGING
//   printf("PRE DEC %u:", blocks);
//         for (i=0; i<strlen(ct); i++) {
//                 printf("%d ",ct[i]);
//         }
//         printf("\n");


	// Perform 128-bit CBC Mode decryption
	status = CBC_EncryptDecrypt (DECRYPTION_128_BITS, out_str, ct, InitialVector, DecryptionKey, blocks);

	// Set size of decrypted ciper text, taking into account the padding
	*out_len = in_len - out_str[16 * blocks - 1];

	return status;
}


