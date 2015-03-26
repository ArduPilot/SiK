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
#include "CTR_EncryptDecrypt.h"
#include <stdlib.h>

/* SEGMENT_VARIABLE (EncryptionKey[32], U8, SEG_XDATA); */
__xdata unsigned char *EncryptionKey;
SEGMENT_VARIABLE (DecryptionKey[32], U8, SEG_XDATA);
SEGMENT_VARIABLE (InitialVector[16], U8, SEG_XDATA);
SEGMENT_VARIABLE (Counter[16], U8, SEG_XDATA);

// The following four will eventually be provided by user and by other means
// They are here at present, to get the encryption/decryption working
const SEGMENT_VARIABLE (Nonce[16], U8, SEG_CODE) = {0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff};
const SEGMENT_VARIABLE (ReferenceInitialVector[16] , U8, SEG_CODE) = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};


/* Helper definitions  */
// First nibble = code for # of bits - 1 = 128, 2 = 192, 3 = 256 
#define	BITS(_l)	(_l)&0xf
// Second nibble = crypto 0 = CBC, 1 = CTR
#define	CRYPTO(_l)	(_l>>4)&0xf

// Variables
uint8_t encryption_level;


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

// Indicate if encryption subsystem initialised and ready.
//
// returns a bool
uint8_t aes_get_encryption_level()
{
        return encryption_level;
}


// Set status of initialistion of aes encryption
//
void aes_set_encryption_level(uint8_t encryption)
{
        encryption_level = encryption;
}



// Generate EncryptionKey from aes key string provided (or default one if one provided is invalid)
//
void aes_initkey()
{
	EncryptionKey = param_get_encryption_key();
}


// Perform Copying of data, to help prepare for encryption
void aesCopyInit2(__xdata unsigned char *dest, __code unsigned char *source)
{
	uint8_t i;

	for(i=16;i>0;i--) {
		*dest++ = *source++;
	}
}


// Initialse variables ready for AES
//
// returns true if successful, or false if not
bool aes_init(uint8_t encryption_level)
{
	uint8_t crypto_type;
	uint8_t status;
	int8_t key_size_code;
	uint8_t bits;

	aes_set_encryption_level(0);  // Initially set to zero - no encryption

	// If encryption level  (first nibble == encryption bits) is zero...no encryption
	bits = BITS(encryption_level);
	if (bits == 0) return true;

	// From the encryption level, determine code for # of bits for AES functions
	key_size_code = bits - 1;

	// Load Encryption Key
	aes_initkey();

	// Generate Decryption Key (Only required by CBC)
	status = GenerateDecryptionKey(EncryptionKey, DecryptionKey, key_size_code);
	if (status != 0) return false;

	// Get Crypto algo type 
	crypto_type = CRYPTO(encryption_level);

	// Based on the crypto algoithm chosen, determine what other step needs to be 
	// done and do them
	switch(crypto_type)
	{
		case 0:
			// Initialise IV
			aesCopyInit2(InitialVector, ReferenceInitialVector);
			break;
		case 1:
			// Nothing to do. We init "Counter" everytime we do encrypt/decrypt
			break;
		default:
			// Initialise IV
			aesCopyInit2(InitialVector, ReferenceInitialVector);
	}

	aes_set_encryption_level(encryption_level);  // If up to here, must have been successful

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
	uint8_t encryption;
	uint8_t crypto_type;
	int8_t key_size_code;
	uint8_t status;
	uint8_t blocks;
	__xdata unsigned char *pt;

	// Make sure we have something to encrypt
	if (in_len == 0) return 0;

	// Get Encryption Level
	encryption = aes_get_encryption_level();

	// From the encryption level, determine code for # of bits for AES functions
	// ENCRYPTION_128_BITS,                // 0x04
	// ENCRYPTION_192_BITS,                // 0x05
	// ENCRYPTION_256_BITS,                // 0x06
	switch (BITS(encryption))
	{
		case 1:
			key_size_code = ENCRYPTION_128_BITS;
			break;	
		case 2:
			key_size_code = ENCRYPTION_192_BITS;
			break;	
		case 3:
			key_size_code = ENCRYPTION_256_BITS;
			break;	
		default:
			key_size_code = ENCRYPTION_128_BITS;
	}


	// Get crypto type...
	// 0 - CBC
	// 1 - CTR
	crypto_type = CRYPTO(encryption);
	
	// We always pad the blocks with up to max 16 bytes.
	// If we don't find a pile of 10 10 10....10 in the last block
	// then we know that the last block was incomplete
	// e.g. 01 02 03 05 06 01 02 03 05 06 06 01 was just 15 bytes long...and the
	// last byte is a 01...is padding

	// Copy String into XDATA
	pt = aes_pad(in_str, in_len); 

	// Calculate # of blocks we need to encrypt
	blocks = 1 + (in_len>>4); // Number of 16-byte blocks to encrypt

	// Based on crypto_type, perform the encryption
	switch(crypto_type)
	{
		case 0:
			// Validate CBC Mode encryption
			status = CBC_EncryptDecrypt (key_size_code, pt, out_str, InitialVector, EncryptionKey, blocks);
			break;
		case 1:
			// Perform CTR Mode decryption
			aesCopyInit2(Counter, Nonce);
			status = CTR_EncryptDecrypt (key_size_code, pt, out_str, Counter, EncryptionKey, blocks);
			break;
		default:
			// Validate CBC Mode encryption
			status = CBC_EncryptDecrypt (key_size_code, pt, out_str, InitialVector, EncryptionKey, blocks);
	}

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
	uint8_t encryption;
	uint8_t crypto_type;
	int8_t key_size_code;
	uint8_t status;
	uint8_t blocks;
	__xdata unsigned char *ct;

	// Make sure we have something to decrypt
	if (in_len == 0) return 0;

	// Get Encryption Level
	encryption = aes_get_encryption_level();

	// From the encryption level, determine code for # of bits for AES functions
	// DECRYPTION_128_BITS = 0,            // 0x00
	// DECRYPTION_192_BITS,                // 0x01
	// DECRYPTION_256_BITS,                // 0x02
	switch (BITS(encryption))
	{
		case 1:
			key_size_code = DECRYPTION_128_BITS;
			break;	
		case 2:
			key_size_code = DECRYPTION_192_BITS;
			break;	
		case 3:
			key_size_code = DECRYPTION_256_BITS;
			break;	
		default:
			key_size_code = DECRYPTION_128_BITS;
	}


	// Get crypto type...
	// 0 - CBC
	// 1 - CTR
	crypto_type = CRYPTO(encryption);

	// Calculate # of 16-byte blocks
	blocks = in_len>>4; 

	// Initialise CipherText
	ct = in_str; 

	// Based on crypto_type, perform the decryption
	switch(crypto_type)
	{
		case 0:
			// Perform CBC Mode decryption
			status = CBC_EncryptDecrypt (key_size_code, out_str, ct, InitialVector, DecryptionKey, blocks);
			break;
		case 1:
			// Perform CTR Mode decryption  (For CTR - DecryptionKey = EncryptionKey)
			aesCopyInit2(Counter, Nonce);
			status = CTR_EncryptDecrypt (key_size_code, out_str, ct, Counter, EncryptionKey, blocks);
			break;
		default:
			// Perform CBC Mode decryption
			status = CBC_EncryptDecrypt (key_size_code, out_str, ct, InitialVector, DecryptionKey, blocks);
	}
			

	// Set size of decrypted ciper text, taking into account the padding
	*out_len = in_len - out_str[16 * blocks - 1];

	return status;
}
