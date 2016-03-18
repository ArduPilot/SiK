/*
 * aes.c
 *
 *  Created on: 18/01/2015
 *      Author: kentm
 */

#include <stdarg.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "em_cmu.h"
#include "aes_defs.H"
#include "aes_ctr_128.h"
#include "parameters.h"
#include "crc.h"
//#include "aes_cbc_128.h"

// ******************** defines and typedefs *************************
#define MAX_ENCRYPT_PACKET_LENGTH 256
// ******************** local variables ******************************
static uint8_t *EncryptionKey;																									// pointer to key
static uint8_t Nonce[16];																												// Nonce
static uint8_t initVector[16];																									// init vector passed to en/decode
static uint8_t encrypt_packet[MAX_ENCRYPT_PACKET_LENGTH];												// Used to hold data that we are going to encrypt
static EncryptionLevel_t  encryption_level;																			// encryption bits, enum
static const uint16_t KeySize[KEY_SIZE_UNDEFINED] = {0,128/8};									// the keycode length for supported length enums
static uint8_t key_size_code = 0;																								// local var for keeping current key length
static uint8_t CounterPos=0;

// ******************** global variables *****************************
// ******************** local function prototypes ********************
static void aes_initkey();
static void aes_InitNonce();
// ********************* Implementation ******************************

uint16_t AES_KEY_LENGTH(EncryptionLevel_t level)
{
	if(level < KEY_SIZE_UNDEFINED){	return(KeySize[level]);}
	else {return(0);}
}

// Indicate if encryption subsystem initialised and ready.
// returns a bool
EncryptionLevel_t aes_get_encryption_level()
{
	return encryption_level;
}

// Set status of initialistion of aes encryption
void aes_set_encryption_level(EncryptionLevel_t encryption)
{
	encryption_level = encryption;
}

// Generate EncryptionKey from aes key string provided (or default one if one provided is invalid)
static void aes_initkey()
{
	EncryptionKey = param_get_encryption_key();
}

static void aes_InitNonce()
{
	uint8_t i;
  srand(crc16(32, param_get_encryption_key()));
	for (i = 0; i < sizeof(Nonce)/sizeof(Nonce[0]); i++)
	{
		Nonce[i] = rand();																											// randomize nonce
	}
	CounterPos = ((uint8_t)rand()) % (sizeof(initVector)/sizeof(initVector[0]));// randomize counter position in nonce
}

// do some random crap with the counter for successive 128 bits of data
void AES_CTRUpdate32Bit(uint8_t *ctr)
{
  uint32_t *_ctr = (uint32_t *)ctr;
  _ctr[3] = ((_ctr[3]) + 1);
}

// Initialse variables ready for AES
// returns true if successful, or false if not
bool aes_init(EncryptionLevel_t encryption_level)
{
  CMU_ClockEnable(cmuClock_AES, true);																					// Enable AES clock
	aes_set_encryption_level(0);  																								// Initially set to zero - no encryption
	key_size_code = KeySize[encryption_level];																		// set local copy of key size
	if (key_size_code == 0) return true;																					// if no key then exit
	// Load Encryption Key
	aes_initkey();
	aes_InitNonce();
	aes_set_encryption_level(encryption_level);  																	// If up to here, must have been successful
	return true;
}

// Pad out the string to encrypt to a multiple of 16 x bytes
// returns the padded string
__STATIC_INLINE uint8_t *aes_pad(unsigned char *in_str, uint8_t len)
{
	// Copy string to encrypt to temp area, because we will be padding out encrypted string
  // and we don't want to affect any code that might be depending upon this string...or
	// any characters that the padding would overwrite. e.g. the injected packet code.
	memcpy(encrypt_packet, in_str, len);
  encrypt_packet[(len&0xf0)+15] = len;																					// store length in last byte
	return encrypt_packet;
}

// encrypt the data pointed to by in_str with length len
// returns a number indicate outcome. 0 is success
uint8_t aes_encrypt(unsigned char *in_str, uint16_t in_len, unsigned char *out_str,
			uint16_t *out_len,uint8_t SeqNo)
{
	uint8_t blocks;
	unsigned char *pt;

	if (in_len == 0) return 1;																										// Make sure we have something to encrypt
	pt = aes_pad(in_str, in_len);																									// pad buffer ,last byte is length
	blocks = 1 + (in_len>>4); 																										// Number of 16-byte blocks to encrypt
	memcpy(initVector,Nonce,sizeof(initVector));																						// copy nonce into init vector
	initVector[CounterPos] = SeqNo;																								// set counter in Nonce
  AesCtr128(EncryptionKey,pt,out_str,blocks,initVector,AES_CTRUpdate32Bit);		  // encrypt blocks, subsequent blocks have different vector
  while (!AesFinished());																												// Wait for AES to finish
	*out_len = 16 * blocks;																												// Set size of encrypted cipher in bytes
	return 0;
}

// decrypt the data pointed to by in_str with length in_len
// returns a number indicate outcome. 0 is success
uint8_t aes_decrypt(unsigned char *in_str, uint8_t in_len, unsigned char *out_str,
			uint8_t *out_len, uint8_t SeqNo)
{
	uint8_t blocks;
	if(in_len == 0) return 1;																										  // Make sure we have something to decrypt
	if(0 != (in_len & 0x0f))return 1;																						// Make sure it is a multiple of encoding size
	blocks = (in_len>>4); 																										// Number of 16-byte blocks to encrypt
	memcpy(initVector,Nonce,sizeof(initVector));																						// copy nonce into init vector
	initVector[CounterPos] = SeqNo;																								// set counter in Nonce
  AesCtr128(EncryptionKey,in_str,encrypt_packet,blocks,initVector,AES_CTRUpdate32Bit);	// decrypt blocks, subsequent blocks have different vector
  while (!AesFinished());																												// Wait for AES to finish
  *out_len = encrypt_packet[in_len-1];																					// Set size of decrypted ciper text, taking into account the padding
	if(*out_len >= in_len)return 1;																								// something went wrong
  memcpy(out_str,encrypt_packet,*out_len);																			// copy into output buffer, input and output may be the same so need a different buffer
	return 0;
}
// ********************* aes.c ***************************************
