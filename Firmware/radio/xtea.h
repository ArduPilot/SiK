// XTEA block cipher implementation for SDCC (C)2007 by Jan Waclawek (wek at efton dot sk)


#ifndef _XTEA_H_
#define _XTEA_H_


typedef union {
    struct {uint32_t y, z;} l;
    uint8_t b[8];
} xtea_data_t;

extern __data xtea_data_t xtea_data;
extern __data uint32_t xtea_key[4];

//following defines determine, whether the encryption, decryption ("inverse") or both routines will be compiled
#define XTEA_ENCRYPT
#define XTEA_DECRYPT

void xtea(void);
void xtea_i(void);

extern void xtea_encrypt(__xdata uint8_t * __pdata in, __pdata uint8_t n);
extern void xtea_decrypt(__xdata uint8_t * __pdata in, __pdata uint8_t n);

#endif
