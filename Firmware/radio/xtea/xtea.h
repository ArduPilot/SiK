// XTEA block cipher implementation for SDCC (C)2007 by Jan Waclawek (wek at efton dot sk)


#ifndef _XTEA_H_
#define _XTEA_H_


typedef union 
  {
    struct {uint32_t y, z;} l;
    uint8_t b[8];
  } xtea_data_t;

extern xtea_data_t data xtea_data;

// the following is the key, which should be left on the user, but we are not able to compile xtea.c without it...
// uint32_t code xtea_k[4] = {0x00010203, 0x04050607, 0x08090a0b, 0x0c0d0e0f};  
// solution: pointer to key passed as parameter



// uncomment the following define and recompile xtea.c, if you want to use the "canonical" C-implementation
//#define XTEA_CANONICAL



//following defines determine, whether the encryption, decryption ("inverse") or both routines will be compiled
#define XTEA_ENCRYPT
#define XTEA_DECRYPT





#ifdef XTEA_CANONICAL
#  ifdef XTEA_ENCRYPT
void xtea(uint32_t code *);
#  endif
#  ifdef XTEA_DECRYPT
void xtea_i(uint32_t code *);
#  endif
#else
#  ifdef XTEA_ENCRYPT
void xtea(uint32_t code *) __naked ;
#  endif
#  ifdef XTEA_DECRYPT
void xtea_i(uint32_t code *) __naked ;
#  endif
#endif

#endif
