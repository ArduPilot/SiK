// XTEA block cipher implementation for SDCC (C)2007 by Jan Waclawek (wek at efton dot sk)

// See http://www.efton.sk/crypt/

// used with permission of the author

#include <stdint.h>
#include <string.h>

#include "xtea.h"

__data xtea_data_t xtea_data;
__data uint32_t xtea_key[4];

// the "canonical" implementation based on the Wheeler & Needham paper 
// -- just slightly less showoff with "true-C" than them, but still much left on precedence, 
//    so it might be used also as regression test that the compiler gets precedence right, hehe
// -- occupies 11 bytes of overlayable data area
// -- there might be a slight optimiation step eliminating 1 byte in data (the loop counter i),
//    if we would examine the schedule of sum and test for the final value of some of its bytes
// sdcc default settings -> this routine is 13424 cycles, 589 bytes of code

void xtea(void) 
{
    register uint32_t sum;
    register uint8_t i;
    
    sum = 0;
    
    for (i=0; i<32; i++) {
        xtea_data.l.y += (xtea_data.l.z<<4 ^ xtea_data.l.z>>5) + xtea_data.l.z ^ sum + xtea_key[sum&3];
        sum += 0x9e3779b9;
        xtea_data.l.z += (xtea_data.l.y<<4 ^ xtea_data.l.y>>5) + xtea_data.l.y ^ sum + xtea_key[sum>>11 &3];
    }
}


// the following is the inverse, again based on the original implementation by Wheeler & Needham
// -- occupies 11 bytes of overlayable data area
// sdcc default settings -> this routine is 13492 cycles, 595 bytes of code
void xtea_i(void) 
{
    register uint32_t sum;
    register uint8_t i;
    
    sum = 0xC6EF3720;
    
    for (i=0; i<32; i++) {
        xtea_data.l.z -= (xtea_data.l.y<<4 ^ xtea_data.l.y>>5) + xtea_data.l.y ^ sum + xtea_key[sum>>11 &3];
        sum -= 0x9e3779b9;
        xtea_data.l.y -= (xtea_data.l.z<<4 ^ xtea_data.l.z>>5) + xtea_data.l.z ^ sum + xtea_key[sum&3];
    }
}


void 
xtea_encrypt(__xdata uint8_t * __pdata in, __pdata uint8_t n)
{
    while (n > 0) {
        register uint8_t count = n;
        register uint8_t i;
        if (count > 8) count = 8;
        for (i=count; i<8; i++) {
            xtea_data.b[i] = in[i];
        }
        for (i=count; i<8; i++) {
            xtea_data.b[i] = 0;
        }
        xtea();
        for (i=0; i<count; i++) {
            in[i] = xtea_data.b[i];
        }
        in += count;
        n -= count;
    }
}

void 
xtea_decrypt(__xdata uint8_t * __pdata in, __pdata uint8_t n)
{
    while (n > 0) {
        register uint8_t count = n;
        register uint8_t i;
        if (count > 8) count = 8;
        for (i=count; i<8; i++) {
            xtea_data.b[i] = in[i];
        }
        for (i=count; i<8; i++) {
            xtea_data.b[i] = 0;
        }
        xtea_i();
        for (i=0; i<count; i++) {
            in[i] = xtea_data.b[i];
        }
        in += count;
        n -= count;
    }
}

