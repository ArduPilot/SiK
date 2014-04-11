// XTEA block cipher implementation for SDCC (C)2007 by Jan Waclawek (wek at efton dot sk)


#include <stdint.h>

#include "xtea.h"

xtea_data_t data xtea_data;


// you can chose between the "canonical" and "asm" implementation commenting/uncommenting the CANONICAL define in xtea.h

#ifdef XTEA_CANONICAL

#  ifdef XTEA_ENCRYPT
// the "canonical" implementation based on the Wheeler & Needham paper 
// -- just slightly less showoff with "true-C" than them, but still much left on precedence, 
//    so it might be used also as regression test that the compiler gets precedence right, hehe
// -- occupies 11 bytes of overlayable data area
// -- there might be a slight optimiation step eliminating 1 byte in data (the loop counter i),
//    if we would examine the schedule of sum and test for the final value of some of its bytes
// sdcc default settings -> this routine is 13424 cycles, 589 bytes of code

void xtea(uint32_t code * k) {

  uint32_t data sum;
  unsigned char i;

  sum = 0;

  for (i=0; i<32; i++) {
    xtea_data.l.y += (xtea_data.l.z<<4 ^ xtea_data.l.z>>5) + xtea_data.l.z ^ sum + k[sum&3];
    sum += 0x9e3779b9;
    xtea_data.l.z += (xtea_data.l.y<<4 ^ xtea_data.l.y>>5) + xtea_data.l.y ^ sum + k[sum>>11 &3];
  }

}


#  endif
#  ifdef XTEA_DECRYPT

// the following is the inverse, again based on the original implementation by Wheeler & Needham
// -- occupies 11 bytes of overlayable data area
// sdcc default settings -> this routine is 13492 cycles, 595 bytes of code

void xtea_i(uint32_t code * k) {

  uint32_t data sum;
  unsigned char i;

  sum = 0xC6EF3720;

  for (i=0; i<32; i++) {
    xtea_data.l.z -= (xtea_data.l.y<<4 ^ xtea_data.l.y>>5) + xtea_data.l.y ^ sum + k[sum>>11 &3];
    sum -= 0x9e3779b9;
    xtea_data.l.y -= (xtea_data.l.z<<4 ^ xtea_data.l.z>>5) + xtea_data.l.z ^ sum + k[sum&3];
  }

}


#  endif


#else


#  ifdef XTEA_ENCRYPT

// optimised asm solution
// occupies slightly more DATA space (8 bytes of local variables)
// - due to explicit definition of xtea_sum and xtea_tmp as data storage class (to prevent clash if 
//   compiled under different model), these are nonoverlayable - this can be experimented with in case of
//   data memory shortage
//   
// input/output data share the same 8-byte data space, which is defined as global xtea_data (see above), 
//   containing xtea_y and xtea_z (as a union with an array of bytes for convenience of caller)
//
// key is defined by the caller and passed via pointer
//
// 6950 cycle, 199 bytes code space


void xtea(uint32_t code * k) __naked {

  static uint32_t data sum, tmp;

  sum; tmp; k;   // hush the compiler

// thanks to "naked" attribute, we can use ret and also the non-local labels!

__asm

xtea_y = _xtea_data+0  ;// _xtea_data.l.y
xtea_z = _xtea_data+4  ;// _xtea_data.l.z

XTea:
      clr   a
      mov   _xtea_sum_1_1+0,a   ;sum = 0
      mov   _xtea_sum_1_1+1,a
      mov   _xtea_sum_1_1+2,a
      mov   _xtea_sum_1_1+3,a
      mov   r2,#32*2   ;nr of rounds *2 (because of trick with twice the main code, one for y and one for z; and another inside...)

// we have this set as input parameter!      mov   dptr,#key  ;dptr will not change
TeaRound:            

      mov   r4,xtea_z+0
      mov   r5,xtea_z+1
      mov   r6,xtea_z+2
      mov   r7,xtea_z+3
      
TeaSubRound:
      mov   r0,#_xtea_tmp_1_1+3    ;tmp = z << 4
      mov   a,r7
      swap  a
      mov   @r0,a            ;@r0=tmp3
      mov   a,r6
      swap  a
      xchd  a,@r0            ;@r0=tmp3
      dec   r0
      mov   @r0,a            ;@r0=tmp2
      mov   a,r5
      swap  a
      xchd  a,@r0            ;@r0=tmp2
      dec   r0
      mov   @r0,a            ;@r0=tmp1
      mov   a,r4
      swap  a
      xchd  a,@r0            ;@r0=tmp1
      mov   _xtea_tmp_1_1+0,a
      anl   _xtea_tmp_1_1+0,#0xF0

      rrc   a              ;tmp ^=  z >> 5
      anl   a,#0x07
      xrl   a,_xtea_tmp_1_1+3
      xch   a,_xtea_tmp_1_1+3
      rrc   a 
      xrl   a,_xtea_tmp_1_1+2
      xch   a,_xtea_tmp_1_1+2
      rrc   a 
      xrl   a,@r0  ;tmp1
      xch   a,@r0  ;tmp1
      rrc   a 
      xrl   a,_xtea_tmp_1_1+0


      add   a,r4         ;z = z+tmp
      mov   r4,a
      mov   a,r5
      addc  a,_xtea_tmp_1_1+1
      mov   r5,a
      mov   a,r6
      addc  a,_xtea_tmp_1_1+2
      mov   r6,a
      mov   a,r7
      addc  a,_xtea_tmp_1_1+3
      mov   r7,a

      mov   a,r2
      jb    acc.0,TeaX1
      mov   a,_xtea_sum_1_1+0         ;r0 = [sum&3]
      rl    a
      rl    a
      sjmp  TeaX2
TeaX1:
      mov   a,_xtea_sum_1_1+1         ;r0 = [sum>>11&3]
      rr    a
TeaX2:
      anl   a,#0x0C
      mov   r0,a

      movc  a,@a+dptr      ;result ^= sum + k[pointer]
      inc   r0
      add   a,_xtea_sum_1_1+0
      xrl   a,r4
      mov   r4,a
      mov   a,r0
      movc  a,@a+dptr
      inc   r0
      addc  a,_xtea_sum_1_1+1
      xrl   a,r5
      mov   r5,a
      mov   a,r0
      movc  a,@a+dptr
      inc   r0
      addc  a,_xtea_sum_1_1+2
      xrl   a,r6
      mov   r6,a
      mov   a,r0
      movc  a,@a+dptr
      addc  a,_xtea_sum_1_1+3
      xrl   a,r7
      mov   r7,a

      dec   r2
      mov   a,r2
      jb    acc.0,TeaSubRound2

      mov   a,r4
      add   a,xtea_z+0
      mov   xtea_z+0,a
      mov   a,r5
      addc  a,xtea_z+1
      mov   xtea_z+1,a
      mov   a,r6
      addc  a,xtea_z+2
      mov   xtea_z+2,a
      mov   a,r7
      addc  a,xtea_z+3
      mov   xtea_z+3,a

      cjne  r2,#0,TeaRoundA
      ret            
TeaRoundA:
      ljmp  TeaRound

TeaSubRound2:      
      mov   a,r4
      add   a,xtea_y+0
      mov   xtea_y+0,a
      mov   r4,a
      mov   a,r5
      addc  a,xtea_y+1
      mov   xtea_y+1,a
      mov   r5,a
      mov   a,r6
      addc  a,xtea_y+2
      mov   xtea_y+2,a
      mov   r6,a
      mov   a,r7
      addc  a,xtea_y+3
      mov   xtea_y+3,a
      mov   r7,a

      mov   a,_xtea_sum_1_1+0   ;sum += delta
      add   a,#0xB9    ;delta[0]
      mov   _xtea_sum_1_1+0,a
      mov   a,_xtea_sum_1_1+1
      addc  a,#0x79    ;delta[1]
      mov   _xtea_sum_1_1+1,a
      mov   a,_xtea_sum_1_1+2
      addc  a,#0x37    ;delta[2]
      mov   _xtea_sum_1_1+2,a
      mov   a,_xtea_sum_1_1+3
      addc  a,#0x9E    ;delta[3]
      mov   _xtea_sum_1_1+3,a
 
      ljmp  TeaSubRound

__endasm;

}



#  endif
#  ifdef XTEA_DECRYPT

// the same usage of variables as in XTea above
// 7049 cycle, 205 bytes code space


void xtea_i(uint32_t code * k) __naked {

  static uint32_t data sum, tmp;

  sum; tmp; k;   // hush the compiler

// thanks to "naked" attribute, we can use ret and also the non-local labels!

__asm

xtea_i_y = _xtea_data+0  ;// _xtea_data.l.y
xtea_i_z = _xtea_data+4  ;// _xtea_data.l.z



XTea_i:
      mov   r2,#32*2   ;nr of rounds *2 (because of trick with twice the main code, one for y and one for z; and another inside...)
      mov   _xtea_i_sum_1_1+3,#0xC6
      mov   _xtea_i_sum_1_1+2,#0xEF
      mov   _xtea_i_sum_1_1+1,#0x37
      mov   _xtea_i_sum_1_1+0,#0x20

TeaIRound:            

      mov   r4,xtea_i_y+0
      mov   r5,xtea_i_y+1
      mov   r6,xtea_i_y+2
      mov   r7,xtea_i_y+3
      
// we have this set as input parameter!      mov   dptr,#key  ;dptr will not change
TeaISubRound:
      mov   r0,#_xtea_i_tmp_1_1+3    ;tmp = y << 4 
      mov   a,r7
      swap  a
      mov   @r0,a            ;@r0=tmp3
      mov   a,r6
      swap  a
      xchd  a,@r0            ;@r0=tmp3
      dec   r0
      mov   @r0,a            ;@r0=tmp2
      mov   a,r5
      swap  a
      xchd  a,@r0            ;@r0=tmp2
      dec   r0
      mov   @r0,a            ;@r0=tmp1
      mov   a,r4
      swap  a
      xchd  a,@r0            ;@r0=tmp1
      mov   _xtea_i_tmp_1_1+0,a
      anl   _xtea_i_tmp_1_1+0,#0xF0

      rrc   a              ;tmp ^=  y >> 5
      anl   a,#0x07
      xrl   a,_xtea_i_tmp_1_1+3
      xch   a,_xtea_i_tmp_1_1+3
      rrc   a 
      xrl   a,_xtea_i_tmp_1_1+2
      xch   a,_xtea_i_tmp_1_1+2
      rrc   a 
      xrl   a,@r0  ;tmp1
      xch   a,@r0  ;tmp1
      rrc   a 
      xrl   a,_xtea_i_tmp_1_1+0

      add   a,r4         ;y = y+tmp
      mov   r4,a
      mov   a,r5
      addc  a,_xtea_i_tmp_1_1+1
      mov   r5,a
      mov   a,r6
      addc  a,_xtea_i_tmp_1_1+2
      mov   r6,a
      mov   a,r7
      addc  a,_xtea_i_tmp_1_1+3
      mov   r7,a

      mov   a,r2
      jnb   acc.0,TeaIX1
      mov   a,_xtea_i_sum_1_1+0         ;r0 = [sum&3]
      rl    a
      rl    a
      sjmp  TeaIX2
TeaIX1:
      mov   a,_xtea_i_sum_1_1+1         ;r0 = [sum>>11&3]
      rr    a
TeaIX2:
      anl   a,#0x0C
      mov   r0,a

      movc  a,@a+dptr      ;result ^= sum + k[pointer]
      inc   r0
      add   a,_xtea_i_sum_1_1+0
      xrl   a,r4
      mov   r4,a
      mov   a,r0
      movc  a,@a+dptr
      inc   r0
      addc  a,_xtea_i_sum_1_1+1
      xrl   a,r5
      mov   r5,a
      mov   a,r0
      movc  a,@a+dptr
      inc   r0
      addc  a,_xtea_i_sum_1_1+2
      xrl   a,r6
      mov   r6,a
      mov   a,r0
      movc  a,@a+dptr
      addc  a,_xtea_i_sum_1_1+3
      xrl   a,r7
      mov   r7,a

      dec   r2
      mov   a,r2
      jb    acc.0,TeaISubRound2

      clr   c
      mov   a,xtea_i_y+0
      subb  a,r4
      mov   xtea_i_y+0,a
      mov   a,xtea_i_y+1
      subb  a,r5
      mov   xtea_i_y+1,a
      mov   a,xtea_i_y+2
      subb  a,r6
      mov   xtea_i_y+2,a
      mov   a,xtea_i_y+3
      subb  a,r7
      mov   xtea_i_y+3,a

      cjne  r2,#0,TeaIRoundA
      ret            
TeaIRoundA:
      ljmp  TeaIRound

TeaISubRound2:  
      clr   c    
      mov   a,xtea_i_z+0
      subb  a,r4
      mov   xtea_i_z+0,a
      mov   r4,a
      mov   a,xtea_i_z+1
      subb  a,r5
      mov   xtea_i_z+1,a
      mov   r5,a
      mov   a,xtea_i_z+2
      subb  a,r6
      mov   xtea_i_z+2,a
      mov   r6,a
      mov   a,xtea_i_z+3
      subb  a,r7
      mov   xtea_i_z+3,a
      mov   r7,a

      clr   c
      mov   a,_xtea_i_sum_1_1+0   ;sum += delta
      subb  a,#0xB9    ;delta[0]
      mov   _xtea_i_sum_1_1+0,a
      mov   a,_xtea_i_sum_1_1+1
      subb  a,#0x79    ;delta[1]
      mov   _xtea_i_sum_1_1+1,a
      mov   a,_xtea_i_sum_1_1+2
      subb  a,#0x37    ;delta[2]
      mov   _xtea_i_sum_1_1+2,a
      mov   a,_xtea_i_sum_1_1+3
      subb  a,#0x9E    ;delta[3]
      mov   _xtea_i_sum_1_1+3,a
 
      ljmp  TeaISubRound

__endasm;

}

#  endif


#endif // XTEA_CANONICAL

