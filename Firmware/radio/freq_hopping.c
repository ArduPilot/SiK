// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2012 Andrew Tridgell, All Rights Reserved
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

///
/// @file	freq_hopping.c
///
/// Frequency hop managerment
///

#include <stdarg.h>
#include "radio.h"
#include "freq_hopping.h"

static __code uint8_t FREQ_2[51] =
{
  0x23, 0x22, 0x23, 0x23, 0x23, 0x22, 0x23, 0x23, 0x22, 0x23,
  0x23, 0x22, 0x23, 0x23, 0x23, 0x22, 0x23, 0x23, 0x22, 0x23,
  0x23, 0x22, 0x23, 0x23, 0x22, 0x23, 0x22, 0x23, 0x23, 0x23,
  0x22, 0x23, 0x23, 0x23, 0x22, 0x23, 0x23, 0x23, 0x22, 0x23,
  0x23, 0x22, 0x23, 0x23, 0x22, 0x23, 0x22, 0x23, 0x22, 0x23,
  0x23
};  
//
//  Table for FREQ1 settings.
//
static __code uint8_t FREQ_1[51] =
{
  0x0D, 0xB4, 0x12, 0x7F, 0x30, 0xDC, 0x9C, 0x52, 0xF4, 0x66,
  0x21, 0xC3, 0x43, 0x8E, 0x03, 0xCD, 0x3A, 0x70, 0xE6, 0xA6,
  0x1C, 0xBE, 0x48, 0x84, 0xF9, 0xA1, 0xD7, 0x2B, 0x5C, 0x92,
  0xB9, 0x08, 0x75, 0x35, 0xE1, 0x4D, 0xAB, 0x6B, 0xEF, 0x17,
  0x57, 0xC8, 0x89, 0x3E, 0xFE, 0x61, 0xD2, 0x7A, 0xEB, 0x26,
  0x97
};  
//
//  Table for FREQ0 settings.
//
static __code uint8_t FREQ_0[51] =
{
  0x97, 0xAB, 0x89, 0x39, 0x2D, 0x31, 0xDD, 0xC3, 0xE5, 0x85,
  0x5B, 0x7D, 0xEF, 0x0B, 0xB7, 0x5F, 0x0F, 0x67, 0x13, 0xBF,
  0x6B, 0x8D, 0xE1, 0x29, 0xD5, 0xCD, 0x41, 0x3D, 0xA3, 0xFB,
  0x9B, 0xA7, 0x57, 0x1D, 0x21, 0xD1, 0xAF, 0x75, 0xF3, 0x79,
  0xB3, 0x6D, 0x19, 0xFF, 0xC5, 0x95, 0x4F, 0x47, 0x03, 0x4B,
  0xED
};

__pdata static volatile uint8_t receive_channel = 5;

// tell the TDM code what channel to receive on
uint32_t 
fhop_receive_freqency(void)
{
	__pdata uint32_t freq = 
		((uint32_t)FREQ_2[receive_channel])<<16 | 
		((uint32_t)FREQ_1[receive_channel])<<8 | 
		((uint32_t)FREQ_0[receive_channel]);
	float freq2 = freq * 26000000.0 / 65536;
	return (uint32_t)freq2;
}

// called when the transmit windows changes owner
void 
fhop_next(void)
{
	receive_channel = (receive_channel + 1) % 51;
}


