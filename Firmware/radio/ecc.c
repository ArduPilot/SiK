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
/// @file	eec.c
///
///

#include <stdarg.h>
#include "radio.h"
#include "golay.h"
#include "ecc.h"

static __xdata uint8_t ebuf[64];

// first attempt at error correcting code

bool
eec_transmit(uint8_t length, __xdata uint8_t *buf, __pdata uint16_t timeout_ticks)
{
	__pdata uint8_t elen = 3*((length+3)/3);
	buf[elen-1] = length;
	golay_encode(elen, buf, ebuf);
	return radio_transmit(elen*2, ebuf, timeout_ticks);
}


bool
eec_receive(uint8_t *length, __xdata uint8_t *buf)
{
	__pdata uint8_t elen;

	if (!radio_receive_packet(&elen, ebuf)) {
		return false;
	}
	if ((elen % 6) != 0) {
		// not a valid encrypted packet
		return false;
	}
	golay_decode(elen, ebuf, buf);
	*length = buf[(elen/2)-1];
	printf("elen=%u len=%u\n", 
	       (unsigned)elen, 
	       (unsigned)*length);
	return true;
}
