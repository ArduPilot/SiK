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
#include "crc.h"

static __xdata uint8_t ebuf[MAX_AIR_PACKET_LENGTH];

// first attempt at error correcting code

bool
ecc_transmit(uint8_t length, __xdata uint8_t * __pdata buf, __pdata uint16_t timeout_ticks)
{
	uint8_t elen;
	__pdata uint16_t crc;

	// round to a multiple of 3 bytes
	elen = 3*((length+2)/3);

	if (elen+3 > MAX_AIR_PACKET_LENGTH/2) {
		panic("ecc packet too long");
	}
	buf[elen] = length;
	crc = crc16(elen+1, buf);
	buf[elen+1] = crc & 0xFF;
	buf[elen+2] = crc >> 8;
	memset(ebuf, 'Z', (elen+3)*2);
	{
		uint8_t i;
		for (i=0; i<elen+3; i++) {
			ebuf[i*2] = (elen+3)-i;
			ebuf[i*2+1] = buf[i];
		}
	}
	//golay_encode(elen+3, buf, ebuf);
	return radio_transmit((elen+3)*2, ebuf, timeout_ticks);
}

static uint8_t num_bytes_different(__xdata uint8_t * __pdata buf1, 
				   __xdata uint8_t * __pdata buf2, 
				   uint8_t n)
{
	uint8_t ret = 0;
	while (n--) {
		if (*buf1++ != *buf2++) ret++;
	}
	return ret;
}

bool
ecc_receive(uint8_t *length, __xdata uint8_t * __pdata buf)
{
	uint8_t elen;
	__pdata uint16_t crc1, crc2;

	if (!radio_receive_packet(&elen, ebuf)) {
		return false;
	}
	if (elen < 6 || elen > MAX_AIR_PACKET_LENGTH || (elen % 6) != 0) {
		printf("invalid elen %u\n", (unsigned)elen);
		return false;
	}
	{
		uint8_t i;
		for (i=0; i<elen/2; i++) {
			buf[i] = ebuf[i*2+1];
		}
	}
//	golay_decode(elen, ebuf, buf);

	elen >>= 1;
	crc1 = buf[elen-2] | (((uint16_t)buf[elen-1])<<8);
	crc2 = crc16(elen-2, buf);
	{
		__xdata uint8_t ebuf2[MAX_AIR_PACKET_LENGTH];
		uint8_t nfix;
		memset(ebuf2, 'Z', elen*2);
		{
			uint8_t i;
			for (i=0; i<elen; i++) {
				ebuf2[i*2] = elen-i;
				ebuf2[i*2+1] = buf[i];
			}
		}
	//golay_encode(elen, buf, ebuf2);
		nfix = num_bytes_different(ebuf, ebuf2, elen*2);
		if (crc1 != crc2) {
			uint8_t i;
			printf("corrected %u bytes (crc1=%x crc2=%x len=%u)\n", 
			       (unsigned)nfix, crc1, crc2, (unsigned)buf[elen-3]);
			for (i=0; i<elen*2; i++) {
				printf("%x/%x ", 
				       (unsigned)ebuf[i], (unsigned)ebuf2[i]);
			}
			printf("\n");
			for (i=0; i<elen-5; i++) {
				printf("%c", buf[i]);
			}
			printf("\n");
		}
	}

	if (crc1 != crc2) {
		printf("crc1=%x crc2=%x\n", crc1, crc2);
		return false;
	}
	*length = buf[elen-3];
	return true;
}
