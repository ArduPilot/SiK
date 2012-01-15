// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2012 Michael Smith, All Rights Reserved
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
/// @file	rtc.c
///
/// Interface to the SmaRTClock
///

#include "radio.h"

static uint8_t	rtc_read_reg(uint8_t reg);
static void	rtc_write_reg(uint8_t reg, uint8_t val);

#define CN_RUN		(RTC_CN_EN)
#define CN_CAPTURE	(CN_RUN | RTC_CN_CAP)
#define CN_LOAD		(CN_RUN | RTC_CN_SET)

void
rtc_init(void)
{
	// unlock the RTC
	RTC0KEY = 0xa5;
	RTC0KEY = 0xf1;

	// configure for self-oscillation
	rtc_write_reg(RTC_PIN, RTC_PIN_SELF_OSC);
	rtc_write_reg(RTC_XCN, RTC_XCN_BIASX2);
	rtc_write_reg(RTC_CN, CN_RUN);
}

uint32_t
rtc_read_count(void)
{
	union {
		uint8_t		b[4];
		uint32_t	l;
	} mix;

	// capture the current counter value
	rtc_write_reg(RTC_CN, CN_CAPTURE);

	// read the counter shadow
	mix.b[0] = rtc_read_reg(RTC_CAPTURE0);
	mix.b[1] = rtc_read_reg(RTC_CAPTURE1);
	mix.b[2] = rtc_read_reg(RTC_CAPTURE2);
	mix.b[3] = rtc_read_reg(RTC_CAPTURE3);

	return mix.l;
}

uint8_t
rtc_read_low(void)
{
	// capture the current counter value
	rtc_write_reg(RTC_CN, CN_CAPTURE);

	// read the counter shadow
	return rtc_read_reg(RTC_CAPTURE0);
}

void
rtc_write_count(uint32_t count)
{
	union {
		uint8_t		b[4];
		uint32_t	l;
	} mix;

	// write the counter shadow
	mix.l = count;
	rtc_write_reg(RTC_CAPTURE0, mix.b[0]);
	rtc_write_reg(RTC_CAPTURE1, mix.b[1]);
	rtc_write_reg(RTC_CAPTURE2, mix.b[2]);
	rtc_write_reg(RTC_CAPTURE3, mix.b[3]);

	// load the new value into the counter
	rtc_write_reg(RTC_CN, CN_LOAD);
}

static uint8_t
rtc_read_reg(uint8_t reg)
{
	RTC0ADR = reg;
	__asm
	NOP
	NOP
	NOP
	__endasm;
	return RTC0DAT;
}

static void
rtc_write_reg(uint8_t reg, uint8_t val)
{
	RTC0ADR = reg;
	RTC0DAT = val;
	__asm
	NOP
	__endasm;
}

