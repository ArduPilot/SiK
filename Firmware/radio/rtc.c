// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2012 Michael Smith, All Rights Reserved
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
/// @file	rtc.c
///
/// Interface to the SmaRTClock
///

#include "radio.h"

#ifdef USE_RTC
static uint8_t	rtc_read_reg(uint8_t reg);
static void	rtc_write_reg(uint8_t reg, uint8_t val);

#define CN_RUN		(RTC_CN_EN | RTC_CN_TR)
#define CN_CAPTURE	(CN_RUN | RTC_CN_CAP)
#define CN_LOAD		(CN_RUN | RTC_CN_SET)

#define EX0_SAVE_DISABLE __bit EX0_saved = EX0; EX0 = 0
#define EX0_RESTORE EX0 = EX0_saved

/// initialise the RTC subsystem
void
rtc_init(void)
{
	// unlock the RTC
	RTC0KEY = 0xa5;
	RTC0KEY = 0xf1;

	// configure for self-oscillation
	rtc_write_reg(RTC_PIN, RTC_PIN_SELF_OSC);

	// double the clock rate
	rtc_write_reg(RTC_XCN, RTC_XCN_BIASX2);

	// start it running
	rtc_write_reg(RTC_CN, CN_RUN);
}

/// return the timer as a 32 bit value.
///
/// @return			The RTC clock in 25usec units
///
uint32_t
rtc_read_count(void)
{
	union {
		uint8_t		b[4];
		uint32_t	l;
	} mix;

	// start a capture
	rtc_write_reg(RTC_CN, CN_CAPTURE);

	// wait for capture to complete
	while (rtc_read_reg(RTC_CN) & RTC_CN_CAP) ;

	// capture the current counter value registers
	// using a auto-incrementing strobe read
	RTC0ADR = RTC0ADR_BUSY | RTC0ADR_AUTO | RTC0ADR_SHORT;
	__asm
	NOP
	NOP
	NOP
	__endasm;
	mix.b[0] = RTC0DAT;
	__asm
	NOP
	NOP
	NOP
	__endasm;
	mix.b[1] = RTC0DAT;
	__asm
	NOP
	NOP
	NOP
	__endasm;
	mix.b[2] = RTC0DAT;
	__asm
	NOP
	NOP
	NOP
	__endasm;
	mix.b[3] = RTC0DAT;

	return mix.l;
}

/// return the RTC timer as a 16 bit value.
///
/// @return			The RTC clock in 25usec units
///
/// Note: this call takes about 50usec
uint16_t
rtc_read_count16(void)
{
	uint16_t ret;

	EX0_SAVE_DISABLE;

	// start a capture
	rtc_write_reg(RTC_CN, CN_CAPTURE);

	// wait for capture to complete
	while (rtc_read_reg(RTC_CN) & RTC_CN_CAP) ;

	// capture the current counter value registers
	// using a auto-incrementing strobe read
	RTC0ADR = RTC0ADR_BUSY | RTC0ADR_AUTO | RTC0ADR_SHORT;
	__asm
	NOP
	NOP
	NOP
	__endasm;
	ret = RTC0DAT;
	__asm
	NOP
	NOP
	NOP
	__endasm;
	ret |= (RTC0DAT<<8);

	EX0_RESTORE;

	return ret;
}

/// return the timer as a 8 bit value.
///
/// @return			The RTC clock in 25usec units
///
uint8_t
rtc_read_low(void)
{
	// capture the current counter value
	rtc_write_reg(RTC_CN, CN_CAPTURE);

	// wait for capture to complete
	while (rtc_read_reg(RTC_CN) & RTC_CN_CAP) ;

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
	while (RTC0ADR & RTC0ADR_BUSY) ; // wait for busy to clear
	RTC0ADR = RTC0ADR_BUSY | reg;
	while (RTC0ADR & RTC0ADR_BUSY) ; // wait for busy to clear
	return RTC0DAT;
}

static void
rtc_write_reg(uint8_t reg, uint8_t val)
{
	while (RTC0ADR & RTC0ADR_BUSY) ; // wait for busy to clear
	RTC0ADR = reg;
	RTC0DAT = val;
}
#endif
