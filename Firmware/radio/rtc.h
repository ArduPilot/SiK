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
/// @file	rtc.h
///
/// Interface to the SmaRTClock
///

// RTC internal registers
// note that values include the short strobe and start bits
#define RTC_CAPTURE0	0x90
#define RTC_CAPTURE1	0x91
#define RTC_CAPTURE2	0x92
#define RTC_CAPTURE3	0x93
#define RTC_CN		0x94
#define RTC_XCN		0x95
#define RTC_XCF		0x96
#define RTC_PIN		0x97
#define RTC_ALARM0	0x98
#define RTC_ALARM1	0x99
#define RTC_ALARM2	0x9a
#define RTC_ALARM3	0x9b

// RTC_CN
#define RTC_CN_EN		(1<<7)
#define RTC_CN_MCLKEN		(1<<6)
#define RTC_CN_OSCFAIL		(1<<5)
#define RTC_CN_TR		(1<<4)
#define RTC_CN_AEN		(1<<3)
#define RTC_CN_ALRM		(1<<2)
#define RTC_CN_SET		(1<<1)
#define RTC_CN_CAP		(1<<0)

// RTC_XCN
#define RTC_XCN_AGCEN		(1<<7)
#define RTC_XCN_XMODE		(1<<6)
#define RTC_XCN_BIASX2		(1<<5)
#define RTC_XCN_CLKVLD		(1<<4)

// RTC_XCF
#define RTC_XCF_AUTOSTP		(1<<7)
#define RTC_XCF_LOADRDY		(1<<6)

// RTC_PIN
#define RTC_PIN_SELF_OSC	0xe7
#define RTC_PIN_XTAL_OSC	0x67

extern void	rtc_init(void);
extern uint32_t	rtc_read_count(void);
extern uint8_t	rtc_read_low(void);
extern void	rtc_write_count(uint32_t val);
