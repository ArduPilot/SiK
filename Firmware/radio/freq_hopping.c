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

/*
  frequency hopping table derived from C1101 registers from 
  http://www.raydees.com/uploads/Davis.zip
 */
static __code uint32_t frequencies[51] = {
	911380218UL, 902349090UL, 911882476UL, 922921051UL, 914892852UL, 906363189UL, 925931427UL, 918405487UL, 
	908872100UL, 920412139UL, 913387664UL, 903854278UL, 916899505UL, 924426239UL, 910377288UL, 904858001UL, 
	915896575UL, 921415863UL, 907366912UL, 926935150UL, 912886199UL, 903352813UL, 917401763UL, 923422515UL, 
	909373565UL, 926432891UL, 905861724UL, 914391387UL, 919408416UL, 924927703UL, 902850555UL, 910878753UL, 
	921917327UL, 915394317UL, 906864654UL, 917903228UL, 927436614UL, 920913604UL, 908369842UL, 912383941UL, 
	918906951UL, 904355743UL, 923923980UL, 916398040UL, 909875030UL, 919910675UL, 905359466UL, 922418792UL, 
	907868377UL, 913889129UL, 925429962UL
};

__pdata static volatile uint8_t receive_channel;

// tell the loop code what channel to receive on
uint32_t 
fhop_receive_freqency(void)
{
	return frequencies[receive_channel];
}

// called when the transmit windows changes owner
void 
fhop_next(void)
{
	receive_channel = (receive_channel + 1) % 51;
}


