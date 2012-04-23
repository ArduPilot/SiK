// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2011 Michael Smith, All Rights Reserved
// Copyright (c) 2011 Andrew Tridgell, All Rights Reserved
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

#include "radio.h"
#include "timer.h"

/// Counter used by delay_msec
///
static __data volatile uint8_t delay_counter;

/// high 16 bits of timer2 SYSCLK/12 interrupt
static __data volatile uint16_t timer2_high;


INTERRUPT(T3_ISR, INTERRUPT_TIMER3)
{
	// re-arm the interrupt by clearing TF3H
	TMR3CN = 0x04;

	// call the AT parser tick
	at_timer();

	// update the delay counter
	if (delay_counter > 0)
		delay_counter--;
}

void
delay_set(register uint16_t msec)
{
	if (msec >= 2550) {
		delay_counter = 255;
	} else {
		delay_counter = (msec + 9) / 10;
	}
}

void 
delay_set_ticks(register uint8_t ticks)
{
	delay_counter = ticks;
}

bool
delay_expired(void)
{
	return delay_counter == 0;
}

void
delay_msec(register uint16_t msec)
{
	delay_set(msec);
	while (!delay_expired())
		;
}


// timer2 interrupt called every 32768 microseconds
INTERRUPT(T2_ISR, INTERRUPT_TIMER2)
{
	// re-arm the interrupt by clearing TF2H
	TMR2CN = 0x04;

	// increment the high 16 bits
	timer2_high++;

	if (feature_rtscts) {
		serial_check_rts();
	}
}

// return the 16 bit timer2 counter
// this call costs about 2 microseconds
uint16_t 
timer2_16(void)
{
	register uint8_t low, high;
	do {
		// we need to make sure that the high byte hasn't changed
		// between reading the high and low parts of the 16 bit timer
		high = TMR2H;
		low = TMR2L;
	} while (high != TMR2H);
	return low | (((uint16_t)high)<<8);
}

#if 0
// return microseconds since boot
// this call costs about 5usec
uint32_t 
micros(void)
{
	uint16_t low, high;
	do {
		high = timer2_high;
		low = timer2_16();
	} while (high != timer2_high);
	return ((((uint32_t)high)<<16) | low) >> 1;
}
#endif

// return a 16 bit value that rolls over in approximately
// one second intervals
uint16_t 
timer2_tick(void)
{
	register uint16_t low, high;
	do {
		high = timer2_high;
		low = timer2_16();
	} while (high != timer2_high);
	// take 11 high bits from the 2MHz counter, and 5 low bits
	// from the rollover of that counter
	return (high<<11) | (low>>5);
}

// initialise timers
void 
timer_init(void)
{
	// 100Hz timer tick using timer3
	// Derive timer values from SYSCLK, just for laughs.
	TMR3RLL	 = (65536UL - ((SYSCLK / 12) / 100)) & 0xff;
	TMR3RLH	 = ((65536UL - ((SYSCLK / 12) / 100)) >> 8) & 0xff;
	TMR3CN	 = 0x04;	// count at SYSCLK / 12 and start
	EIE1	|= 0x80;

	// setup TMR2 as a timer source at SYSCLK/12
	TMR2RLL = 0;
	TMR2RLH = 0;
	TMR2CN  = 0x04; // start running, count at SYSCLK/12
	ET2 = 1;
}

// return some entropy
uint8_t
timer_entropy(void)
{
	// use the SYSCLK/12 timer
	return TMR2L;
}
