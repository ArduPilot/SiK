// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2012 Andrew Tridgell, All Rights Reserved
// Copyright (c) 2011 Michael Smith, All Rights Reserved
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
/// @file	loop.c
///
/// main loop
///

#include <stdarg.h>
#include "radio.h"
#include "loop.h"
#include "timer.h"
#include "freq_hopping.h"

/// a packet buffer for the LOOP code
__xdata uint8_t	pbuf[MAX_PACKET_LENGTH];

// a stack carary to detect a stack overflow
__at(0xFF) uint8_t __idata _canary;


static uint8_t swap_bit_order(register uint8_t b) 
{
    /*
     * The data bytes come over the air from the ISS least significant bit first.  Fix them as we go. From
     * http://www.ocf.berkeley.edu/~wwu/cgi-bin/yabb/YaBB.cgi?board=riddles_cs;action=display;num=1103355188
     */
    b = ((b & 0b11110000) >>4 ) | ((b & 0b00001111) << 4);
    b = ((b & 0b11001100) >>2 ) | ((b & 0b00110011) << 2);
    b = ((b & 0b10101010) >>1 ) | ((b & 0b01010101) << 1);
    return b;
}


static void 
swap_packet_bit_order(__pdata uint8_t len)
{
	__pdata uint8_t i;
	for (i=0; i<len; i++) {
		pbuf[i] = swap_bit_order(pbuf[i]);
	}
}

static __pdata one_second_counter;

static void one_second(void)
{
	one_second_counter++;
	if (one_second_counter == 4) {
		fhop_next();
		radio_set_frequency(fhop_receive_freqency());
		radio_receiver_on();
		one_second_counter = 0;
		printf("Searching...\n");
	}
}

/// main loop for time division multiplexing transparent serial
///
void
serial_loop(void)
{
	__pdata uint16_t last_t = timer2_tick();
	__pdata uint16_t last_link_update = last_t;

	_canary = 42;

	// set right receive channel
	radio_set_frequency(fhop_receive_freqency());

	delay_set_ticks(100);

	for (;;) {
		__pdata uint8_t	len;

		if (delay_expired()) {
			one_second();
			delay_set_ticks(100);
		}

		if (_canary != 42) {
			panic("stack blown\n");
		}

		if (pdata_canary != 0x41) {
			panic("pdata canary changed\n");
		}

		// give the AT command processor a chance to handle a command
		at_command();

		// see if we have received a packet
		if (radio_receive_packet(&len, pbuf)) {
			__pdata uint8_t i;
			swap_packet_bit_order(len);
			for (i=0; i<len; i++) {
				printf("%x ", (unsigned)pbuf[i]);
			}
			printf("\n");
			fhop_next();

			// re-enable the receiver
			radio_set_frequency(fhop_receive_freqency());
			radio_receiver_on();

			delay_set_ticks(100);
			one_second_counter = 0;
		}
	}
}


