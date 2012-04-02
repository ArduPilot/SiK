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

/// how many channels are we hopping over
__pdata uint8_t num_fh_channels;

/// whether we current have good lock with the other end
static bool have_radio_lock;

/// current transmit channel
/// This changes every time the TDM transmit window opens or closes,
/// regardless of our lock state
__pdata static volatile uint8_t transmit_channel;

/// current receive channel
/// When we have good lock with the other radio the receive channel
/// follows the transmit channel. When we don't have lock the receive
/// channel only changes
/// very slowly - it moves only when the transmit channel wraps
__pdata static volatile uint8_t receive_channel;

/// map between hopping channel numbers and physical channel numbers
__xdata static uint8_t channel_map[MAX_FREQ_CHANNELS];

// a vary simple array shuffle
// based on shuffle from
// http://benpfaff.org/writings/clc/shuffle.html
static inline void shuffle(__xdata uint8_t *array, uint8_t n)
{
	uint8_t i;
	for (i = 0; i < n - 1; i++) {
		uint8_t j = ((uint8_t)rand()) % n;
		uint8_t t = array[j];
		array[j] = array[i];
		array[i] = t;
	}
}

// initialise frequency hopping logic
void 
fhop_init(uint16_t netid)
{
	uint8_t i;
	// create a random mapping between virtual and physical channel
	// numbers, seeded by the network ID
	for (i = 0; i < num_fh_channels; i++) {
		channel_map[i] = i;
	}
	srand(netid);
	shuffle(channel_map, num_fh_channels);
}

// tell the TDM code what channel to transmit on
uint8_t 
fhop_transmit_channel(void)
{
	return channel_map[transmit_channel];
}

// tell the TDM code what channel to receive on
uint8_t 
fhop_receive_channel(void)
{
	return channel_map[receive_channel];
}

// called when the transmit windows changes owner
void 
fhop_window_change(void)
{
	transmit_channel = (transmit_channel + 1) % num_fh_channels;
	if (have_radio_lock) {
		// when we have lock, the receive channel follows the
		// transmit channel
		receive_channel = transmit_channel;
	} else if (transmit_channel == 0) {
		// when we don't have lock, the receive channel only
		// changes when the transmit channel wraps
		receive_channel = (receive_channel + 1) % num_fh_channels;
		debug("Trying RCV on channel %d\n", (int)receive_channel);
	}
}

// called when we get or lose radio lock
void 
fhop_set_locked(bool locked)
{
#if DEBUG
	if (locked && !have_radio_lock) {
		debug("FH lock\n");
	}
#endif
	have_radio_lock = locked;
	if (have_radio_lock) {
		// we have just received a packet, so we know the
		// other radios transmit channel must be our receive
		// channel
		transmit_channel = receive_channel;
	} else {
		// try the next receive channel
		receive_channel = (receive_channel+1) % num_fh_channels;
	}
}

