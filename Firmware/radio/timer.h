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

/// return the 16 bit timer2 counter
///
/// @return		timer counter, in 0.5usec units
///
extern uint16_t timer2_16(void);

/// return microseconds since boot
///
/// @return		microseconds since boot
extern uint32_t micros(void);

/// return a 16 bit value that rolls over in approximately
/// one second intervals
///
/// @return		16 bit value in units of 16 microseconds
///
extern uint16_t timer2_tick(void);


/// initialise timers
///
extern void timer_init(void);

/// Set the delay timer
///
/// @note Maximum delay is ~2.5sec in the current implementation.
///
/// @param	msec		Minimum time before the timer expiers.  The actual time
///				may be greater.
///
extern void	delay_set(register uint16_t msec);

/// Set the delay timer in 100Hz ticks
///
/// @param ticks		Number of ticks before the timer expires.
///
extern void delay_set_ticks(register uint8_t ticks);

/// Check the delay timer.
///
/// @return			True if the timer has expired.
///
extern bool	delay_expired(void);

/// Wait for a period of time to expire.
///
/// @note Maximum wait is ~2.5sec in the current implementation.
///
/// @param	msec		Minimum time to wait.  The actual time
///				may be greater.
///
extern void	delay_msec(register uint16_t msec);

/// return some entropy from timers
///
extern uint8_t timer_entropy(void);
