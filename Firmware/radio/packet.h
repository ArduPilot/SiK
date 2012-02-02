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


/// return the next packet to be sent
///
/// @param max_xmit		maximum bytes that can be sent
/// @param buf			buffer to put bytes in
///
/// @return			number of bytes to send
extern uint8_t packet_get_next(register uint8_t max_xmit, __xdata uint8_t * __pdata buf);

/// return true if the last packet was a resend
///
/// @return			true is a resend
extern bool packet_is_resend(void);

/// return true if the last packet was a injected packet
///
/// @return			true is injected
extern bool packet_is_injected(void);

/// determine if a received packet is a duplicate
///
/// @return			true if this is a duplicate
extern bool packet_is_duplicate(uint8_t len, __xdata uint8_t * __pdata buf, bool is_resend);

/// force the last packet to be re-sent. Used when packet transmit has
/// failed
extern void packet_force_resend(void);

/// set the maximum size of a packet
///
extern void packet_set_max_xmit(uint8_t max);

/// set the serial rate in bytes/s
///
/// @param  speed		serial speed bytes/s
///
extern void packet_set_serial_speed(uint16_t speed);

/// inject a packet to be sent when possible
/// @param buf			buffer to send
/// @param len			number of bytes
///			
extern void packet_inject(__xdata uint8_t * __pdata buf, __pdata uint8_t len);

