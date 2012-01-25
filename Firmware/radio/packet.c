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
/// @file	packet.c
///
/// packet handling code
///

#include <stdarg.h>
#include "radio.h"
#include "packet.h"

static __bit last_sent_is_resend;
static __bit last_recv_is_resend;
static __bit force_resend;

static __xdata uint8_t last_received[64];
static __xdata uint8_t last_sent[64];
static __xdata uint8_t last_sent_len;
static __xdata uint8_t last_recv_len;

#define PACKET_RESEND_THRESHOLD 256

// return the next packet to be sent
uint8_t
packet_get_next(uint8_t max_xmit, __xdata uint8_t *buf)
{
	uint16_t slen = serial_read_available();

	if (force_resend ||
	    (last_sent_is_resend == 0 && last_sent_len != 0 && 
	     slen < PACKET_RESEND_THRESHOLD)) {
		if (max_xmit < last_sent_len) {
			return 0;
		}
		last_sent_is_resend = true;
		force_resend = false;
		memcpy(buf, last_sent, last_sent_len);
#if 0
		printf("RT(%u)[", (unsigned)last_sent_len);
		serial_write_buf(buf, last_sent_len);
		printf("]\r\n");
#endif
		return last_sent_len;
	}

	last_sent_is_resend = false;

	// if we have received something via serial see how
	// much of it we could fit in the transmit FIFO
	if (slen > max_xmit) {
		slen = max_xmit;
	}
	if (slen > 0 && serial_read_buf(buf, slen)) {
		memcpy(last_sent, buf, slen);
		last_sent_len = slen;
#if 0
		printf("NT(%u)[", (unsigned)last_sent_len);
		serial_write_buf(buf, last_sent_len);
		printf("]\r\n");
#endif
		return (uint8_t)slen;
	}
	last_sent_len = 0;
	return 0;
}

// return true if the packet currently being sent
// is a resend
bool 
packet_is_resend(void)
{
	return last_sent_is_resend;
}

// force the last packet to be resent. Used when transmit fails
void
packet_force_resend(void)
{
	force_resend = true;
}


// determine if a received packet is a duplicate
bool packet_is_duplicate(uint8_t len, __xdata uint8_t *buf, bool is_resend) __reentrant
{
	if (!is_resend) {
		memcpy(last_received, buf, len);
		last_recv_len = len;
		last_recv_is_resend = false;
		return false;
	}
	if (last_recv_is_resend == false && 
	    len == last_recv_len &&
	    memcmp(last_received, buf, len) == 0) {
		last_recv_is_resend = false;
		return true;
	}
#if 0
	printf("RS(%u,%u)[", (unsigned)len, (unsigned)last_recv_len);
	serial_write_buf(last_received, last_recv_len);
	serial_write_buf(buf, len);
	printf("]\r\n");
#endif
	last_recv_is_resend = true;
	return false;
}
