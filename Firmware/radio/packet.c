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
#include "timer.h"

#ifdef INCLUDE_AES
#include "AES/aes.h"
#endif

static __bit last_sent_is_resend;
static __bit last_sent_is_injected;
static __bit last_recv_is_resend;
static __bit force_resend;

static __xdata uint8_t last_received[MAX_PACKET_LENGTH];
static __xdata uint8_t last_sent[MAX_PACKET_LENGTH];
static __xdata uint8_t last_sent_len;
static __xdata uint8_t last_recv_len;

// serial speed in 16usecs/byte
static __pdata uint16_t serial_rate;

// the length of a pending MAVLink packet, or zero if no MAVLink
// packet is expected
static __pdata uint8_t mav_pkt_len;

// the timer2_tick time when the MAVLink header was seen
static __pdata uint16_t mav_pkt_start_time;

// the number of timer2 ticks this packet should take on the serial link
static __pdata uint16_t mav_pkt_max_time;

static __pdata uint8_t mav_max_xmit;

// true if we have a injected packet to send
static bool injected_packet;

// have we seen a mavlink packet?
uint8_t seen_mavlink;

#define PACKET_RESEND_THRESHOLD 32

// check if a buffer looks like a MAVLink heartbeat packet - this
// is used to determine if we will inject RADIO status MAVLink
// messages into the serial stream for ground station and aircraft
// monitoring of link quality
static void check_response(__xdata uint8_t * __pdata buf)
{
	if (buf[1] == 9 && buf[0] == MAVLINK10_STX && buf[5] == 0) {
		// looks like a MAVLink 1.x heartbeat
		// do not overwrite a protocol version request
		if (seen_mavlink < 3) {
			seen_mavlink = 1;
		}

	} else if (buf[1] <= 9 && buf[0] == MAVLINK20_STX && (buf[7] == 0) && buf[8] == 0 && buf[9] == 0) {
		// looks like a MAVLink 2.x heartbeat
		// do not overwrite a protocol version request
		if (seen_mavlink < 3) {
			seen_mavlink = 2;
		}

	} else if ((((buf[1] == 35 && buf[5] == 75) || (buf[1] == 33 && buf[5] == 76)) && buf[0] == MAVLINK10_STX && buf[28] == 0x07 && buf[29] == 0x20)) {
		// looks like a MAVLink 1.x command int/long requesting PROTOCOL_VERSION as part of the handshake:
		// https://mavlink.io/en/mavlink-version.html#version-handshaking
		seen_mavlink = 3;

	}
}

#define MSG_TYP_RC_OVERRIDE 70
#define MSG_LEN_RC_OVERRIDE (9 * 2)


#define MAVLINK_FRAMING_DISABLED 0
#define MAVLINK_FRAMING_SIMPLE 1
#define MAVLINK_FRAMING_HIGHPRI 2

// return a complete MAVLink frame, possibly expanding
// to include other complete frames that fit in the max_xmit limit
static 
uint8_t mavlink_frame(uint8_t max_xmit, __xdata uint8_t * __pdata buf)
{
	__data uint16_t slen;

	last_sent_len = 0;
	mav_pkt_len = 0;

	slen = serial_read_available();

	// see if we have more complete MAVLink frames in the serial
	// buffer that we can fit in this packet
	while (slen >= 8) {
		register uint8_t c = serial_peekx(0);
                register uint8_t extra_len = 8;
		if (c != MAVLINK10_STX && c != MAVLINK20_STX) {
			// its not a MAVLink packet
			return last_sent_len;			
		}
                if (c == MAVLINK20_STX) {
                        extra_len += 4;
                        if (serial_peekx(2) & 1) {
                                // signed packet
                                extra_len += 13;
                        }
                }
                // fetch the length byte
		c = serial_peekx(1);
		if (c >= 255 - extra_len || 
		    c+extra_len > max_xmit - last_sent_len) {
			// it won't fit
			break;
		}
		if (c+extra_len > slen) {
			// we don't have the full MAVLink packet in
			// the serial buffer
			break;
		}

                c += extra_len;

                // we can add another MAVLink frame to the packet
                serial_read_buf(&last_sent[last_sent_len], c);
                memcpy(&buf[last_sent_len], &last_sent[last_sent_len], c);
                
                check_response(buf+last_sent_len);
                        
		last_sent_len += c;
		slen -= c;
	}

	return last_sent_len;
}

#ifdef INCLUDE_AES
__xdata uint8_t len_encrypted;
#endif // INCLUDE_AES

uint8_t encryptReturn(__xdata uint8_t *buf_out, __xdata uint8_t *buf_in, uint8_t buf_in_len)
{
#ifdef INCLUDE_AES
  if (aes_get_encryption_level() > 0) {
    if (aes_encrypt(buf_in, buf_in_len, buf_out, &len_encrypted) != 0)
    {
      panic("error while trying to encrypt data");
    }
    return len_encrypted;
  }
#endif // INCLUDE_AES
  
  // if no encryption or not supported fall back to copy
  memcpy(buf_out, buf_in, buf_in_len);
  return buf_in_len;
}

// return the next packet to be sent
uint8_t
packet_get_next(register uint8_t max_xmit, __xdata uint8_t *buf)
{
	register uint16_t slen;

#ifdef INCLUDE_AES
  // Encryption takes 1 byte and is in multiples of 16.
  // 16, 32, 48 etc, lets not send anything above 32 bytes back
  // If you change this increase the buffer in serial.c serial_write_buf()
  if (aes_get_encryption_level() > 0) {
    if(max_xmit <= 16) return 0;
    if(max_xmit <= 32) max_xmit = 15;
    if(max_xmit > 31 ) max_xmit = 31;
  }
#endif // INCLUDE_AES
  
	if (injected_packet) {
		// send a previously injected packet
		slen = last_sent_len;

		// sending these injected packets at full size doesn't
		// seem to work well ... though I don't really know why!
		if (max_xmit > 32) {
   		    max_xmit = 32;
		}

		if (max_xmit < slen) {
			// send as much as we can
 		        last_sent_len = slen - max_xmit;
   		        slen = encryptReturn(buf, last_sent, max_xmit);

			memcpy(last_sent, &last_sent[max_xmit], last_sent_len);
			last_sent_is_injected = true;
			return slen;
		}
		// send the rest
		injected_packet = false;
		last_sent_is_injected = true;
		return encryptReturn(buf, last_sent, last_sent_len);
	}

	last_sent_is_injected = false;

	slen = serial_read_available();
	if (force_resend) {
		if (max_xmit < last_sent_len) {
			return 0;
		}
		last_sent_is_resend = true;
		force_resend = false;
		return encryptReturn(buf, last_sent, last_sent_len);
	}

	last_sent_is_resend = false;

	// if we have received something via serial see how
	// much of it we could fit in the transmit FIFO
	if (slen > max_xmit) {
		slen = max_xmit;
	}

	last_sent_len = 0;

	if (slen == 0) {
		// nothing available to send
		return 0;
	}

	if (!feature_mavlink_framing) {
		// simple framing
		if (slen > 0 && serial_read_buf(buf, slen)) {
			last_sent_len = slen;
      return encryptReturn(last_sent, buf, slen);
		}
    return 0;
	}

	// try to align packet boundaries with MAVLink packets

	if (mav_pkt_len == 1) {
		// we're waiting for the MAVLink length byte
		if (slen == 1) {
			if ((uint16_t)(timer2_tick() - mav_pkt_start_time) > mav_pkt_max_time) {
				// we didn't get the length byte in time
				last_sent[last_sent_len++] = serial_read(); // Send the STX
				mav_pkt_len = 0;
				return encryptReturn(buf, last_sent, last_sent_len);
			}
			// still waiting ....
			return 0;
		}
		// we have more than one byte, use normal packet frame
		// detection below
		mav_pkt_len = 0;
	}


	if (mav_pkt_len != 0) {
		if (slen < mav_pkt_len) {
			if ((uint16_t)(timer2_tick() - mav_pkt_start_time) > mav_pkt_max_time) {
				// timeout waiting for the rest of
				// it. Send what we have now.
				serial_read_buf(last_sent, slen);
				last_sent_len = slen;
				mav_pkt_len = 0;
				return encryptReturn(buf, last_sent, last_sent_len);
			}
			// leave it in the serial buffer till we have the
			// whole MAVLink packet			
			return 0;
		}
		
		// the whole of the MAVLink packet is available
		return mavlink_frame(max_xmit, buf);
	}

		// We are now looking for a new packet (mav_pkt_len == 0)
	while (slen > 0) {
		register uint8_t c = serial_peekx(0);
		if (c == MAVLINK10_STX || c == MAVLINK20_STX) {
			if (slen == 1) {
				// we got a bare MAVLink header byte
				if (last_sent_len == 0) {
					// wait for the next byte to
					// give us the length
					mav_pkt_len = 1;
					mav_pkt_start_time = timer2_tick();
					mav_pkt_max_time = serial_rate;
					return 0;
				}
				break;
			}
			mav_pkt_len = serial_peekx(1);
			if (mav_pkt_len >= 255-(8+4+13) ||
			    mav_pkt_len+(8+4+13) > mav_max_xmit) {
				// its too big for us to cope with
				mav_pkt_len = 0;
				last_sent[last_sent_len++] = serial_read(); // Send the STX and try again (we will lose framing)
				slen--;				
				continue;
			}

			// the length byte doesn't include
			// the header or CRC
			mav_pkt_len += 8;
                        if (c == MAVLINK20_STX) {
                                mav_pkt_len += 4;
                                if (slen > 2 && (serial_peekx(2) & 1)) {
                                        // it is signed
                                        mav_pkt_len += 13;
                                }
                        }
			
			if (last_sent_len != 0) {
				// send what we've got so far,
				// and send the MAVLink payload
				// in the next packet
				mav_pkt_start_time = timer2_tick();
				mav_pkt_max_time = mav_pkt_len * serial_rate;
				return encryptReturn(buf, last_sent, last_sent_len);
			} else if (mav_pkt_len > slen) {
				// the whole MAVLink packet isn't in
				// the serial buffer yet. 
				mav_pkt_start_time = timer2_tick();
				mav_pkt_max_time = mav_pkt_len * serial_rate;
				return 0;					
			} else {
// TODO FIX THIS FOR ENCRYPT
				// the whole packet is there
				// and ready to be read
				return mavlink_frame(max_xmit, buf);
			}
		} else {
			last_sent[last_sent_len++] = serial_read();
			slen--;
		}
	}
	return encryptReturn(buf, last_sent, last_sent_len);
}

// return true if the packet currently being sent
// is a resend
bool 
packet_is_resend(void)
{
	return last_sent_is_resend;
}

// return true if the packet currently being sent
// is an injected packet
bool 
packet_is_injected(void)
{
	return last_sent_is_injected;
}

// force the last packet to be resent. Used when transmit fails
void
packet_force_resend(void)
{
	force_resend = true;
}

// set the maximum size of a packet
void
packet_set_max_xmit(uint8_t max)
{
	mav_max_xmit = max;
}

// set the serial speed in bytes/s
void
packet_set_serial_speed(uint16_t speed)
{
	// convert to 16usec/byte to match timer2_tick()
	serial_rate = (65536UL / speed) + 1;
}

// determine if a received packet is a duplicate
bool 
packet_is_duplicate(uint8_t len, __xdata uint8_t *buf, bool is_resend)
{
	if (!is_resend) {
		memcpy(last_received, buf, len);
		last_recv_len = len;
		last_recv_is_resend = false;
		return false;
	}

	// We are now looking at a packet with the resend bit set
	if (last_recv_is_resend == false && 
			len == last_recv_len &&
			memcmp(last_received, buf, len) == 0) {
		last_recv_is_resend = false;  // FIXME - this has no effect
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

// inject a packet to send when possible
void 
packet_inject(__xdata uint8_t *buf, __pdata uint8_t len)
{
	if (len > sizeof(last_sent)) {
		len = sizeof(last_sent);
	}
	memcpy(last_sent, buf, len);
	last_sent_len = len;
	last_sent_is_resend = false;
	injected_packet = true;
}
