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

#include <stdint.h>
#include <stdarg.h>
#include "radio_old.h"
#include "packet.h"
#include "timer.h"
#include "serial.h"
#include "printfl.h"
#include "aes.h"

#define MAVPKTOFFSET 625
static bool last_sent_is_resend;
static bool last_sent_is_injected;
static bool last_recv_is_resend;
static bool force_resend;

static uint8_t last_received[MAX_PACKET_LENGTH];
static uint8_t last_sent[(MAX_PACKET_LENGTH*3)/2];
static uint16_t last_sent_len;
static uint8_t last_recv_len;

// serial speed in 16usecs/byte
static uint16_t serial_rate;

// the length of a pending MAVLink packet, or zero if no MAVLink
// packet is expected
static uint8_t mav_pkt_len;

// the timer2_tick time when the MAVLink header was seen
static uint16_t mav_pkt_start_time;

// the number of timer2 ticks this packet should take on the serial link
static uint16_t mav_pkt_max_time;

static uint8_t mav_max_xmit;

static uint8_t mav_pkt_sent;

// true if we have a injected packet to send
static bool injected_packet;

// have we seen a mavlink packet?
bool seen_mavlink;

#define PACKET_RESEND_THRESHOLD 32

// check if a buffer looks like a MAVLink heartbeat packet - this
// is used to determine if we will inject RADIO status MAVLink
// messages into the serial stream for ground station and aircraft
// monitoring of link quality
static void check_heartbeat(uint8_t * buf)
{
    if ((buf[1] == 9 && buf[0] == MAVLINK10_STX && buf[5] == 0) ||
            (buf[1] <= 9 && buf[0] == MAVLINK20_STX && buf[7] == 0 && buf[8] == 0 && buf[9] == 0)) {
        // looks like a MAVLink heartbeat
        seen_mavlink = true;
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
uint8_t mavlink_frame(uint8_t max_xmit, uint8_t * buf)
{
    uint16_t slen;

    //
    // There is already a packet sitting waiting here
    //
    // but this optimization is redundant with the loop below.  By letting the very slightly
    // more expensive version its thing we can ensure we skip _all_ redundant rc_override msgs
#if 0
    serial_read_buf(last_sent, mav_pkt_len);
    last_sent_len = mav_pkt_len;
    memcpy(buf, last_sent, last_sent_len);
    check_heartbeat(buf);
#else
    last_sent_len = 0;
#endif
    mav_pkt_len = 0;
    slen = serial_read_available();

    // see if we have more complete MAVLink frames in the serial
    // buffer that we can fit in this packet
    while (slen >= 8) {
        register uint8_t c = serial_peekx(0);
        register uint8_t extra_len = 8;
        register uint8_t msgid = serial_peekx(5);
        if (c != MAVLINK10_STX && c != MAVLINK20_STX) {
            // its not a MAVLink packet
            return last_sent_len;
        }
        if (c == MAVLINK20_STX) {
            msgid = serial_peekx(7);
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
        check_heartbeat(buf+last_sent_len);

        mav_pkt_sent++;

        last_sent_len += c;
        slen -= c;
    }

    return last_sent_len;
}


static uint16_t encryptReturn(uint8_t *buf_out, uint8_t *buf_in, uint16_t buf_in_len, uint8_t SeqNo)
{
    uint16_t len_encrypted;
    if (aes_get_encryption_level() > 0) {
        if (aes_encrypt(buf_in, buf_in_len, buf_out, &len_encrypted,SeqNo) != 0)
        {
            panic("error while trying to encrypt data");
        }
        return len_encrypted;
    }
    // if no encryption or not supported fall back to copy
    memcpy(buf_out, buf_in, buf_in_len);
    return buf_in_len;
}


// return the next packet to be sent
uint8_t packet_get_next(register uint8_t max_xmit, uint8_t * buf, uint8_t SeqNo)
{
    register uint16_t slen;

    // Encryption takes 1 byte extra and is in multiples of 16.
    // buffer size for encryption is (len&f0+16)
    // buffer 0-15  max size is 0
    // buffer 16-31 max size is 15
    // buffer 32-47 max size is 31
    if (aes_get_encryption_level() > 0)
    {
        max_xmit = (max_xmit&0xf0);
        if(0 == max_xmit) return 0;
        max_xmit--;
    }

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
            slen = encryptReturn(buf, last_sent, max_xmit,SeqNo);

            memcpy(last_sent, &last_sent[max_xmit], last_sent_len);
            last_sent_is_injected = true;
            return slen;
        }
        // send the rest
        injected_packet = false;
        last_sent_is_injected = true;
        return encryptReturn(buf, last_sent, last_sent_len,SeqNo);
    }

    last_sent_is_injected = false;

    slen = serial_read_available();
    if (force_resend) {
        if (max_xmit < last_sent_len) {
            return 0;
        }
        last_sent_is_resend = true;
        force_resend = false;
        return encryptReturn(buf, last_sent, last_sent_len,SeqNo);
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
            memcpy(last_sent,buf,slen);
            last_sent_len = slen;
            return encryptReturn(buf,last_sent, slen,SeqNo);
        }
        else
        {
            last_sent_len = 0;
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
                return encryptReturn(buf, last_sent, last_sent_len,SeqNo);
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
                return encryptReturn(buf, last_sent, last_sent_len,SeqNo);
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
                    mav_pkt_max_time = serial_rate + MAVPKTOFFSET;
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
                mav_pkt_max_time = MAVPKTOFFSET + (mav_pkt_len * serial_rate);
                return encryptReturn(buf, last_sent, last_sent_len,SeqNo);
            } else if (mav_pkt_len > slen) {
                // the whole MAVLink packet isn't in
                // the serial buffer yet.
                mav_pkt_start_time = timer2_tick();
                mav_pkt_max_time = MAVPKTOFFSET+ (mav_pkt_len * serial_rate);
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

    return encryptReturn(buf, last_sent, last_sent_len,SeqNo);
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
packet_set_serial_speed(uint16_t speedbps)
{
    // convert to 16usec/byte to match timer2_tick()
    // ft2 = 62500, 10 bits per byte on serial
    speedbps = speedbps/10;
    serial_rate = ((62500UL+(speedbps>>1))/speedbps) + 5;
}

// determine if a received packet is a duplicate
bool
packet_is_duplicate(uint8_t len, uint8_t * buf, bool is_resend)
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
packet_inject(uint8_t * buf, uint16_t len)
{
    if (len > sizeof(last_sent)) {
        len = sizeof(last_sent);
    }
    memcpy(last_sent, buf, len);
    last_sent_len = len;
    last_sent_is_resend = false;
    injected_packet = true;
}
