// -*- Mode: C; c-basic-offset: 8; -*-
//
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
/// @file	pkt_buf.h
///
/// Packet buffers.
///

#ifndef _PKT_BUF_H_
#define _PKT_BUF_H_

#include "radio.h"

#define	PBUF_NULL	0xff		///< "no packet buffer" pointer
#define PBUF_MAX_SIZE	64		///< maximum buffer payload size
#define PBUF_POOL_SIZE	16		///< size of the packet buffer pool
#define PBUF_MAX_QUEUES	4		///< number of pbuf queues
#define PBUF_QUEUE_FREE	0		///< free packet buffers
#define PBUF_QUEUE_TX	1		///< packet buffers waiting to transmit
#define PBUF_QUEUE_RX	2		///< packet buffers received

typedef	uint8_t		PBufIndex;	///< packet buffer 'pointer', index into global buffer array(s)
typedef uint8_t		PBufQueueIndex;	///< packet buffer queue 'pointer'
typedef uint8_t		PBufDataCount;	///< count of bytes in a packet buffer

/// initialise the pbuf system
///
extern void		pbuf_init(void);

// pbuf queue manipulation
//
// Note that as the queues are singly linked, it is not possible to remove a buffer from the
// tail of a queue.
//
extern void		pbuf_queue_add_head(PBufQueueIndex qi, PBufIndex pi);
extern void		pbuf_queue_add_tail(PBufQueueIndex qi, PBufIndex pi);
extern PBufIndex	pbuf_queue_remove_head(PBufQueueIndex qi);
extern bool		pbuf_queue_empty(PBufQueueIndex qi);

// copy data from an arbitrary buffer to a pbuf
extern void		pbuf_copy_to_pbuf(PBufIndex idx, uint8_t ofs, __xdata uint8_t *ptr, PBufDataCount cnt);

// copy data from a pbuf to an arbitrary buffer
extern void 		pbuf_copy_from_pbuf(PBufIndex idx, uint8_t ofs, __xdata uint8_t *ptr, PBufDataCount cnt);

// returns a pointer in the data buffer for a given packet buffer
extern __xdata uint8_t	*pbuf_data_ptr(PBufIndex idx, PBufDataCount ofs);

// Packet buffer accessor macros.
//
// Use these macros and packet buffer indices rather than pointers to buffers
// to access fields in the packet buffer header.  This will make it easier to
// e.g. separate the fields at a later stage.
//
#define pbuf_getbuf()		pbuf_queue_remove_head(PBUF_QUEUE_FREE);
#define pbuf_putbuf(_idx)	pbuf_queue_add_head(PBUF_QUEUE_FREE, _idx);

#define	pbuf_size(_idx)		pbuf_pool[_idx].data_size
#define	pbuf_next(_idx)		pbuf_pool[_idx].next

struct pbuf {
	PBufIndex	next;			// next packet buffer in queue
	PBufDataCount	data_size;		// number of bytes in the packet
};
extern __xdata struct pbuf		pbuf_pool[];



#endif // PTK_BUF_H_
