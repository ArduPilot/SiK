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
typedef __xdata uint8_t *PBufDataPtr;	///< pointer to pbuf data

/// packet buffer queue
///
typedef struct {
	PBufIndex	head;
	PBufIndex	tail;
} PBufQueue;

/// packet buffer data pool
///
extern __xdata uint8_t		pbufData[PBUF_POOL_SIZE][PBUF_MAX_SIZE];

/// Pointers to the first and last pbuf in each queue.
///
extern __pdata PBufQueue	pbufQueues[PBUF_MAX_QUEUES];

/// Forward linkage pointers for each pbuf
///
extern __pdata PBufIndex	pbufNext[PBUF_POOL_SIZE];

/// initialise the pbuf system
///
extern void		pbuf_init(void);

// pbuf queue manipulation
//
// Note that as the queues are singly linked, it is not possible to remove a buffer from the
// tail of a queue.
//
// XXX TBD: these are inlines to avoid making function calls when they're used in
// interrupt context.  Should we have non-inline versions for other code's use?

/// Add a pbuf to the head of a queue
///
/// @param _queue_index		The queue to add the packet to.
/// @param _packet_index	The packet to add.
///
static inline void
pbuf_queue_add_head(PBufQueueIndex qi, PBufIndex pi)
__critical {
	PBufIndex	ni;

	ni = pbufQueues[qi].head;
	pbufNext[pi] = ni;
	pbufQueues[qi].head = pi;
	if (ni == PBUF_NULL)
		pbufQueues[qi].tail = pi;
}

/// Add a pbuf to the tail of a queue
///
/// @param _queue_index		The queue to add the packet to.
/// @param _packet_index	The packet to add.
///
static inline void
pbuf_queue_add_tail(PBufQueueIndex qi, PBufIndex pi)
__critical {
	PBufIndex	li;

	pbufNext[pi] = PBUF_NULL;
	li = pbufQueues[qi].tail;
	pbufQueues[qi].tail = pi;
	if (li == PBUF_NULL) {
		pbufQueues[qi].head = pi;
	} else {
		pbufNext[li] = pi;
	}
}

/// Remove a pbuf from the head of a queue
///
/// @param _queue_index		The queue to remove a packet from
/// @param _head_index		Returns the packet index that was removed,
///				or PBUF_NULL if the queue was empty.
///
static inline PBufIndex
pbuf_queue_remove_head(PBufQueueIndex qi)
__critical {
	PBufIndex	hi, ni;

	hi = pbufQueues[qi].head;
	if (hi != PBUF_NULL) {
		ni = pbufNext[hi];
		pbufQueues[qi].head = ni;
		if (ni == PBUF_NULL)
			pbufQueues[qi].tail = ni;
	}
	return hi;
}

/// Test whether a queue is empty.
///
static inline bool
pbuf_queue_empty(PBufQueueIndex qi)
{
	return pbufQueues[qi].head == PBUF_NULL;
}

/// Return a pointer to a pbuf's data buffer
///
static inline PBufDataPtr
pbuf_data_ptr(PBufIndex pi)
{
	return &pbufData[pi][0];
}

/// Return the next pbuf in a chain
///
static inline PBufIndex
pbuf_next(PBufIndex pi)
{
	return pbufNext[pi];
}

/// Free a pbuf
///
static inline void
pbuf_free(PBufIndex pi)
{
	pbuf_queue_add_head(PBUF_QUEUE_FREE, pi);
}

/// Get a pbuf
///
static inline PBufIndex
pbuf_alloc(void)
{
	return pbuf_queue_remove_head(PBUF_QUEUE_FREE);
}

extern void	pbuf_copy_to_pbuf(PBufIndex idx, uint8_t ofs, __xdata uint8_t *ptr, PBufDataCount cnt);
extern void	pbuf_copy_from_pbuf(PBufIndex idx, uint8_t ofs, __xdata uint8_t *ptr, PBufDataCount cnt);

#endif // PTK_BUF_H_
