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
/// @file	pkt_buf.c
///
/// Packet buffers for in/outbound radio packets
///

#include "pkt_buf.h"

/// packet buffer pool
/// @todo could be pdata?
///
__xdata struct pbuf		pbuf_pool[PBUF_POOL_SIZE];

/// packet data pool
///
__xdata uint8_t		pbuf_data[PBUF_POOL_SIZE][PBUF_MAX_SIZE];

/// packet buffer queue
///
struct pbuf_queue {
	PBufIndex	head;
	PBufIndex	tail;
};

/// packet buffer queues
///
__xdata struct pbuf_queue	pbuf_queues[PBUF_MAX_QUEUES];


void
pbuf_init(void)
{
	PBufQueueIndex	queue_index;
	PBufIndex		buffer_index;

	// initialise all the packet buffer queues to empty
	for (queue_index = 0; queue_index < PBUF_MAX_QUEUES; queue_index++) {
		pbuf_queues[queue_index].head = PBUF_NULL;
		pbuf_queues[queue_index].tail = PBUF_NULL;
	}

	// free all the packet buffers
	for (buffer_index = 0; buffer_index < PBUF_POOL_SIZE; buffer_index++)
		pbuf_putbuf(buffer_index);
}

void
pbuf_queue_add_head(PBufQueueIndex queue_index, PBufIndex buffer_index)
{
	__xdata struct pbuf_queue *pq = &pbuf_queues[queue_index];
	bool		istate;
	PBufIndex	hi;

	interrupt_disable(istate);

	// get the next buffer in the queue
	hi = pq->head;

	// make this buffer the head of the queue
	pq->head = buffer_index;

	// if the queue was empty, point the tail to this buffer as well
	if (hi == PBUF_NULL)
		pq->tail = buffer_index;

	// link this buffer to the next
	pbuf_next(buffer_index) = hi;

	interrupt_restore(istate);
}

void
pbuf_queue_add_tail(PBufQueueIndex queue_index, PBufIndex buffer_index)
{
	__xdata struct pbuf_queue *pq = &pbuf_queues[queue_index];
	bool		istate;
	PBufIndex	ti;

	interrupt_disable(istate);

	// get the last buffer in the queue
	ti = pq->head;

	// make this buffer the last one
	pq->tail = buffer_index;

	// if the queue is not empty
	if (ti != PBUF_NULL) {
		// link the last buffer to this one
		pbuf_next(ti) = buffer_index;
	} else {
		// queue was empty, new buffer is also the head
		pq->head = buffer_index;
	}

	interrupt_restore(istate);
}

static void
pbuf_queue_remove(PBufQueueIndex queue_index, PBufIndex previous_index, PBufIndex buffer_index)
{
	__xdata struct pbuf_queue *pq = &pbuf_queues[queue_index];
	__xdata struct pbuf *pb = &pbuf_pool[buffer_index];
	__xdata struct pbuf *pp = &pbuf_pool[previous_index];

	if (previous_index == PBUF_NULL) {
		pq->head = pb->next;
	} else {
		pp->next = pb->next;
	}
	if (pq->tail == buffer_index)
		pq->tail = previous_index;
	pb->next = PBUF_NULL;
}

PBufIndex
pbuf_queue_remove_head(PBufQueueIndex queue_index)
{
	bool		istate;
	PBufIndex	hi;

	interrupt_disable(istate);

	// get the first buffer
	hi = pbuf_queues[queue_index].head;
	if (hi != PBUF_NULL)
		pbuf_queue_remove(queue_index, PBUF_NULL, hi);

	interrupt_restore(istate);

	return hi;
}

bool
pbuf_queue_empty(PBufQueueIndex queue_index)
{
	if (pbuf_queues[queue_index].head == PBUF_NULL)
		return true;
	return false;
}

void
pbuf_copy_to_pbuf(PBufIndex idx, uint8_t ofs, __xdata uint8_t *ptr, PBufDataCount cnt)
{
	__xdata uint8_t		*b = pbuf_data_ptr(idx, ofs);

	while (cnt--)
		*b++ = *ptr++;
}

void
pbuf_copy_from_pbuf(PBufIndex idx, uint8_t ofs, __xdata uint8_t *ptr, PBufDataCount cnt)
{
	__xdata uint8_t		*b = pbuf_data_ptr(idx, ofs);

	while (cnt--)
		*ptr++ = *b++;
}

__xdata uint8_t *
pbuf_data_ptr(PBufIndex idx, PBufDataCount ofs)
{
	return &pbuf_data[idx][ofs];
}
