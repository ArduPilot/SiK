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

// packet buffer data pool
__xdata uint8_t		pbufData[PBUF_POOL_SIZE][PBUF_MAX_SIZE];

// pointers to the first and last pbuf in each queue.
__pdata PBufQueue	pbufQueues[PBUF_MAX_QUEUES];

// forward linkage pointers for each pbuf
__pdata PBufIndex	pbufNext[PBUF_POOL_SIZE];

void
pbuf_init(void)
{
	__pdata PBufQueueIndex	queue_index;
	__pdata PBufIndex	buffer_index;

	// initialise all the packet buffer queues to empty
	for (queue_index = 0; queue_index < PBUF_MAX_QUEUES; queue_index++) {
		pbufQueues[queue_index].head = PBUF_NULL;
		pbufQueues[queue_index].tail = PBUF_NULL;
	}

	// free all the packet buffers
	for (buffer_index = 0; buffer_index < PBUF_POOL_SIZE; buffer_index++)
		pbuf_free(buffer_index);
}

void
pbuf_copy_to_pbuf(PBufIndex idx, uint8_t ofs, __xdata uint8_t *ptr, PBufDataCount cnt)
{
	__xdata uint8_t		*b = pbuf_data_ptr(idx) + ofs;

	while (cnt--)
		*b++ = *ptr++;
}

void
pbuf_copy_from_pbuf(PBufIndex idx, uint8_t ofs, __xdata uint8_t *ptr, PBufDataCount cnt)
{
	__xdata uint8_t		*b = pbuf_data_ptr(idx) + ofs;

	while (cnt--)
		*ptr++ = *b++;
}

