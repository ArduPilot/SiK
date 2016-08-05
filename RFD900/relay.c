
///
/// @file	relay.c
///
/// handle storage of relayed packets
///

#include <stdint.h>
#include <stdbool.h>
#include "radio_old.h"

#define RELAY_BUFF_MAX 8192
static uint8_t r_buf[RELAY_BUFF_MAX] = {0};

struct {
    uint8_t buf[256];
    uint8_t len;
    uint8_t src_nodeId;
} saved;

// FIFO insert/remove pointers
static volatile uint16_t r_insert, r_remove;

#define BUF_NEXT_INSERT(_b)	((_b##_insert + 1) == sizeof(_b##_buf)?0:(_b##_insert + 1))
#define BUF_NEXT_REMOVE(_b)	((_b##_remove + 1) == sizeof(_b##_buf)?0:(_b##_remove + 1))
#define BUF_FULL(_b)	(BUF_NEXT_INSERT(_b) == (_b##_remove))
#define BUF_NOT_FULL(_b)	(BUF_NEXT_INSERT(_b) != (_b##_remove))
#define BUF_EMPTY(_b)	(_b##_insert == _b##_remove)
#define BUF_NOT_EMPTY(_b)	(_b##_insert != _b##_remove)
#define BUF_USED(_b)	((_b##_insert >= _b##_remove)?(_b##_insert - _b##_remove):(sizeof(_b##_buf) - _b##_remove) + _b##_insert)
#define BUF_FREE(_b)	((_b##_insert >= _b##_remove)?(sizeof(_b##_buf) + _b##_remove - _b##_insert):(_b##_remove - _b##_insert)-1)

#define BUF_INSERT(_b, _c)	do { _b##_buf[_b##_insert] = (_c); \
		_b##_insert = BUF_NEXT_INSERT(_b); } while(0)
#define BUF_REMOVE(_b, _c)	do { (_c) = _b##_buf[_b##_remove]; \
		_b##_remove = BUF_NEXT_REMOVE(_b); } while(0)
#define BUF_PEEK(_b)	_b##_buf[_b##_remove]

/*
  store a packet that needs relaying
 */
void relay_store_packet(const uint8_t *p, uint8_t len, uint8_t src_nodeId)
{
    if (BUF_FREE(r) < len+2) {
        // no space
        return;
    }
    BUF_INSERT(r, len);
    BUF_INSERT(r, src_nodeId);
    while (len--) {
        BUF_INSERT(r, *p++);
    }
}

/*
  get a packet from relay store
 */
uint8_t relay_get_packet(uint8_t *p, uint8_t max_len, uint8_t *src_nodeId)
{
    uint8_t len, i;

    if (saved.len) {
        len = saved.len;
        *src_nodeId = saved.src_nodeId;
        memcpy(p, saved.buf, len);
        saved.len = 0;
        return len;
    }

    if (BUF_EMPTY(r)) {
        // no packet
        return 0;
    }
    len = BUF_PEEK(r);
    if (len > max_len) {
        return 0;
    }
    BUF_REMOVE(r, len);
    BUF_REMOVE(r, *src_nodeId);
    for (i=0; i<len; i++) {
        BUF_REMOVE(r, *p);
        p++;
    }
    return len;
}

/*
  return number of bytes pending to be relayed. Includes length bytes
 */
uint16_t relay_bytes_pending(void)
{
    return BUF_USED(r);
}

/*
  save a relayed packet that has failed to transmit
 */
void relay_save_pkt(const uint8_t *p, uint8_t len, uint8_t src_nodeId)
{
    saved.len = len;
    saved.src_nodeId = src_nodeId;
    memcpy(saved.buf, p, len);
}
