/*
  store a packet that needs relaying
 */
extern void relay_store_packet(const uint8_t *p, uint8_t len, uint8_t src_nodeId);

/*
  get a packet from relay store
 */
extern uint8_t relay_get_packet(uint8_t *p, uint8_t max_len, uint8_t *src_nodeId);

/*
  return number of bytes pending to be relayed. Includes length bytes
 */
extern uint16_t relay_bytes_pending(void);
