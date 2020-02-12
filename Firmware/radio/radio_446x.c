// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2019 Martin Poviser, All Rights Reserved
// Copyright (c) 2011 Michael Smith, All Rights Reserved
// Copyright (c) 2011 Andrew Tridgell, All Rights Reserved
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

//#define DEBUG 1
//#define INCLUDE_GOLAY

#include "board.h"
#ifdef CPU_SI1060

/* radio property groups */
#define GROUP_GLOBAL 0x00
#define GROUP_INT_CTL 0x01
#define GROUP_FRR_CTL 0x02
#define GROUP_PREAMBLE 0x10
#define GROUP_SYNC 0x11
#define GROUP_PKT 0x12
#define GROUP_MODEM 0x20
#define GROUP_PA 0x22
#define GROUP_SYNTH 0x23
#define GROUP_MATCH 0x30
#define GROUP_FREQ_CONTROL 0x40
#define GROUP_RX_HOP 0x50
#define GROUP_PTI 0xf0

/* radio states */
#define STATE_READY		3
#define STATE_TX		7
#define STATE_RX		8

/* radio interrupt flags */
#define PH_STATUS_FILTER_MATCH			(1<<7)
#define PH_STATUS_FILTER_MISS			(1<<6)
#define PH_STATUS_PACKET_SENT			(1<<5)
#define	PH_STATUS_PACKET_RX				(1<<4)
#define PH_STATUS_CRC_ERROR				(1<<3)
#define PH_STATUS_PREAMBLE_DETECTED		(1<<2)
#define PH_STATUS_TX_FIFO_ALMOST_EMPTY  (1<<1)
#define PH_STATUS_RX_FIFO_ALMOST_FULL 	(1<<0)

#define MODEM_STATUS_RSSI_LATCH			(1<<7)
#define MODEM_STATUS_POSTAMBLE_DETECT	(1<<6)
#define MODEM_STATUS_INVALID_SYNC		(1<<5)
#define MODEM_STATUS_RSSI_JUMP			(1<<4)
#define MODEM_STATUS_RSSI				(1<<3)
#define MODEM_STATUS_INVALID_PREAMBLE	(1<<2)
#define MODEM_STATUS_PREAMBLE_DETECT	(1<<1)
#define MODEM_STATUS_SYNC_DETECT		(1<<0)

#define CHIP_STATUS_FIFO_UNDERFLOW_OVERFLOW_ERROR	(1<<5)
#define CHIP_STATUS_STATE_CHANGE					(1<<4)
#define CHIP_STATUS_CMD_ERROR						(1<<3)

#include "radio.h"
#include "timer.h"
#include "golay.h"
#include "crc.h"
#include "pins_user.h"

#include "radio_446x_api.h"

__xdata uint8_t radio_buffer[MAX_PACKET_LENGTH];
__pdata uint8_t receive_packet_length;
__pdata uint8_t partial_packet_length;
__pdata uint8_t last_rssi;
__pdata uint8_t netid[2];

__pdata uint8_t _skip;

/* values saved for RX_HOP command */
__xdata uint8_t outdiv;
__xdata uint32_t freq_control_base;
__xdata uint16_t freq_control_spacing;

#define XO_FREQ 	30000000

static volatile __bit packet_received;
static volatile __bit sync_detected;

__pdata struct radio_settings settings;

// save and restore radio interrupt. We use this rather than
// __critical to ensure we don't disturb the timer interrupt at all.
// minimal tick drift is critical for TDM
#define EX0_SAVE_DISABLE __bit EX0_saved = EX0; EX0 = 0
#define EX0_RESTORE EX0 = EX0_saved

static bool	software_reset(void);
static void	set_frequency_registers(__pdata uint32_t base_freq, __pdata uint32_t freq_spacing);

#define TX_FIFO_THRESHOLD 0x40
#define RX_FIFO_THRESHOLD 0x40

#define RX_INT0_PH_MASK 	(PH_STATUS_FILTER_MISS \
							| PH_STATUS_PACKET_RX \
							| PH_STATUS_CRC_ERROR \
							| PH_STATUS_RX_FIFO_ALMOST_FULL)
#define RX_INT0_MODEM_MASK	MODEM_STATUS_SYNC_DETECT


static void
_radio_receiver_on(void) __reentrant;

static void
inc_tx_error(void) __reentrant
{
	if (errors.tx_errors != 0xffff)
		errors.tx_errors++;
}

static void
inc_rx_error(void) __reentrant
{
	if (errors.rx_errors != 0xffff)
		errors.rx_errors++;
}

static void
clear_rx_fifo(void) __reentrant
{
	cmd_fifo_info(0x02);
	wait_for_cts();
}

bool
radio_receive_packet(uint8_t *length, __xdata uint8_t * __pdata buf)
{
#ifdef INCLUDE_GOLAY
	__xdata uint8_t gout[3];
	__data uint16_t crc1, crc2;
	__data uint8_t errcount = 0;
	__data uint8_t elen;
#endif

	if (!packet_received)
		return false;

	if (receive_packet_length > MAX_PACKET_LENGTH
		|| receive_packet_length < 3) {
		EX0=0;
		_radio_receiver_on();
		EX0=1;
		goto failed;
	}


#ifdef INCLUDE_GOLAY
	if (!feature_golay)
#endif // INCLUDE_GOLAY
  {
	*length = receive_packet_length-3;
	memcpy(buf, radio_buffer+3, receive_packet_length-3);

		EX0=0;
		_radio_receiver_on();
		EX0=1;


		//debug("packet received len=%d chan=%d", *length, settings.current_channel);

		return true;
	}

#ifdef INCLUDE_GOLAY
	// decode it in the callers buffer. This relies on the
	// in-place decode properties of the golay code. Decoding in
	// this way allows us to overlap decoding with the next receive
	memcpy(buf, radio_buffer+1, receive_packet_length-1);

	// enable the receiver for the next packet. This also
	// enables the EX0 interrupt
	elen = receive_packet_length-1;
	EX0=0;
	_radio_receiver_on();
	EX0=1;

	if (elen < 12 || (elen%6) != 0) {
		// not a valid length
		debug("rx len invalid %u\n", (unsigned)elen);
		goto failed;
	}

	// decode the header
	errcount = golay_decode(6, buf, gout);
	if (gout[0] != netid[0] ||
	    gout[1] != netid[1]) {
		// its not for our network ID 
		debug("netid %x %x\n",
		       (unsigned)gout[0],
		       (unsigned)gout[1]);
		goto failed;
	}

	if (6*((gout[2]+2)/3+2) != elen) {
		debug("rx len mismatch1 %u %u\n",
		       (unsigned)gout[2],
		       (unsigned)elen);		
		goto failed;
	}

	// decode the CRC
	errcount += golay_decode(6, &buf[6], gout);
	crc1 = gout[0] | (((uint16_t)gout[1])<<8);

	if (elen != 12) {
		errcount += golay_decode(elen-12, &buf[12], buf);
	}

	*length = gout[2];

	crc2 = crc16(*length, buf);

	if (crc1 != crc2) {
		debug("crc1=%x crc2=%x len=%u [%x %x]\n",
		       (unsigned)crc1, 
		       (unsigned)crc2, 
		       (unsigned)*length,
		       (unsigned)buf[0],
		       (unsigned)buf[1]);
		goto failed;
	}

	if (errcount != 0) {
		if ((uint16_t)(0xFFFF - errcount) > errors.corrected_errors) {
			errors.corrected_errors += errcount;
		} else {
			errors.corrected_errors = 0xFFFF;
		}
		if (errors.corrected_packets != 0xFFFF) {
			errors.corrected_packets++;
		}
	}

  return true;
#endif // INCLUDE_GOLAY

failed:
	inc_rx_error();
	return false;
}

bool
radio_receive_in_progress(void)
{
	return sync_detected;
}

bool
radio_preamble_detected(void)
{
	/* we return whether sync, not preamble, has been detected */
	return sync_detected;
}

uint8_t
radio_last_rssi(void)
{
	return last_rssi;
}

uint8_t
radio_current_rssi(void)
{
	register uint8_t ret;

	EX0_SAVE_DISABLE;
	cmd_get_modem_status(0xff);
	get_modem_status_reply(
		_skip, _skip, ret, _skip, _skip, _skip, _skip
	);
	EX0_RESTORE;

	return ret;
}

uint8_t
radio_air_rate(void)
{
	return settings.air_data_rate;
}

static bool
in_rx_mode(void) __reentrant
{
	uint8_t state;
	EX0_SAVE_DISABLE;
	frr_d_read(state);
	if (state == STATE_RX) {
		EX0_RESTORE;
		return true;
	}
	EX0_RESTORE;
	return false;
}

static bool
radio_transmit_simple(uint8_t length, __xdata uint8_t * __pdata buf, __pdata uint16_t timeout_ticks, bool insert_netid)
{
	__pdata uint16_t tstart;
	__data uint8_t n, len_remaining;
	uint8_t chip_status, ph_status, tx_space;

	if (length > sizeof(radio_buffer))
		panic("oversized packet");

	tstart = timer2_tick();

	EX0_SAVE_DISABLE;
	/* exit from RX state */
	cmd_change_state(STATE_READY);
	wait_for_cts();

	EX0_RESTORE;

	cmd_get_int_status_clear_all();
	wait_for_cts();

	cmd_fifo_info(0x03); /* clear the TX FIFO */
	wait_for_cts();

	if (insert_netid) {
		radio_buffer[0] = netid[1];
		radio_buffer[1] = netid[0];
		radio_buffer[2] = length;
		write_tx_fifo(3, radio_buffer);
	} else {
		//__xdata uint8_t length_xdata;
		//radio_buffer[0] = length;
		write_tx_fifo(1, &length);
	}

	n = length;
	if (n > 32)
		n = 32;
	write_tx_fifo(n, buf);
	len_remaining = length - n;
	buf += n;

	if (insert_netid) {
		cmd_start_tx(settings.current_channel, 0x30, length+3, 0, 0);
	} else {
		cmd_start_tx(settings.current_channel, 0x30, length+1, 0, 0);
	}
	wait_for_cts();

	while ((uint16_t) (timer2_tick() - tstart) < timeout_ticks) {
		///* Fast Read Registers contain CHIP_STATUS and PH_STATUS */
		cmd_fifo_info(0x00);
		fifo_info_reply(_skip, tx_space);
		frr_bc_read(chip_status, ph_status);

		// looking at PH_STATUS_TX_FIFO_ALMOST_EMPTY proved unreliable
		if (//(ph_status & PH_STATUS_TX_FIFO_ALMOST_EMPTY)
			(tx_space >= TX_FIFO_THRESHOLD)
				&& len_remaining != 0) {
			n = len_remaining;
			if (n > TX_FIFO_THRESHOLD)
				n = TX_FIFO_THRESHOLD;
			write_tx_fifo(n, buf);
			len_remaining -= n; buf += n;
		}

		if (chip_status & (CHIP_STATUS_FIFO_UNDERFLOW_OVERFLOW_ERROR \
							  | CHIP_STATUS_CMD_ERROR)) {
			debug("tx error: chip_status=%x len=%u len_remaining=%u", chip_status, length, len_remaining);
			goto error;
		}

		if (ph_status & PH_STATUS_PACKET_SENT) {
			return true;
		}
	}

	debug("TX timeout %u ts=%u tn=%u len=%u",
		timeout_ticks,
		tstart,
		timer2_tick(),
		(unsigned)length);

error:
	/* exit from TX state */
	cmd_change_state(STATE_READY);
	wait_for_cts();

	/* here we assume we are out of TX state, so that the FIFO reset is safe to do */
	cmd_fifo_info(0x03);
	wait_for_cts();

	inc_tx_error();
	return false;
}

#ifdef INCLUDE_GOLAY
// start transmitting a packet from the transmit FIFO
//
// @param length		number of data bytes to send
// @param timeout_ticks		number of 16usec RTC ticks to allow
//				for the send
//
// @return	    true if packet sent successfully
//
static bool
radio_transmit_golay(uint8_t length, __xdata uint8_t * __pdata buf, __pdata uint16_t timeout_ticks)
{
	__pdata uint16_t crc;
	__xdata uint8_t gin[3];
	__pdata uint8_t elen, rlen;

	if (length > (sizeof(radio_buffer)/2)-6) {
		debug("golay packet size %u\n", (unsigned)length);
		panic("oversized golay packet");		
	}

	// rounded length
	rlen = ((length+2)/3)*3;

	// encoded length
	elen = (rlen+6)*2;

	// start of packet is network ID and packet length
	gin[0] = netid[0];
	gin[1] = netid[1];
	gin[2] = length;

	// golay encode the header
	golay_encode(3, gin, radio_buffer);

	// next add a CRC, we round to 3 bytes for simplicity, adding 
	// another copy of the length in the spare byte
	crc = crc16(length, buf);
	gin[0] = crc&0xFF;
	gin[1] = crc>>8;
	gin[2] = length;

	// golay encode the CRC
	golay_encode(3, gin, &radio_buffer[6]);

	// encode the rest of the payload
	golay_encode(rlen, buf, &radio_buffer[12]);

	return radio_transmit_simple(elen, radio_buffer, timeout_ticks, false);
}
#endif // INCLUDE_GOLAY

// start transmitting a packet from the transmit FIFO
//
// @param length		number of data bytes to send
// @param timeout_ticks		number of 16usec RTC ticks to allow
//				for the send
//
// @return	    true if packet sent successfully
//
bool
radio_transmit(uint8_t length, __xdata uint8_t * __pdata buf, __pdata uint16_t timeout_ticks)
{
	bool ret;

	EX0_SAVE_DISABLE;

#ifdef INCLUDE_GOLAYq
	if (!feature_golay) {
		ret = radio_transmit_simple(length, buf, timeout_ticks, true);
	} else {
		ret = radio_transmit_golay(length, buf, timeout_ticks);
	}
#else
  ret = radio_transmit_simple(length, buf, timeout_ticks, true);
#endif // INCLUDE_GOLAY

	EX0_RESTORE;
	return ret;
}

static void
_radio_receiver_on(void) __reentrant
{
	packet_received = 0;
	receive_packet_length = 0;
	sync_detected = 0;
	partial_packet_length = 0;

	cmd_get_int_status_clear_all();
	wait_for_cts();

	clear_rx_fifo();

	cmd_start_rx(
		settings.current_channel,
		0,
		0,
		0, /* no timeout when looking for preamble */
		STATE_READY, /* go to READY state after reception of valid packet */
		STATE_READY  /* ditto after invalid packet */
	);
	wait_for_cts();
}

// put the radio in receive mode
//
bool
radio_receiver_on(void)
{
	if (in_rx_mode())
		return true;

	EX0 = 0;
	_radio_receiver_on();
	EX0 = 1;

	return true;
}

static bool
check_part(void) __reentrant
{
	uint8_t chiprev, romid;
	uint16_t part, id;

	cmd_part_info();
	part_info_reply(chiprev, part, _skip, id, _skip, romid);
	
	debug("radio part info: chiprev=%x part=%x id=%x romid=%x",
		  (unsigned) chiprev, (unsigned) part, (unsigned) id, (unsigned) romid);
	
	return part == 0x4463;
}

// initialise the radio hardware
//
bool
radio_initialise(void)
{
	/* TODO: check timing */
	SDN = 1;
	delay_msec(1);
	SDN = 0;
	delay_msec(6);

	wait_for_cts();

	cmd_power_up(0x1, 0x0, 30000000);
	wait_for_cts();

	if (!check_part())
		return false;

	cmd_set_property4(GROUP_FRR_CTL, 0x0,
		10, /* FRR A = latched RSSI */
		7, /* FRR B = INT_CHIP_STATUS */
		3, /* FRR C = INT_PH_STATUS */
		9 /* FRR D = CURRENT_STATE */
	);
	wait_for_cts();

	cmd_set_property4(GROUP_INT_CTL, 0x0,
		0x03, /* MODEM_INT_STATUS_EN | PH_INT_STATUS_EN */
		RX_INT0_PH_MASK,
		RX_INT0_MODEM_MASK,
		0x00
	);
	wait_for_cts();

	/* GLOBAL_XO_TUNE */
	cmd_set_property1(GROUP_GLOBAL, 0x00, EZRADIOPRO_OSC_CAP_VALUE);
	wait_for_cts();

	/* GLOBAL_CONFIG: fast switch to TX, shared FIFO mode */
	cmd_set_property1(GROUP_GLOBAL, 0x03, 0x10);
	wait_for_cts();

	cmd_set_property1(GROUP_GLOBAL, 0x01, 0x48);
	wait_for_cts();

	cmd_gpio_pin_cfg(
		0x40 | 32, // 0: TX_STATE
		0x40 | 35, // 1: TX_FIFO_EMPTY
		//33, // 1: RX_STATE
		0x40 | 8, // 2: CTS
		0x40 | 33, // GPIO4: RX state
		39, // nIRQ
		0, // SDO
		2
	);
	wait_for_cts();

	return true;
}

bool
radio_set_frequency(__pdata uint32_t base, __pdata uint32_t spacing)
{
	if (base < 240000000UL || base > 935000000UL) {
		return false;
	}
	settings.frequency = base;
	settings.channel_spacing = spacing;
	set_frequency_registers(base, spacing);
	return true;
}

static void
rx_hop(uint8_t channel)
{
	uint16_t vco_cnt;

	EX0_SAVE_DISABLE;
	uint32_t fc = (freq_control_base + channel*((uint32_t) freq_control_spacing));
	/* formula for VCO_CNT taken from Silabs' spreadsheet calculator */
	vco_cnt = (uint16_t) (((fc<<5) - (((uint32_t) outdiv)<<17) \
						  + (((uint32_t) 1)<<18)) >> 19);

	cmd_rx_hop(
		(fc>>19) - 1,
		(fc & 0x7ffff) | 0x80000,
		vco_cnt
	);
	wait_for_cts();
	EX0_RESTORE;
}

// set the tx/rx frequency channel
//
void
radio_set_channel(uint8_t channel)
{
	if (channel != settings.current_channel) {
		settings.current_channel = channel;

		if (in_rx_mode())
			rx_hop(channel);
	}
}

// get the tx/rx frequency channel
//
uint8_t 
radio_get_channel(void)
{
	return settings.current_channel;
}

#define NUM_DATA_RATES 13
// air data rates in kbps units
__code static const uint8_t air_data_rates[NUM_DATA_RATES] = {
	2,	4,	8,	16,	19,	24,	32,	48,	64,	96,	128,	192,	250
};


#include "radio_446x_conf.h"

static void
send_bulk_conf(__code uint8_t * __pdata ids, __code uint8_t * __pdata data)
{
	register uint8_t len;
	while ((len = *ids++) > 0) {
		NSS1 = 0;
		exchange_byte(0x11);
		exchange_byte(*ids++); /* group id */
		exchange_byte(len);
		exchange_byte(*ids++); /* property id start */
		while (len--)
			exchange_byte(*data++);
		NSS1 = 1;
		wait_for_cts();
	}
}

// configure radio based on the air data rate
//
bool
radio_configure(__pdata uint8_t air_rate)
{
	__pdata uint8_t i, field_flags;

	for (i = 0; i < NUM_DATA_RATES - 1; i++) {
		if (air_data_rates[i] >= air_rate) break;
	}
	send_bulk_conf(shared_prop_ids, shared_prop_vals);
	if (g_board_frequency == FREQ_433) {
		send_bulk_conf(variable_prop_ids, band_433_prop_vals[i]);
	} else if (g_board_frequency == FREQ_868) {
		send_bulk_conf(variable_prop_ids, band_868_prop_vals[i]);
	} else {
		return false; /* unsupported band */
	}

	settings.air_data_rate = air_data_rates[i];
	settings.preamble_length = 16;

	/* preamble is 16 nibbles, preamble rx treshold at 5<<3 bits*/
	cmd_set_property2(GROUP_PREAMBLE, 0x00, 0x10, 5<<3); // 0x08, 5<<2);
	wait_for_cts();
	/* preamble starts 0101, preamble length in nibbles */
	cmd_set_property1(GROUP_PREAMBLE, 0x04, /* 0x21); */ 0x02);
	wait_for_cts();

	cmd_set_property2(GROUP_SYNC, 0x01, 0xb4, 0x2b);
	wait_for_cts();
	
	/* CRC-16, seed is all zeroes */
	cmd_set_property1(GROUP_PKT, 0x00, 0x04);
	wait_for_cts();

	/* set data whitening compatible with Si1000 */
	cmd_set_property5(GROUP_PKT, 0x01, 0x01, 0x08, 0xff, 0xfe, 0x20);
	wait_for_cts();

	/* split fields for TX and RX, invert order of CRC bytes */
	cmd_set_property1(GROUP_PKT, 0x06, 0x82);
	wait_for_cts();

	if (!feature_golay) {
		/* leave length byte in FIFO, field 3 will have variable length */
		cmd_set_property1(GROUP_PKT, 0x08, 0x08 | 0x03);
		wait_for_cts();

		/* field 2 contains packet length */
		cmd_set_property1(GROUP_PKT, 0x09, 0x02);
		wait_for_cts();

		/* common flags shared by all fields */
		field_flags = 0x02; /* enable whitening */

		/* RX field 1 (header): two bytes long, seed PN generator, enable and seed CRC */
		cmd_set_property4(GROUP_PKT, 0x21, 0x00, 0x02, field_flags | 0x04, 0x82);
		wait_for_cts();

		/* RX field 2 (length): one byte long, enable CRC */
		cmd_set_property4(GROUP_PKT, 0x25, 0x00, 0x01, field_flags, 0x02);
		wait_for_cts();

		/* RX field 3 (payload): variable length, enable CRC, check CRC at end of field */
		cmd_set_property4(GROUP_PKT, 0x29, 0x00, MAX_PACKET_LENGTH, field_flags, 0x0a);
		wait_for_cts();

		/* TX field 1 (whole packet): seed PN generator, enable and seed CRC, send CRC at end of field */
		cmd_set_property4(GROUP_PKT, 0x0d, 0x00, 0x00, field_flags | 0x04, 0xa2);
		wait_for_cts();

		/* enable match bytes 1 and 2, point them toward the header */
		cmd_set_property3(GROUP_MATCH, 0x00, 0x00, 0xff, 0x40);
		wait_for_cts();
		cmd_set_property3(GROUP_MATCH, 0x03, 0x00, 0xff, 0x01);
		wait_for_cts();
		/* comply with requirement of 'non-descending offsets' */
		cmd_set_property1(GROUP_MATCH, 0x08, 0x01);
		wait_for_cts();
		cmd_set_property1(GROUP_MATCH, 0x0b, 0x01);
		wait_for_cts();
	} else {
		/* leave length byte in FIFO, field 2 will have variable length */
		cmd_set_property1(GROUP_PKT, 0x08, 0x08 | 0x02);
		wait_for_cts();

		/* field 1 contains packet length */
		cmd_set_property1(GROUP_PKT, 0x09, 0x01);
		wait_for_cts();

		/* common flags shared by all fields */
		field_flags = 0x02; /* enable whitening */

		/* RX field 1 (length): one byte long, no CRC */
		cmd_set_property4(GROUP_PKT, 0x21, 0x00, 0x01, field_flags | 0x04, 0x00);
		wait_for_cts();

		/* RX field 2 (payload): variable length, no CRC */
		cmd_set_property4(GROUP_PKT, 0x25, 0x00, MAX_PACKET_LENGTH, field_flags, 0x00);
		wait_for_cts();

		/* TX field 1 (whole packet): seed PN generator */
		cmd_set_property4(GROUP_PKT, 0x0d, 0x00, 0x00, field_flags | 0x04, 0x00);
		wait_for_cts();
	}

	cmd_set_property2(GROUP_PKT, 0x0b, TX_FIFO_THRESHOLD, RX_FIFO_THRESHOLD);
	wait_for_cts();

	/* latch RSSI at sync detect, no threshold, averaging AVERAGE4 */
	cmd_set_property1(GROUP_MODEM, 0x4c, 0x02);
	wait_for_cts();

	/* clear the FIFO to apply the mode change */
	cmd_fifo_info(0x03);
	wait_for_cts();

	return true;
}

// set the radio transmit power (in dBm)
//

/* TODO: not in dBm, measure? */

void 
radio_set_transmit_power(uint8_t power)
{
	EX0_SAVE_DISABLE;
	cmd_set_property1(GROUP_PA, 0x01, power);
	wait_for_cts();
	EX0_RESTORE;

	settings.transmit_power = power;
}

// get the current transmit power (in dBm)
//
uint8_t
radio_get_transmit_power(void)
{
	return settings.transmit_power;
}

// setup a 16 bit network ID
//
void
radio_set_network_id(uint16_t id)
{
	netid[0] = id&0xFF;
	netid[1] = id>>8;

	if (!feature_golay) {
		cmd_set_property1(GROUP_MATCH, 0x00, netid[1]);
		wait_for_cts();
		cmd_set_property1(GROUP_MATCH, 0x03, netid[0]);
		wait_for_cts();
	}
}

/// clear interrupts by reading the two status registers
///
static void
clear_status_registers(void)
{
	cmd_get_int_status_clear_all();
	wait_for_cts();
}

/// reset the radio using a software reset
///
/// @return			True if the radio reset correctly
static bool
software_reset(void)
{
	delay_set(20);
	return false;
}

static uint32_t convert_freq(uint8_t outdiv, uint32_t frequency) __reentrant
{
	register uint8_t x, i;
	uint32_t freq_control;

	frequency *= (outdiv / 2);

	freq_control = 0;
	for (i = 0; i < 20; i++) {
		freq_control <<= 1;
		if (i == 19)
			frequency += XO_FREQ / 2;
		x = frequency / XO_FREQ;
		freq_control += x;
		frequency -= x * XO_FREQ;
		frequency <<= 1;
	}

	return freq_control;
}

static void
set_frequency_registers(__pdata uint32_t base_freq, __pdata uint32_t freq_spacing)
{
	uint8_t band;

	outdiv = 2*XO_FREQ / (base_freq / 70);

	band = 0;
	if (outdiv >= 24) {
		outdiv = 24; band = 5;
	} else if (outdiv >= 16) {
		outdiv = 16; band = 4;
	} else if (outdiv >= 12) {
		outdiv = 12; band = 3;
	} else if (outdiv >= 8) {
		outdiv = 8; band = 2;
	} else if (outdiv >= 6) {
		outdiv = 6; band = 1;
	} else if (outdiv >= 4) {
		outdiv = 4; band = 0;
	} else {
		panic("set_frequency_registers: outdiv out of range");
	}

	/* MODEM_CLKGEN_BAND */
	cmd_set_property1(GROUP_MODEM, 0x51, 0x08 | band);
	wait_for_cts();

	freq_control_base = convert_freq(outdiv, base_freq);
	cmd_set_property4(
		GROUP_FREQ_CONTROL, 0x00,
		(freq_control_base>>19) - 1,
		((freq_control_base>>16) & 0x7) | 0x8,
		(freq_control_base>>8) & 0xff,
		freq_control_base & 0xff
	);
	wait_for_cts();

	// for compatibility with si1000
	freq_spacing += 10000/2;
	freq_spacing /= 10000;
	freq_spacing *= 10000;

	freq_control_spacing = (uint16_t) convert_freq(outdiv, freq_spacing);
	cmd_set_property2(
		GROUP_FREQ_CONTROL, 0x04,
		(freq_control_spacing>>8) & 0xff,
		freq_control_spacing & 0xff
	);
	wait_for_cts();
}

/// return temperature in degrees C
///
/// @return		temperature in degrees C
///
int16_t
radio_temperature(void)
{
	/*
	uint16_t ret;
	cmd_get_adc_reading(0x10, 0);
	get_adc_reading_reply(_skip, _skip, ret);
	ret *= 39;
	ret >>= 5;
	ret *= 29;
	ret >>= 7;
	return ((int16_t) ret) - 293;
	*/
	return 0;
}

/// Turn off radio diversity
///
void
radio_set_diversity(enum DIVERSITY_Enum state)
{
  switch (state) {
    case DIVERSITY_ENABLED:
    case DIVERSITY_ANT2:
    case DIVERSITY_DISABLED:
    case DIVERSITY_ANT1:
    default:
      break;
  }
}

/*
	Interrupt handler

	Only used when radio is receiving. The events that trigger interrupt
	are selected in RX_INT0_PH_MASK and RX_INT0_MODEM_MASK.

	My reading of the docs leads me to believe that SYNC word detection must
	be followed by one of:

		- Header filter miss
		- CRC error
		- Valid packet receipt

	All of these should be accompanied by change from RX to READY state, as
	we requested when issuing START_RX. In the first two cases, we go back
	to RX state from within the interrupt handler below. In the case of valid
	packet, we return to RX state when the packet contents is being picked up
	with a call to radio_receive_packet.

	We set sync_detected when SYNC word is detected, and clear it when
	reentering RX state.
*/

INTERRUPT(Receiver_ISR, INTERRUPT_INT0)
{
	uint8_t modem_pend, ph_status, ph_pend;
	uint8_t rx_fifo_count;

	/* only clear pending bits we handle here */
	cmd_get_int_status(~RX_INT0_PH_MASK, ~RX_INT0_MODEM_MASK, 0xff);
	get_int_status_reply(_skip, _skip, ph_pend, ph_status, modem_pend,
						  _skip, _skip, _skip);

	if (modem_pend & MODEM_STATUS_SYNC_DETECT) {
		sync_detected = 1;
	}

	if (ph_status & PH_STATUS_RX_FIFO_ALMOST_FULL) {
		if (RX_FIFO_THRESHOLD + (uint16_t) partial_packet_length > MAX_PACKET_LENGTH) {
			inc_rx_error();
			goto rxfail;
		}

		read_rx_fifo(RX_FIFO_THRESHOLD, radio_buffer + partial_packet_length);
		partial_packet_length += RX_FIFO_THRESHOLD;
	}

	if (ph_pend & PH_STATUS_FILTER_MISS)
		goto rxfail;

	if (ph_pend & PH_STATUS_CRC_ERROR) {
		debug("CRC error");
		inc_rx_error();
		goto rxfail;
	}

	if (ph_pend & PH_STATUS_PACKET_RX) {
		/* get packet length */
		cmd_packet_info(0, 0, 0);
		packet_info_reply(receive_packet_length);

		/* account for header and size byte */
		if (feature_golay) {
			receive_packet_length += 1;
		} else {
			receive_packet_length += 3;
		}

		if (receive_packet_length > MAX_PACKET_LENGTH) {
			debug("dropping too long a packet");
			inc_rx_error();
			goto rxfail;
		}

		/* retrieve number of bytes in FIFO */
		cmd_fifo_info(0);
		fifo_info_reply(rx_fifo_count, _skip);

		if (rx_fifo_count + partial_packet_length != receive_packet_length) {
			debug("length mismatch fifo=%x partial=%x packet=%x",
				  rx_fifo_count, partial_packet_length, receive_packet_length);
			inc_rx_error();
			goto rxfail;
		}

		if (partial_packet_length < receive_packet_length)
			read_rx_fifo(receive_packet_length - partial_packet_length,
						 radio_buffer + partial_packet_length);

		frr_a_read(last_rssi);
		packet_received = true;
	}

	return;

rxfail:
	_radio_receiver_on();
}

#endif
