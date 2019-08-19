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

#include "board.h"
#ifdef CPU_SI1060

#define DEBUG 1

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

__pdata const uint32_t xo_freq = 30000000;

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
static uint32_t scale_uint32(__pdata uint32_t value, __pdata uint32_t scale);

#define TX_FIFO_THRESHOLD 0x20
#define RX_FIFO_THRESHOLD 0x20

#define RX_INT0_PH_MASK 	(PH_STATUS_FILTER_MISS \
							| PH_STATUS_PACKET_RX \
							| PH_STATUS_CRC_ERROR \
							| PH_STATUS_RX_FIFO_ALMOST_FULL)
#define RX_INT0_MODEM_MASK	MODEM_STATUS_SYNC_DETECT


static void
_radio_receiver_on(void) __reentrant;

static void
clear_rx_fifo(void) __reentrant
{
	cmd_fifo_info(0x02);
	wait_for_cts();
}

bool
radio_receive_packet(uint8_t *length, __xdata uint8_t * __pdata buf)
{
	if (!packet_received)
		return false;

	*length = receive_packet_length-3;
	/* copy the packet into the caller's buffer, skip header and length */
	memcpy(buf, radio_buffer+3, receive_packet_length-3);

	EX0=0;
	_radio_receiver_on();
	EX0=1;
	return true;
}

bool
radio_receive_in_progress(void)
{
	return sync_detected;
}

bool
radio_preamble_detected(void)
{
	/* it's a lie, we return if sync is detected */
	return sync_detected;
}

uint8_t
radio_last_rssi(void)
{
	return 0;
	//return last_rssi;
}

uint8_t
radio_current_rssi(void)
{
	return 0;
}

uint8_t
radio_air_rate(void)
{
	return settings.air_data_rate;
}

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

static bool
in_rx_mode(void) __reentrant
{
	EX0_SAVE_DISABLE;
	frr_d_read();
	if (ret3.frr_d == STATE_RX) {
		EX0_RESTORE;
		return true;
	}
	EX0_RESTORE;
	return false;
}

bool
radio_transmit(uint8_t length, __xdata uint8_t * __pdata buf, __pdata uint16_t timeout_ticks)
{
	__pdata uint16_t tstart;
	__data uint8_t n, len_remaining;
	__xdata uint8_t scratch[3];

	if (timeout_ticks < 100)
		return false;

	EX0_SAVE_DISABLE;

	if (length > sizeof(radio_buffer))
		panic("oversized packet");

	cmd_get_int_status_clear_all();
	wait_for_cts();

	cmd_fifo_info(0x01); /* clear the TX FIFO */
	wait_for_cts();

	scratch[0] = netid[1];
	scratch[1] = netid[0];
	scratch[2] = length;
	write_tx_fifo(3, scratch);

	n = length;
	if (n > 32)
		n = 32;
	write_tx_fifo(n, buf);
	len_remaining = length - n;
	buf += n;

	cmd_start_tx(settings.current_channel, 0x30, length+3, 0, 0);
	wait_for_cts();

	tstart = timer2_tick();
	while ((uint16_t) (timer2_tick() - tstart) < timeout_ticks) {
		/* Fast Read Registers contain MODEM_STATUS, CHIP_STATUS and PH_STATUS */
		frr_abc_read();

		if ((ret2.frr_c & PH_STATUS_TX_FIFO_ALMOST_EMPTY)
				&& len_remaining != 0) {
			n = len_remaining;
			if (n > TX_FIFO_THRESHOLD)
				n = TX_FIFO_THRESHOLD;
			write_tx_fifo(n, buf);
			len_remaining -= n; buf += n;
		}

		if (ret1.frr_b & (CHIP_STATUS_FIFO_UNDERFLOW_OVERFLOW_ERROR \
							  | CHIP_STATUS_CMD_ERROR)) {
			debug("tx error: chip_pend=%x", ret1.frr_b);
			goto error;
		}

		if (ret2.frr_c & PH_STATUS_PACKET_SENT) {
			EX0_RESTORE;
			return true;
		}
	}

	debug("TX timeout %u ts=%u tn=%u len=%u\n",
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

	EX0_RESTORE;
	return false;
}

static void
_radio_receiver_on(void) __reentrant
{
	packet_received = 0;
	receive_packet_length = 0;
	sync_detected = 0;
	partial_packet_length = 0;

	/* make sure we are not in RX state before the FIFO reset */
	cmd_change_state(STATE_READY);
	wait_for_cts();

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
check_part(void)
{
	cmd_part_info();
	part_info_reply();
	
	debug("radio part info: chiprev=%x part=%x id=%x romid=%x",
				(unsigned) ret0.chiprev, (unsigned) ret1.part, (unsigned) ret3.id,
				(unsigned) ret5.romid);
	
	return true; //ret1.part&0xf0f0 == 0x4060;
}

static void
send_bulk_conf(__code uint8_t * __pdata data)
{
	register uint8_t len;
	while ((len = *data++) > 0) {
		NSS1 = 0;
		while (len--)
			exchange_byte(*data++);
		NSS1 = 1;
		wait_for_cts();
	}
}

#define RF_GLOBAL_XO_TUNE_2 0x11, 0x00, 0x02, 0x00, 0x52, 0x00
#define RF_GLOBAL_CONFIG_1 0x11, 0x00, 0x01, 0x03, 0x60
#define RF_MODEM_MOD_TYPE_12 0x11, 0x20, 0x0C, 0x00, 0x02, 0x00, 0x07, 0x09, 0xC4, 0x00, 0x01, 0xC9, 0xC3, 0x80, 0x00, 0x11
#define RF_MODEM_FREQ_DEV_0_1 0x11, 0x20, 0x01, 0x0C, 0x68
#define RF_MODEM_TX_RAMP_DELAY_8 0x11, 0x20, 0x08, 0x18, 0x01, 0x00, 0x08, 0x03, 0x80, 0x00, 0x10, 0x20
#define RF_MODEM_BCR_OSR_1_9 0x11, 0x20, 0x09, 0x22, 0x00, 0x75, 0x04, 0x5E, 0x7B, 0x02, 0x32, 0x02, 0x00
#define RF_MODEM_AFC_GEAR_7 0x11, 0x20, 0x07, 0x2C, 0x00, 0x12, 0x82, 0x2F, 0x02, 0xC5, 0xE0
#define RF_MODEM_AGC_CONTROL_1 0x11, 0x20, 0x01, 0x35, 0xE2
#define RF_MODEM_AGC_WINDOW_SIZE_9 0x11, 0x20, 0x09, 0x38, 0x11, 0x1A, 0x1A, 0x00, 0x02, 0x7F, 0x80, 0x00, 0x28
#define RF_MODEM_OOK_CNT1_9 0x11, 0x20, 0x09, 0x42, 0xA4, 0x03, 0xD6, 0x03, 0x02, 0x1F, 0x01, 0x80, 0xFF
#define RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12 0x11, 0x21, 0x0C, 0x00, 0xFF, 0xC4, 0x30, 0x7F, 0xF5, 0xB5, 0xB8, 0xDE, 0x05, 0x17, 0x16, 0x0C
#define RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12 0x11, 0x21, 0x0C, 0x0C, 0x03, 0x00, 0x15, 0xFF, 0x00, 0x00, 0xFF, 0xC4, 0x30, 0x7F, 0xF5, 0xB5
#define RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12 0x11, 0x21, 0x0C, 0x18, 0xB8, 0xDE, 0x05, 0x17, 0x16, 0x0C, 0x03, 0x00, 0x15, 0xFF, 0x00, 0x00
#define RF_SYNTH_PFDCP_CPFF_7 0x11, 0x23, 0x07, 0x00, 0x2C, 0x0E, 0x0B, 0x04, 0x0C, 0x73, 0x03

__code static const uint8_t conf_64kbps[] = {
    0x06, RF_GLOBAL_XO_TUNE_2, \
    0x05, RF_GLOBAL_CONFIG_1, \
    0x10, RF_MODEM_MOD_TYPE_12, \
    0x05, RF_MODEM_FREQ_DEV_0_1, \
    0x0C, RF_MODEM_TX_RAMP_DELAY_8, \
    0x0D, RF_MODEM_BCR_OSR_1_9, \
    0x0B, RF_MODEM_AFC_GEAR_7, \
    0x05, RF_MODEM_AGC_CONTROL_1, \
    0x0D, RF_MODEM_AGC_WINDOW_SIZE_9, \
    0x0D, RF_MODEM_OOK_CNT1_9, \
    0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12, \
    0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12, \
    0x10, RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12, \
    0x0B, RF_SYNTH_PFDCP_CPFF_7, \
    0x00 \
};

// initialise the radio hardware
//
bool
radio_initialise(void)
{
	__pdata uint8_t	len;

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

	cmd_gpio_pin_cfg(
		17,
		20,
		9, // GPIO3: inverted CTS
		33, // GPIO4: RX state
		0, // nIRQ
		0, // SDO
		0
	);
	wait_for_cts();

	cmd_set_property4(GROUP_FRR_CTL, 0x0,
		0x05, /* FRR A = INT_MODEM_STATUS */
		0x07, /* FRR B = INT_CHIP_STATUS */
		0x03, /* FRR C = INT_PH_STATUs */
		0x09 /* FRR D = CURRENT_STATE */
	);
	wait_for_cts();

	cmd_set_property4(GROUP_INT_CTL, 0x0,
		0x03, /* MODEM_INT_STATUS_EN | PH_INT_STATUS_EN */
		RX_INT0_PH_MASK,
		RX_INT0_MODEM_MASK,
		0x00
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

// set the tx/rx frequency channel
//
void
radio_set_channel(uint8_t channel)
{
	if (channel != settings.current_channel) {
		settings.current_channel = channel;

		EX0_SAVE_DISABLE;
		if (in_rx_mode())
			_radio_receiver_on();
		EX0_RESTORE;
	}
}

// get the tx/rx frequency channel
//
uint8_t 
radio_get_channel(void)
{
	return settings.current_channel;
}

// configure radio based on the air data rate
//
bool
radio_configure(__pdata uint8_t air_rate)
{
	uint8_t field_flags;

	cmd_set_property2(GROUP_PREAMBLE, 0x00, 0x10, 5<<3);
	wait_for_cts();
	cmd_set_property1(GROUP_PREAMBLE, 0x04, 0x21);
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

	/* leave length byte in FIFO, field 3 will have variable length */
	cmd_set_property1(GROUP_PKT, 0x08, 0x08 | 0x03);
	wait_for_cts();

	/* field 2 contains packet length */
	cmd_set_property1(GROUP_PKT, 0x09, 0x02);
	wait_for_cts();

	/* commond flags shared by all fields */
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

	cmd_set_property2(GROUP_PKT, 0x0b, TX_FIFO_THRESHOLD, RX_FIFO_THRESHOLD);
	wait_for_cts();

	/* PA_PWR_LVL = 0x01 */
	cmd_set_property1(GROUP_PA, 0x01, 0x01);
	wait_for_cts();

	/* only one airrate supported at the moment */
	send_bulk_conf(conf_64kbps);

	/* clear the FIFO to apply the mode change */
	clear_rx_fifo();

	return true;
}

// set the radio transmit power (in dBm)
//
void 
radio_set_transmit_power(uint8_t power)
{
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

	cmd_set_property1(GROUP_MATCH, 0x00, netid[1]);
	wait_for_cts();
	cmd_set_property1(GROUP_MATCH, 0x03, netid[0]);
	wait_for_cts();
}

/// clear interrupts by reading the two status registers
///
static void
clear_status_registers(void)
{
	cmd_get_int_status_clear_all();
	wait_for_cts();
}

/// scale a uint32_t, rounding to nearest multiple
///
/// @param value		The value to scale
/// @param scale		The scale factor
/// @return			value / scale, rounded to the nearest integer
///
static uint32_t
scale_uint32(__pdata uint32_t value, __pdata uint32_t scale)
{
	return (value + (scale >> 1)) / scale;
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

static uint32_t convert_freq(__pdata uint8_t outdiv, __pdata uint32_t frequency)
{
	register uint8_t x, i;
	uint32_t freq_control;

	frequency *= (outdiv / 2);

	freq_control = 0;
	for (i = 0; i < 20; i++) {
		freq_control <<= 1;
		if (i == 19)
			frequency += xo_freq / 2;
		x = frequency / xo_freq;
		freq_control += x;
		frequency -= x * xo_freq;
		frequency <<= 1;
	}

	return freq_control;
}

static void
set_frequency_registers(__pdata uint32_t base_freq, __pdata uint32_t freq_spacing)
{
	uint8_t band;
	uint32_t freq_control;
	uint8_t outdiv;

	outdiv = 2*xo_freq / (base_freq / 70);

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

	freq_control = convert_freq(outdiv, base_freq);
	cmd_set_property4(
		GROUP_FREQ_CONTROL, 0x00,
		(freq_control >> 19) - 1,
		((freq_control>>16) & 0x7) | 0x8,
		(freq_control>>8) & 0xff,
		freq_control & 0xff
	);

	freq_control = convert_freq(outdiv, freq_spacing);
	cmd_set_property2(
		GROUP_FREQ_CONTROL, 0x04,
		(freq_control>>8) & 0xff,
		freq_control & 0xff
	);
}

/// return temperature in degrees C
///
/// @return		temperature in degrees C
///
int16_t
radio_temperature(void)
{
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
	Interrupt handler, used for receiving and triggered by events selected
	in RX_INT0_PH_MASK and RX_INT0_MODEM_MASK

	My reading of the docs leads me to believe that SYNC word detection must 
	be followed by one of:

		- Header filter miss
		- CRC error
		- Valid packet receipt

	All of these should be accompanied with change from RX to READY state, as
	we requested when issuing START_RX. In the first two cases, we go back to RX state
	from within the interrupt handler below. In the case of a valid packet, we wait
	until the packet is picked up by call to radio_receive_packet, then we change
	back to RX state there.

	We set sync_detected when SYNC word is detected, and clear it when reentering RX state.
*/

INTERRUPT(Receiver_ISR, INTERRUPT_INT0)
{
	/* only clear pending bits we handle here */
	cmd_get_int_status(~RX_INT0_PH_MASK, ~RX_INT0_MODEM_MASK, 0xff);
	get_int_status_reply();

	if (ret4.modem_pend & MODEM_STATUS_SYNC_DETECT) {
		sync_detected = 1;
	}

	if (ret3.ph_status & PH_STATUS_RX_FIFO_ALMOST_FULL) {
		if (RX_FIFO_THRESHOLD + (uint16_t) partial_packet_length > MAX_PACKET_LENGTH) {
			inc_rx_error();
			goto rxfail;
		}

		read_rx_fifo(RX_FIFO_THRESHOLD, radio_buffer + partial_packet_length);
		partial_packet_length += RX_FIFO_THRESHOLD;
	}

	if (ret2.ph_pend & PH_STATUS_FILTER_MISS)
		goto rxfail;

	if (ret2.ph_pend & PH_STATUS_CRC_ERROR) {
		inc_rx_error();
		goto rxfail;
	}

	if (ret2.ph_pend & PH_STATUS_PACKET_RX) {
		/* get packet length */
		cmd_packet_info(0, 0, 0);
		packet_info_reply();

		/* account for header and size byte */
		ret0.length += 3;

		if (ret0.length > MAX_PACKET_LENGTH) {
			debug("dropping too long a packet");
			inc_rx_error();
			goto rxfail;
		}
		receive_packet_length = ret0.length;

		/* retrieve number of bytes in FIFO */
		//cmd_fifo_info(0);
		//fifo_info_reply();
		/*
		if (ret0.rx_fifo_count + partial_packet_length != receive_packet_length) {
			debug("length mismatch fifo=%x partial=%x packet=%x",
				  ret0.rx_fifo_count, partial_packet_length, receive_packet_length);
			inc_rx_error();
			goto rxfail;
		}
		*/

		if (partial_packet_length < receive_packet_length)
			read_rx_fifo(receive_packet_length - partial_packet_length,
						 radio_buffer + partial_packet_length);

		packet_received = true;
	}

	return;

rxfail:
	_radio_receiver_on();
}

#endif
