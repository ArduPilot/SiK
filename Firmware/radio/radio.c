// -*- Mode: C; c-basic-offset: 8; -*-
//
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

#define DEBUG 1

#include "board.h"
#include "radio.h"
#include "timer.h"
#include "crc.h"

__xdata uint8_t radio_buffer[MAX_PACKET_LENGTH];
__pdata uint8_t receive_packet_length;
__pdata uint8_t last_rssi;
__pdata uint8_t netid[2];

static volatile __bit packet_received;
static volatile __bit preamble_detected;

__pdata struct radio_settings settings;


// internal helper functions
//
static void	register_write(uint8_t reg, uint8_t value) __reentrant;
static uint8_t	register_read(uint8_t reg);
static bool	software_reset(void);
static void	set_frequency_registers(uint32_t frequency);
static uint32_t scale_uint32(uint32_t value, uint32_t scale);
static void	clear_status_registers(void);

// save and restore radio interrupt. We use this rather than
// __critical to ensure we don't disturb the timer interrupt at all.
// minimal tick drift is critical for TDM
#define EX0_SAVE_DISABLE __bit EX0_saved = EX0; EX0 = 0
#define EX0_RESTORE EX0 = EX0_saved

#define RADIO_RX_INTERRUPTS (EZRADIOPRO_ENRXFFAFULL|EZRADIOPRO_ENPKVALID|EZRADIOPRO_ENCRCERROR)

// FIFO thresholds to allow for packets larger than 64 bytes
#define TX_FIFO_THRESHOLD_LOW 32
#define TX_FIFO_THRESHOLD_HIGH 60
#define RX_FIFO_THRESHOLD_HIGH 50

// return a received packet
//
// returns true on success, false on no packet available
//
bool
radio_receive_packet(uint8_t *length, __xdata uint8_t * __pdata buf)
{
	__data uint8_t errcount = 0;

	if (!packet_received) {
		return false;
	}

	if (receive_packet_length > MAX_PACKET_LENGTH) {
		radio_receiver_on();
		goto failed;
	}

#if 0
	// useful for testing high packet loss
	if ((timer_entropy() & 0x7) != 0) {
		radio_receiver_on();
		goto failed;		
	}
#endif

	// simple unencoded packets
	*length = receive_packet_length;
	memcpy(buf, radio_buffer, receive_packet_length);
	radio_receiver_on();
	return true;

failed:
	if (errors.rx_errors != 0xFFFF) {
		errors.rx_errors++;
	}
	return false;
}


// write to the radios transmit FIFO
//
static void
radio_write_transmit_fifo(register uint8_t n, __xdata uint8_t * __pdata buffer)
{
	NSS1 = 0;
	SPIF1 = 0;
	SPI1DAT = (0x80 | EZRADIOPRO_FIFO_ACCESS);

	while (n--) {
		while (!TXBMT1) /* noop */;
		SPI1DAT = *buffer++;
	}

	while (!TXBMT1) /* noop */;
	while ((SPI1CFG & 0x80) == 0x80);

	SPIF1 = 0;
	NSS1 = 1;
}

// check if a packet is being received
//
bool
radio_receive_in_progress(void)
{
	if (packet_received) {
		return true;
	}

	// check the status register to see if a receive is in progress
	if (register_read(EZRADIOPRO_EZMAC_STATUS) & EZRADIOPRO_PKRX) {
		return true;
	}
	return false;
}

// return true if a packet preamble has been detected. This means that
// a packet may be coming in
//
bool
radio_preamble_detected(void)
{
	EX0_SAVE_DISABLE;
	if (preamble_detected) {
		preamble_detected = 0;
		EX0_RESTORE;
		return true;
	}
	EX0_RESTORE;
	return false;
}



// return the RSSI from the last packet
//
uint8_t
radio_last_rssi(void)
{
	return last_rssi;
}

// return the current signal strength, for LBT
//
uint8_t
radio_current_rssi(void)
{
	return register_read(EZRADIOPRO_RECEIVED_SIGNAL_STRENGTH_INDICATOR);
}

// return the actual air data rate in BPS
//
uint8_t
radio_air_rate(void)
{
	return settings.air_data_rate;
}

// clear the transmit FIFO
//
static void
radio_clear_transmit_fifo(void)
{
	register uint8_t control;
	control = register_read(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_2);
	register_write(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_2, control | EZRADIOPRO_FFCLRTX);
	register_write(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_2, control & ~EZRADIOPRO_FFCLRTX);
}


// clear the receive FIFO
//
static void
radio_clear_receive_fifo(void) __reentrant
{
	register uint8_t control;
	control = register_read(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_2);
	register_write(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_2, control | EZRADIOPRO_FFCLRRX);
	register_write(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_2, control & ~EZRADIOPRO_FFCLRRX);
}

// put the radio in receive mode
//
bool
radio_receiver_on(void)
{
	EX0 = 0;

	packet_received = 0;
	receive_packet_length = 0;
	preamble_detected = 0;

	// enable receive interrupts
	register_write(EZRADIOPRO_INTERRUPT_ENABLE_1, RADIO_RX_INTERRUPTS);
	register_write(EZRADIOPRO_INTERRUPT_ENABLE_2, EZRADIOPRO_ENPREAVAL);

	clear_status_registers();
	radio_clear_transmit_fifo();
	radio_clear_receive_fifo();

	// put the radio in receive mode
	register_write(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_1, EZRADIOPRO_RXON | EZRADIOPRO_XTON);

	// enable receive interrupt
	EX0 = 1;

	return true;
}


// initialise the radio hardware
//
bool
radio_initialise(void)
{
	uint8_t status;

	delay_msec(50);

	// make sure there is a radio on the SPI bus
	status = register_read(EZRADIOPRO_DEVICE_VERSION);
	if (status == 0xFF || status < 5) {
		// no valid radio there?
		return false;
	}

	status = register_read(EZRADIOPRO_INTERRUPT_STATUS_2);

	if ((status & EZRADIOPRO_IPOR) == 0) {
		// it hasn't powered up cleanly, reset it
		return software_reset();
	}

	if (status & EZRADIOPRO_ICHIPRDY) {
		// already ready
		return true;
	}

	// enable chip ready interrupt
	register_write(EZRADIOPRO_INTERRUPT_ENABLE_1, 0);
	register_write(EZRADIOPRO_INTERRUPT_ENABLE_2, EZRADIOPRO_ENCHIPRDY);

	// wait for the chip ready bit for 10ms
	delay_set(50);
	while (!delay_expired()) {
		status = register_read(EZRADIOPRO_INTERRUPT_STATUS_1);
		status = register_read(EZRADIOPRO_INTERRUPT_STATUS_2);
		if (status & EZRADIOPRO_ICHIPRDY) {
			return true;
		}
	}

	return false;
}


// set the transmit frequency
//
bool
radio_set_frequency(__pdata uint32_t value)
{
	if (value < 240000000UL || value > 935000000UL) {
		return false;
	}
	printf("Set frequency %lu\n", (unsigned long)value);
	settings.frequency = value;
	set_frequency_registers(value);
	return true;
}


// get the tx/rx frequency channel
//
uint8_t 
radio_get_channel(void)
{
	return settings.current_channel;
}

// This table gives the register settings for the radio core indexed by
// the desired air data rate.
//
#define NUM_DATA_RATES 13
#define NUM_RADIO_REGISTERS 12

// this is the list of registers that we will use the register tables for
__code static const uint8_t reg_index[NUM_RADIO_REGISTERS] = {
		EZRADIOPRO_IF_FILTER_BANDWIDTH, // 0x1C
		EZRADIOPRO_CLOCK_RECOVERY_GEARSHIFT_OVERRIDE, // 0x1F
		EZRADIOPRO_CLOCK_RECOVERY_OVERSAMPLING_RATIO, // 0x20
		EZRADIOPRO_CLOCK_RECOVERY_OFFSET_2, // 0x21
		EZRADIOPRO_CLOCK_RECOVERY_OFFSET_1, // 0x22
		EZRADIOPRO_CLOCK_RECOVERY_OFFSET_0, // 0x23
		EZRADIOPRO_CLOCK_RECOVERY_TIMING_LOOP_GAIN_1, // 0x24
		EZRADIOPRO_CLOCK_RECOVERY_TIMING_LOOP_GAIN_0, // 0x25
		EZRADIOPRO_AFC_LIMITER, // 0x2A
		EZRADIOPRO_TX_DATA_RATE_1, // 0x6E
		EZRADIOPRO_TX_DATA_RATE_0, // 0x6F
		EZRADIOPRO_FREQUENCY_DEVIATION, // 0x72
};

// air data rates in kbps units
__code static const uint8_t air_data_rates[NUM_DATA_RATES] = {
	2,	4,	8,	16,	19,	24,	32,	48,	64,	96,	128,	192,	250
};

// register table for 915MHz radios
__code static const uint8_t reg_table_915[NUM_RADIO_REGISTERS][NUM_DATA_RATES] = {
	{0x01,	0x01,	0x01,	0x01,	0x01,	0x01,	0x05,	0x0B,	0x9A,	0x88,	0x8A,	0x8C,	0x8D},
	{0x03,	0x03,	0x03,	0x03,	0x03,	0x03,	0x03,	0x03,	0x03,	0x03,	0x03,	0x03,	0x03},
	{0xD0,	0xE8,	0xF4,	0xFA,	0xD0,	0xD0,	0x7D,	0x53,	0x5E,	0x7D,	0x5E,	0x3F,	0x30},
	{0xE0,	0x60,	0x20,	0x00,	0x00,	0x00,	0x01,	0x01,	0x01,	0x01,	0x01,	0x02,	0x02},
	{0x10,	0x20,	0x41,	0x83,	0x9D,	0xC4,	0x06,	0x89,	0x5D,	0x06,	0x5D,	0x0C,	0xAA},
	{0x62,	0xC5,	0x89,	0x12,	0x49,	0x9C,	0x25,	0x37,	0x86,	0x25,	0x86,	0x4A,	0xAB},
	{0x00,	0x00,	0x00,	0x01,	0x02,	0x01,	0x02,	0x03,	0x02,	0x02,	0x02,	0x04,	0x07},
	{0x23,	0x44,	0x85,	0x08,	0x7D,	0x8A,	0x0E,	0x18,	0xBB,	0x0E,	0xBB,	0xEA,	0xFF},
	{0x1E,	0x1E,	0x1E,	0x1E,	0x1E,	0x1E,	0x20,	0x30,	0x41,	0x50,	0x50,	0x50,	0x50},
	{0x10,	0x20,	0x41,	0x83,	0x9D,	0xC4,	0x08,	0x0C,	0x10,	0x18,	0x20,	0x31,	0x40},
	{0x62,	0xC5,	0x89,	0x12,	0x49,	0x9C,	0x31,	0x4A,	0x62,	0x93,	0xC5,	0x27,	0x00},
	{0x03,	0x06,	0x0D,	0x1A,	0x0F,	0x26,	0x33,	0x4D,	0x66,	0x9A,	0xCD,	0xFE,	0xFE}
};


// register table for 433MHz radios
__code static const uint8_t reg_table_433[NUM_RADIO_REGISTERS][NUM_DATA_RATES] = {
	{0x27,	0x27,	0x27,	0x2E,	0x16,	0x01,	0x05,	0x0B,	0x9A,	0x88,	0x8A,	0x8C,	0x8D},
	{0x03,	0x03,	0x03,	0x03,	0x03,	0x03,	0x03,	0x03,	0x03,	0x03,	0x03,	0x03,	0x03},
	{0xF4,	0xFA,	0x7D,	0x3F,	0x69,	0xA7,	0x7D,	0x53,	0x5E,	0x7D,	0x5E,	0x3F,	0x30},
	{0x20,	0x00,	0x01,	0x02,	0x01,	0x00,	0x01,	0x01,	0x01,	0x01,	0x01,	0x02,	0x02},
	{0x41,	0x83,	0x06,	0x0C,	0x37,	0xC4,	0x06,	0x89,	0x5D,	0x06,	0x5D,	0x0C,	0xAA},
	{0x89,	0x12,	0x25,	0x4A,	0x4C,	0x9C,	0x25,	0x37,	0x86,	0x25,	0x86,	0x4A,	0xAB},
	{0x00,	0x01,	0x02,	0x04,	0x02,	0x01,	0x02,	0x03,	0x02,	0x02,	0x02,	0x04,	0x07},
	{0x85,	0x08,	0x0E,	0x12,	0x72,	0x8A,	0x0E,	0x18,	0xBB,	0x0E,	0xBB,	0xEA,	0xFF},
	{0x1D,	0x1D,	0x1D,	0x1E,	0x1E,	0x1E,	0x20,	0x30,	0x41,	0x50,	0x50,	0x50,	0x50},
	{0x10,	0x20,	0x41,	0x83,	0x9B,	0xC4,	0x08,	0x0C,	0x10,	0x18,	0x20,	0x31,	0x40},
	{0x62,	0xC5,	0x89,	0x12,	0xA6,	0x9C,	0x31,	0x4A,	0x62,	0x93,	0xC5,	0x27,	0x00},
	{0x03,	0x06,	0x0D,	0x1A,	0x1E,	0x26,	0x33,	0x4D,	0x66,	0x9A,	0xCD,	0xFE,	0xFE}
};

// register table for 470MHz radios
__code static const uint8_t reg_table_470[NUM_RADIO_REGISTERS][NUM_DATA_RATES] = {
        {0x2B,  0x2B,   0x2B,   0x2E,   0x16,   0x01,   0x05,   0x0B,   0x9A,   0x88,   0x8A,   0x8C,   0x8D},
        {0x03,  0x03,   0x03,   0x03,   0x03,   0x03,   0x03,   0x03,   0x03,   0x03,   0x03,   0x03,   0x03},
        {0xF4,  0xFA,   0x7D,   0x3F,   0x69,   0xA7,   0x7D,   0x53,   0x5E,   0x7D,   0x5E,   0x3F,   0x30},
        {0x20,  0x00,   0x01,   0x02,   0x01,   0x00,   0x01,   0x01,   0x01,   0x01,   0x01,   0x02,   0x02},
        {0x41,  0x83,   0x06,   0x0C,   0x37,   0xC4,   0x06,   0x89,   0x5D,   0x06,   0x5D,   0x0C,   0xAA},
        {0x89,  0x12,   0x25,   0x4A,   0x4C,   0x9C,   0x25,   0x37,   0x86,   0x25,   0x86,   0x4A,   0xAB},
        {0x00,  0x01,   0x02,   0x04,   0x02,   0x01,   0x02,   0x03,   0x02,   0x02,   0x02,   0x04,   0x07},
        {0x85,  0x08,   0x0E,   0x12,   0x72,   0x8A,   0x0E,   0x18,   0xBB,   0x0E,   0xBB,   0xEA,   0xFF},
        {0x1E,  0x1E,   0x1E,   0x21,   0x21,   0x21,   0x21,   0x30,   0x41,   0x50,   0x50,   0x50,   0x50},
        {0x10,  0x20,   0x41,   0x83,   0x9B,   0xC4,   0x08,   0x0C,   0x10,   0x18,   0x20,   0x31,   0x40},
        {0x62,  0xC5,   0x89,   0x12,   0xA6,   0x9C,   0x31,   0x4A,   0x62,   0x93,   0xC5,   0x27,   0x00},
        {0x03,  0x06,   0x0D,   0x1A,   0x1E,   0x26,   0x33,   0x4D,   0x66,   0x9A,   0xCD,   0xFE,   0xFE}
};

// register table for 868MHz radios
__code static const uint8_t reg_table_868[NUM_RADIO_REGISTERS][NUM_DATA_RATES] = {
        {0x01,  0x01,   0x01,   0x01,   0x01,   0x01,   0x05,   0x0B,   0x9A,   0x88,   0x8A,   0x8C,   0x8D},
        {0x03,  0x03,   0x03,   0x03,   0x03,   0x03,   0x03,   0x03,   0x03,   0x03,   0x03,   0x03,   0x03},
        {0xD0,  0xE8,   0xF4,   0xFA,   0xD3,   0xA7,   0x7D,   0x53,   0x5E,   0x7D,   0x5E,   0x3F,   0x30},
        {0xE0,  0x60,   0x20,   0x00,   0x00,   0x00,   0x01,   0x01,   0x01,   0x01,   0x01,   0x02,   0x02},
        {0x10,  0x20,   0x41,   0x83,   0x9B,   0xC4,   0x06,   0x89,   0x5D,   0x06,   0x5D,   0x0C,   0xAA},
        {0x62,  0xC5,   0x89,   0x12,   0xA6,   0x9C,   0x25,   0x37,   0x86,   0x25,   0x86,   0x4A,   0xAB},
        {0x00,  0x00,   0x00,   0x01,   0x01,   0x01,   0x02,   0x03,   0x02,   0x02,   0x02,   0x04,   0x07},
        {0x23,  0x44,   0x85,   0x08,   0x39,   0x8A,   0x0E,   0x18,   0xBB,   0x0E,   0xBB,   0xEA,   0xFF},
        {0x1C,  0x1C,   0x1C,   0x1C,   0x1C,   0x1E,   0x20,   0x30,   0x41,   0x50,   0x50,   0x50,   0x50},
        {0x10,  0x20,   0x41,   0x83,   0x9B,   0xC4,   0x08,   0x0C,   0x10,   0x18,   0x20,   0x31,   0x40},
        {0x62,  0xC5,   0x89,   0x12,   0xA6,   0x9C,   0x31,   0x4A,   0x62,   0x93,   0xC5,   0x27,   0x00},
        {0x03,  0x06,   0x0D,   0x1A,   0x1E,   0x26,   0x33,   0x4D,   0x66,   0x9A,   0xCD,   0xFE,   0xFE}
};

// configure radio based on the air data rate
//
bool
radio_configure(__pdata uint8_t air_rate)
{
	__pdata uint8_t i, rate_selection, control;

	// disable interrupts
	register_write(EZRADIOPRO_INTERRUPT_ENABLE_1, 0x00);
	register_write(EZRADIOPRO_INTERRUPT_ENABLE_2, 0x00);

	clear_status_registers();

#ifdef ENABLE_RF_SWITCH
	//set GPIO0 to GND
	register_write(EZRADIOPRO_GPIO0_CONFIGURATION, 0x14);	// RX data (output)
	//set GPIO1 & GPIO2 to control the TRX switch
	register_write(EZRADIOPRO_GPIO1_CONFIGURATION, 0x12);	// TX state (output)
	register_write(EZRADIOPRO_GPIO2_CONFIGURATION, 0x15);	// RX state (output)
#elif ENABLE_RFM50_SWITCH
	//set GPIO0 & GPIO1 to control the TRX switch
	register_write(EZRADIOPRO_GPIO0_CONFIGURATION, 0x15);	// RX state (output)
	register_write(EZRADIOPRO_GPIO1_CONFIGURATION, 0x12);	// TX state (output)
	//set GPIO2 to GND
#elif ENABLE_RFD900_SWITCH
	register_write(EZRADIOPRO_GPIO0_CONFIGURATION, 0x15);	// RX data (output)
	register_write(EZRADIOPRO_GPIO1_CONFIGURATION, 0x12);	// RX data (output)
#if RFD900_DIVERSITY
	radio_set_diversity(true);
#else
	radio_set_diversity(false);
#endif
#else
	//set GPIOx to GND
	register_write(EZRADIOPRO_GPIO0_CONFIGURATION, 0x14);	// RX data (output)
	register_write(EZRADIOPRO_GPIO1_CONFIGURATION, 0x14);	// RX data (output)
	register_write(EZRADIOPRO_GPIO2_CONFIGURATION, 0x14);	// RX data (output)
#endif

	// set capacitance
	register_write(EZRADIOPRO_CRYSTAL_OSCILLATOR_LOAD_CAPACITANCE, EZRADIOPRO_OSC_CAP_VALUE);

	// see Si1000.pdf section 23.3.8
	if (air_rate > 100) {
		register_write(EZRADIOPRO_CHARGEPUMP_CURRENT_TRIMMING_OVERRIDE, 0xC0);
	}

	// setup frequency and channel spacing
	set_frequency_registers(settings.frequency);
	register_write(EZRADIOPRO_FREQUENCY_HOPPING_STEP_SIZE, settings.channel_spacing);

	register_write(EZRADIOPRO_DATA_ACCESS_CONTROL,
		       EZRADIOPRO_ENPACTX | 
		       EZRADIOPRO_ENPACRX |
		       EZRADIOPRO_CRC_16);

	// 2 sync bytes and no header bytes
	register_write(EZRADIOPRO_HEADER_CONTROL_2, 
		       EZRADIOPRO_FIXPKLEN | 
		       EZRADIOPRO_HDLEN_0BYTE | 
		       EZRADIOPRO_SYNCLEN_2BYTE);
	register_write(EZRADIOPRO_TRANSMIT_PACKET_LENGTH, 10);
	
	// no header check
	register_write(EZRADIOPRO_HEADER_CONTROL_1, 0x00);
	
	register_write(EZRADIOPRO_SYNC_WORD_3, 0xCB);
	register_write(EZRADIOPRO_SYNC_WORD_2, 0x89);

	register_write(EZRADIOPRO_FREQUENCY_HOPPING_CHANNEL_SELECT, 0);

	// set FIFO limits to allow for sending larger than 64 byte packets
	register_write(EZRADIOPRO_TX_FIFO_CONTROL_1, TX_FIFO_THRESHOLD_HIGH);
	register_write(EZRADIOPRO_TX_FIFO_CONTROL_2, TX_FIFO_THRESHOLD_LOW);
	register_write(EZRADIOPRO_RX_FIFO_CONTROL, RX_FIFO_THRESHOLD_HIGH);

	settings.preamble_length = 8;

	register_write(EZRADIOPRO_PREAMBLE_LENGTH, settings.preamble_length); // nibbles 
	register_write(EZRADIOPRO_PREAMBLE_DETECTION_CONTROL, 4<<3);

	// setup minimum output power during startup
	radio_set_transmit_power(0);

	// work out which register table column we will use
	for (i = 0; i < NUM_DATA_RATES - 1; i++) {
		if (air_data_rates[i] >= air_rate) break;
	}
	rate_selection = i;

	settings.air_data_rate = air_data_rates[rate_selection];

	if (settings.air_data_rate >= 32) {
		control = 0x0D;
	} else {
		control = 0x2D;
	}
	if (param_get(PARAM_MANCHESTER) && settings.air_data_rate <= 128) {
		// manchester encoding is not possible at above 128kbps
		control |= EZRADIOPRO_ENMANCH;
	}
	register_write(EZRADIOPRO_MODULATION_MODE_CONTROL_1, control);

	register_write(EZRADIOPRO_MODULATION_MODE_CONTROL_2, 0x23);

	// note that EZRADIOPRO_AFCBD does not seem to work, which
	// is a pity!
	register_write(EZRADIOPRO_AFC_LOOP_GEARSHIFT_OVERRIDE, 0x44);

	// this follows the recommendation in the register spreadsheet
	// for AFC enabled and manchester disabled
	if (settings.air_data_rate < 200) {
		register_write(EZRADIOPRO_AFC_TIMING_CONTROL, 0x0A);
	} else {
		register_write(EZRADIOPRO_AFC_TIMING_CONTROL, 0x02);
	}

	// set the registers from the tables
	if (g_board_frequency == FREQ_433) {
		for (i = 0; i < NUM_RADIO_REGISTERS; i++) {
			register_write(reg_index[i],
				       reg_table_433[i][rate_selection]);
		}
	} else if (g_board_frequency == FREQ_470) {
		for (i = 0; i < NUM_RADIO_REGISTERS; i++) {
			register_write(reg_index[i],
				       reg_table_470[i][rate_selection]);
		}
	} else if (g_board_frequency == FREQ_868) {
		for (i = 0; i < NUM_RADIO_REGISTERS; i++) {
			register_write(reg_index[i],
				       reg_table_868[i][rate_selection]);
		}
	} else {
		for (i = 0; i < NUM_RADIO_REGISTERS; i++) {
			register_write(reg_index[i],
				       reg_table_915[i][rate_selection]);
		}
	}

	return true;
}

#ifdef _BOARD_RFD900
	#define NUM_POWER_LEVELS 5
	__code static const uint8_t power_levels[NUM_POWER_LEVELS] = { 17, 20, 27, 29, 30 };
#endif
#ifdef _BOARD_RFD900A
	#define NUM_POWER_LEVELS 16
	#define POWER_LEVEL_STEP 2
	// the power_levels array define 8 bit PWM values for each respective power level starting at 0dBm
	// PWM=240 gives TXout=0dBm
	//run1 __code static const uint8_t power_levels[NUM_POWER_LEVELS] = { 240, 234, 226, 221, 214, 209, 204, 199, 193, 187, 180, 174, 165, 153, 137, 50 };
	__code static const uint8_t power_levels[NUM_POWER_LEVELS] = { 235, 230, 224, 218, 211, 206, 201, 196, 190, 184, 178, 171, 164, 150, 136, 80 };
#endif
#ifdef _BOARD_HM_TRP_H_
	#define NUM_POWER_LEVELS 8
	__code static const uint8_t power_levels[NUM_POWER_LEVELS] = { 1, 2, 5, 8, 11, 14, 17, 20 };
#endif
#ifdef _BOARD_RF50_H
	#define NUM_POWER_LEVELS 8
	__code static const uint8_t power_levels[NUM_POWER_LEVELS] = { 1, 2, 5, 8, 11, 14, 17, 20 };
#endif

// set the radio transmit power (in dBm)
//
void 
radio_set_transmit_power(uint8_t power)
{
	uint8_t i;

#ifdef _BOARD_RFD900A
	register_write(EZRADIOPRO_TX_POWER, 6); // Set output power of Si1002 to 6 = +10dBm as a nominal level
	i = calibration_get(power);
	if (i != 0xFF)
	{
		PCA0CPH3 = i;     // Set PWM for PA to correct duty cycle
		settings.transmit_power = power;
	}
	else
	{
		i = power / POWER_LEVEL_STEP;
		PCA0CPH3 = power_levels[i];     // Set PWM for PA to correct duty cycle
		settings.transmit_power = i * POWER_LEVEL_STEP;
	}
#else
	for (i=0; i<NUM_POWER_LEVELS; i++) {
		if (power <= power_levels[i]) break;
	}
	if (i == NUM_POWER_LEVELS) {
		i = NUM_POWER_LEVELS-1;
	}
	settings.transmit_power = power_levels[i];
	register_write(EZRADIOPRO_TX_POWER, i);
#endif
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
	// when not using golay encoding we use the hardware
	// headers for network ID
	register_write(EZRADIOPRO_TRANSMIT_HEADER_3, id >> 8);
	register_write(EZRADIOPRO_TRANSMIT_HEADER_2, id & 0xFF);
	register_write(EZRADIOPRO_CHECK_HEADER_3, id >> 8);
	register_write(EZRADIOPRO_CHECK_HEADER_2, id & 0xFF);
}


/// write to a radio register
///
/// @param reg			The register to write
/// @param value		The value to write
///
static void
register_write(uint8_t reg, uint8_t value) __reentrant
{
	EX0_SAVE_DISABLE;

	NSS1 = 0;                           // drive NSS low
	SPIF1 = 0;                          // clear SPIF
	SPI1DAT = (reg | 0x80);             // write reg address
	while (!TXBMT1);                    // wait on TXBMT
	SPI1DAT = value;                    // write value
	while (!TXBMT1);                    // wait on TXBMT
	while ((SPI1CFG & 0x80) == 0x80);   // wait on SPIBSY

	SPIF1 = 0;                          // leave SPIF cleared
	NSS1 = 1;                           // drive NSS high

	EX0_RESTORE;
}


/// read from a radio register
///
/// @param reg			The register to read
/// @return			The value read
///
static uint8_t
register_read(uint8_t reg) __reentrant
{
	register uint8_t value;
	EX0_SAVE_DISABLE;

	NSS1 = 0;				// dsrive NSS low
	SPIF1 = 0;				// clear SPIF
	SPI1DAT = (reg);			// write reg address
	while (!TXBMT1);			// wait on TXBMT
	SPI1DAT = 0x00;				// write anything
	while (!TXBMT1);			// wait on TXBMT
	while ((SPI1CFG & 0x80) == 0x80);	// wait on SPIBSY
	value = SPI1DAT;			// read value
	SPIF1 = 0;				// leave SPIF cleared
	NSS1 = 1;				// drive NSS high

	EX0_RESTORE;

	return value;
}

/// read some bytes from the receive FIFO into a buffer
///
/// @param n			The number of bytes to read
static void
read_receive_fifo(register uint8_t n, __xdata uint8_t * buf) __reentrant
{
	NSS1 = 0;				// drive NSS low
	SPIF1 = 0;				// clear SPIF
	SPI1DAT = EZRADIOPRO_FIFO_ACCESS;
	while (!SPIF1);				// wait on SPIF
	ACC = SPI1DAT;				// discard first byte

	while (n--) {
		SPIF1 = 0;			// clear SPIF
		SPI1DAT = 0x00;			// write anything
		while (!SPIF1);			// wait on SPIF
		*buf++ = SPI1DAT;		// copy to buffer
	}

	SPIF1 = 0;				// leave SPIF cleared
	NSS1 = 1;				// drive NSS high
}

/// clear interrupts by reading the two status registers
///
static void
clear_status_registers(void)
{
	register_read(EZRADIOPRO_INTERRUPT_STATUS_1);
	register_read(EZRADIOPRO_INTERRUPT_STATUS_2);
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
	uint8_t status;

	// Clear interrupt enable and interrupt flag bits
	register_write(EZRADIOPRO_INTERRUPT_ENABLE_1, 0);
	register_write(EZRADIOPRO_INTERRUPT_ENABLE_2, 0);

	clear_status_registers();

	// SWReset
	register_write(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_1, (EZRADIOPRO_SWRES | EZRADIOPRO_XTON));

	// wait on any interrupt with a 2 MS timeout
	delay_set(2);
	while (IRQ) {
		if (delay_expired()) {
			return false;
		}
	}

	// enable chip ready interrupt
	register_write(EZRADIOPRO_INTERRUPT_ENABLE_1, 0);
	register_write(EZRADIOPRO_INTERRUPT_ENABLE_2, EZRADIOPRO_ENCHIPRDY);

	delay_set(20);
	while (!delay_expired()) {
		status = register_read(EZRADIOPRO_INTERRUPT_STATUS_1);
		status = register_read(EZRADIOPRO_INTERRUPT_STATUS_2);
		if (status & EZRADIOPRO_ICHIPRDY) {
			return true;
		}
	}
	return false;
}

/// set the radio frequency registers
///
/// @param frequency		The frequency to set, in Hz
static void
set_frequency_registers(__pdata uint32_t frequency)
{
	uint8_t band;
	__pdata uint16_t carrier;

	if (frequency > 480000000UL) {
		frequency -= 480000000UL;
		band  = frequency / 20000000UL;
		frequency -= (uint32_t)band * 20000000UL;
		frequency  = scale_uint32(frequency, 625);
		frequency <<= 1;
		band |= EZRADIOPRO_HBSEL;
	} else {
		frequency -= 240000000UL;
		band  = frequency / 10000000UL;
		frequency -= (uint32_t)band * 10000000UL;
		frequency  = scale_uint32(frequency, 625);
		frequency <<= 2;
	}

	band |= EZRADIOPRO_SBSEL;
	carrier = (uint16_t)frequency;

	register_write(EZRADIOPRO_FREQUENCY_BAND_SELECT, band);
	register_write(EZRADIOPRO_NOMINAL_CARRIER_FREQUENCY_1, carrier >> 8);
	register_write(EZRADIOPRO_NOMINAL_CARRIER_FREQUENCY_0, carrier & 0xFF);
}


/// return temperature in degrees C
///
/// @return		temperature in degrees C
///
int16_t
radio_temperature(void)
{
	register int16_t temp_local;

	AD0BUSY = 1;		// Start ADC conversion
	while (AD0BUSY) ;  	// Wait for completion of conversion

	temp_local = (ADC0H << 8) | ADC0L;
	temp_local *= 1.64060;  // convert reading into mV ( (val/1024) * 1680 )  vref=1680mV
	temp_local = 25.0 + (temp_local - 1025) / 3.4; // convert mV reading into degC.

	return temp_local;
}

/// Turn off radio diversity
///
void
radio_set_diversity(bool enable)
{
	if (enable)
	{
		register_write(EZRADIOPRO_GPIO2_CONFIGURATION, 0x18);
		// see table 23.8, page 279
		register_write(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_2, (register_read(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_2) & ~EZRADIOPRO_ANTDIV_MASK) | 0x80);
	}
	else
	{
		// see table 23.8, page 279
		register_write(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_2, (register_read(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_2) & ~EZRADIOPRO_ANTDIV_MASK));

		register_write(EZRADIOPRO_GPIO2_CONFIGURATION, 0x0A);	// GPIO2 (ANT1) output set high fixed
		register_write(EZRADIOPRO_IO_PORT_CONFIGURATION, 0x04);	// GPIO2 output set high (fixed on ant 1)
	}
}

/// the receiver interrupt
///
/// We expect to get the following types of interrupt:
///
///  - packet valid, when we have received a good packet
///   - CRC error, when a packet fails the CRC check
///   - preamble valid, when a packet has started arriving
///
INTERRUPT(Receiver_ISR, INTERRUPT_INT0)
{
	__data uint8_t status, status2;

	status2 = register_read(EZRADIOPRO_INTERRUPT_STATUS_2);
	status  = register_read(EZRADIOPRO_INTERRUPT_STATUS_1);

#if 0
	if (status != 0x20 || status2 != 0x52) {
		debug("status=%d status2=%d\n", (int)status, (int)status2);
	}
#endif
			
	if (status2 & EZRADIOPRO_IPREAVAL) {
		// a valid preamble has been detected
		preamble_detected = true;

		// read the RSSI register for logging
		last_rssi = register_read(EZRADIOPRO_RECEIVED_SIGNAL_STRENGTH_INDICATOR);
	}

	if (status & EZRADIOPRO_ICRCERROR) {
		debug("rx crcerror\n");
		goto rxfail;
	}

	if (status & EZRADIOPRO_IPKVALID) {
		__data uint8_t len = register_read(EZRADIOPRO_TRANSMIT_PACKET_LENGTH);
		debug("status=0x%x rxlen=%d\n", 
		      (unsigned)status,
		      (int)register_read(EZRADIOPRO_RECEIVED_PACKET_LENGTH));
		read_receive_fifo(len, &radio_buffer[0]);
		receive_packet_length = len;

		// we have a full packet
		packet_received = true;

		// disable interrupts until the tdm code has grabbed the packet
		register_write(EZRADIOPRO_INTERRUPT_ENABLE_1, 0);
		register_write(EZRADIOPRO_INTERRUPT_ENABLE_2, 0);

		// go into tune mode
		register_write(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_1, EZRADIOPRO_PLLON);
	}
	return;

rxfail:
	if (errors.rx_errors != 0xFFFF) {
		errors.rx_errors++;
	}
	radio_receiver_on();
}

