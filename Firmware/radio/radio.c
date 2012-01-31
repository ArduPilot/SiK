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
#include "board.h"
#include "radio.h"
#include "timer.h"
#include "golay.h"
#include "crc.h"

__xdata static uint8_t receive_buffer[MAX_AIR_PACKET_LENGTH];
__xdata static uint8_t receive_packet_length;
__xdata static uint8_t partial_packet_length;
__xdata static uint8_t last_rssi;
__xdata static uint8_t netid[2];

static volatile __bit packet_received;
static volatile __bit preamble_detected;

__xdata static struct {
	uint32_t frequency;
	uint32_t channel_spacing;
	uint32_t air_data_rate;
	uint8_t current_channel;
	uint8_t transmit_power;
} settings;


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

#define RADIO_RX_INTERRUPTS (EZRADIOPRO_ENRXFFAFULL|EZRADIOPRO_ENPKVALID)

// FIFO thresholds to allow for packets larger than 64 bytes
#define FIFO_THRESHOLD_LOW 50
#define FIFO_THRESHOLD_HIGH 50

static uint8_t num_bytes_different(__xdata uint8_t * __pdata buf1, 
				   __xdata uint8_t * __pdata buf2, 
				   uint8_t n)
{
	uint8_t ret = 0;
	while (n--) {
		if (*buf1++ != *buf2++) ret++;
	}
	return ret;
}

// return a received packet
//
// returns true on success, false on no packet available
//
bool
radio_receive_packet(uint8_t *length, __xdata uint8_t * __pdata buf)
{
	__xdata uint8_t gout[3];
	__pdata uint16_t crc1, crc2;
	EX0_SAVE_DISABLE;

	if (!packet_received) {
		EX0_RESTORE;
		return false;
	}

	if (receive_packet_length < 12 || (receive_packet_length%6) != 0) {
		// not a valid length
		printf("rx len invalid %u\n",
		       (unsigned)receive_packet_length);
		goto failed;
	}

	// decode and check the header
	golay_decode(6, receive_buffer, gout);
	if (gout[0] != netid[0] ||
	    gout[1] != netid[1]) {
		// its not for our network ID 
		printf("netid %x %x\n",
		       (unsigned)gout[0],
		       (unsigned)gout[1]);
		goto failed;
	}

	if (6*((gout[2]+2)/3+2) != receive_packet_length) {
		printf("rx len mismatch1 %u %u\n",
		       (unsigned)gout[2],
		       (unsigned)receive_packet_length);		
		goto failed;
	}

	// decode the CRC
	golay_decode(6, &receive_buffer[6], gout);
	crc1 = gout[0] | (((uint16_t)gout[1])<<8);

	if (6*((gout[2]+2)/3+2) != receive_packet_length) {
		printf("rx len mismatch2 %u %u\n",
		       (unsigned)gout[2],
		       (unsigned)receive_packet_length);		
		goto failed;
	}

	if (receive_packet_length != 12) {
		__xdata uint8_t buf2[MAX_AIR_PACKET_LENGTH-12];
		uint8_t n;
		golay_decode(receive_packet_length-12, &receive_buffer[12], buf);
		golay_encode((receive_packet_length-12)/2, buf, buf2);
		n = num_bytes_different(&receive_buffer[12], buf2, receive_packet_length-12);
		if (n != 0) {
			printf("fixed %u bytes\n", (unsigned)n);
		}
	}

	*length = gout[2];

	crc2 = crc16(*length, buf);

	if (crc1 != crc2) {
		printf("crc1=%x crc2=%x len=%u [%x %x]\n",
		       (unsigned)crc1, 
		       (unsigned)crc2, 
		       (unsigned)*length,
		       (unsigned)buf[0],
		       (unsigned)buf[1]);
		goto failed;
	}

	EX0_RESTORE;

	// get ready for the next packet
	radio_receiver_on();
	return true;

failed:
	if (errors.rx_errors != 255) {
		errors.rx_errors++;
	}
	EX0_RESTORE;
	radio_receiver_on();
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

// return the actual air data rate in BPS
//
uint32_t
radio_air_rate(void)
{
	return settings.air_data_rate;
}

// clear the transmit FIFO
//
static void
radio_clear_transmit_fifo(void)
{
	uint8_t control;
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
	__pdata uint16_t tstart, crc;
	bool transmit_started;
	uint8_t fifo_count;
	__xdata uint8_t gin[3];
	__xdata uint8_t gout[6];

	EX0_SAVE_DISABLE;

	radio_clear_transmit_fifo();

	register_write(EZRADIOPRO_TRANSMIT_PACKET_LENGTH, ((length+2)/3+2)*6);

	// start of packet is network ID and packet length
	gin[0] = netid[0];
	gin[1] = netid[1];
	gin[2] = length;

	// golay encode the header
	golay_encode(3, gin, gout);

	// put header in the FIFO
	radio_write_transmit_fifo(6, gout);
	fifo_count = 6;

	// next add a CRC, we round to 3 bytes for simplicity, adding 
	// another copy of the length in the spare byte
	crc = crc16(length, buf);
	gin[0] = crc&0xFF;
	gin[1] = crc>>8;
	gin[2] = length;

	// golay encode the CRC
	golay_encode(3, gin, gout);

	// put CRC in the FIFO
	radio_write_transmit_fifo(6, gout);
	fifo_count += 6;

	// push as many multiples of 3 input bytes as we can into the FIFO
	while (length > 0 && fifo_count < 60) {
		if (length > 3) {
			xmemcpy(gin, buf, 3);
			length -= 3;
		} else {
			xmemset(gin, 0, 3);
			xmemcpy(gin, buf, length);
			length = 0;
		}
		golay_encode(3, gin, gout);
		radio_write_transmit_fifo(6, gout);
		fifo_count += 6;
		buf += 3;
	}

	// enable the fifo low and error interrupts
	register_write(EZRADIOPRO_INTERRUPT_ENABLE_1, EZRADIOPRO_ENTXFFAEM | EZRADIOPRO_ENFFERR);
	register_write(EZRADIOPRO_INTERRUPT_ENABLE_2, 0);

	clear_status_registers();

	preamble_detected = 0;
	transmit_started = false;

	// start TX
	register_write(EZRADIOPRO_OPERATING_AND_FUNCTION_CONTROL_1, EZRADIOPRO_TXON | EZRADIOPRO_XTON);

	// to detect transmit completion, watch the chip power state
	// wait for the radio to first go into TX state, then out
	// again. This seems to be more reliable than waiting for the PKSENT
	// interrupt. About 2% of the time we don't see the PKSENT
	// interrupt at all even if the packet does get through
	tstart = timer2_tick();
	while ((uint16_t)(timer2_tick() - tstart) < timeout_ticks) {
		__data uint8_t status;

		status = register_read(EZRADIOPRO_INTERRUPT_STATUS_1);
		if (length != 0 && (status & EZRADIOPRO_ITXFFAEM)) {
			// the FIFO can fit some more bytes
			// push as many multiples of 3 input bytes as we can into the FIFO
			fifo_count = FIFO_THRESHOLD_LOW;
			while (length > 0 && fifo_count < 60) {
				if (length > 3) {
					xmemcpy(gin, buf, 3);
					length -= 3;
				} else {
					xmemset(gin, 0, 3);
					xmemcpy(gin, buf, 3);
					length = 0;
				}
				golay_encode(3, gin, gout);
				radio_write_transmit_fifo(6, gout);
				fifo_count += 6;
				buf += 3;
			}
			continue;
		}

		status = register_read(EZRADIOPRO_DEVICE_STATUS);
		if (status & 0x02) {
			// the chip power status is in TX mode
			transmit_started = true;
		}
		if (transmit_started && (status & 0x02) == 0) {
			// transmitter has finished, and is now in RX mode
			clear_status_registers();
			if (length != 0) {
				// we didn't send it all??
				printf("short TX %u\n",
				       (unsigned)length);
				if (errors.tx_errors != 255) {
					errors.tx_errors++;
				}
				register_write(EZRADIOPRO_INTERRUPT_ENABLE_1, 0);
				register_write(EZRADIOPRO_INTERRUPT_ENABLE_2, 0);
				EX0_RESTORE;
				return false;
			}
			register_write(EZRADIOPRO_INTERRUPT_ENABLE_1, 0);
			register_write(EZRADIOPRO_INTERRUPT_ENABLE_2, 0);
			EX0_RESTORE;
			return true;			
		}
	}

	clear_status_registers();
	// transmit timeout ... clear the FIFO
#if 1
	printf("%u ts=%u tn=%u len=%u\n",
	       timeout_ticks,
	       tstart,
	       timer2_tick(),
	       (unsigned)length);
#endif
	if (errors.tx_errors != 255) {
		errors.tx_errors++;
	}

	register_write(EZRADIOPRO_INTERRUPT_ENABLE_1, 0);
	register_write(EZRADIOPRO_INTERRUPT_ENABLE_2, 0);
	EX0_RESTORE;
	return false;
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
	partial_packet_length = 0;

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

	SDN = 0;

	delay_msec(25);

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
	delay_set(10);
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
	if (value < 240000000L || value > 930000000L) {
		return false;
	}
	settings.frequency = value;
	set_frequency_registers(value);
	return true;
}


// set the channel spacing
//
bool
radio_set_channel_spacing(__pdata uint32_t value)
{
	if (value > 2550000L)
		return false;
	value = scale_uint32(value, 10000);
	settings.channel_spacing = value;
	register_write(EZRADIOPRO_FREQUENCY_HOPPING_STEP_SIZE, value);
	return true;
}

// set the tx/rx frequency channel
//
void
radio_set_channel(uint8_t channel)
{
	if (channel != settings.current_channel) {
		settings.current_channel = channel;
		register_write(EZRADIOPRO_FREQUENCY_HOPPING_CHANNEL_SELECT, channel);
		preamble_detected = 0;
	}
}

// get the tx/rx frequency channel
//
uint8_t radio_get_channel(void)
{
	return settings.current_channel;
}


// This table gives the register settings for the radio core indexed by
// the desired air data rate.
//
// the data for this table is based on the OpenPilot rfm22b driver
//
// Note that air rates below 1000 bps won't work with the current TDM scheme
//
#define NUM_DATA_RATES 14
#define NUM_RADIO_REGISTERS 16

// the minimum speed allowed ensures that we don't go over the 0.4s
// transmit time limit
#define MIN_SPEED_ALLOWED 1000

// the 256000 rate doesn't currently work
#define MAX_SPEED_ALLOWED 192000

__code static const uint32_t air_data_rates[NUM_DATA_RATES] = {
	500,  1000,  2000,  4000,  8000,  9600, 16000, 19200, 24000,  32000,  64000, 128000, 192000, 256000
};

__code static const uint8_t reg_table[NUM_RADIO_REGISTERS][1 + NUM_DATA_RATES] = {
	// first column is the register number
	// remaining columns are the values for the corresponding air data rate
	{
		EZRADIOPRO_IF_FILTER_BANDWIDTH,
		0x37,  0x37,  0x37,  0x37,  0x3A,  0x3B,  0x26,  0x28,  0x2E,   0x16,   0x07,   0x83,   0x8A,  0x8C
	},
	{
		EZRADIOPRO_AFC_LOOP_GEARSHIFT_OVERRIDE,
		0x44,  0x44,  0x44,  0x44,  0x44,  0x44,  0x44,  0x44,  0x44,   0x44,   0x44,   0x44,   0x44,  0x44
	},
	{
		EZRADIOPRO_AFC_TIMING_CONTROL,
		0x0A,  0x0A,  0x0A,  0x0A,  0x0A,  0x0A,  0x0A,  0x0A,  0x0A,   0x0A,   0x0A,   0x0A,   0x0A,  0x02
	},
	{
		EZRADIOPRO_CLOCK_RECOVERY_GEARSHIFT_OVERRIDE,
		0x03,  0x03,  0x03,  0x03,  0x03,  0x03,  0x03,  0x03,  0x03,   0x03,   0x03,   0x03,   0x03,  0x03
	},
	{
		EZRADIOPRO_CLOCK_RECOVERY_OVERSAMPLING_RATIO,
		0xE8,  0xF4,  0xFA,  0x70,  0x3F,  0x34,  0x3F,  0x34,  0x2A,   0x3F,   0x3F,   0x5E,   0x3F,  0x2f
	},
	{
		EZRADIOPRO_CLOCK_RECOVERY_OFFSET_2,
		0x60,  0x20,  0x00,  0x01,  0x02,  0x02,  0x02,  0x02,  0x03,   0x02,   0x02,   0x01,   0x02,  0x02
	},
	{
		EZRADIOPRO_CLOCK_RECOVERY_OFFSET_1,
		0x20,  0x41,  0x83,  0x06,  0x0C,  0x75,  0x0C,  0x75,  0x12,   0x0C,   0x0C,   0x5D,   0x0C,  0xBB
	},
	{
		EZRADIOPRO_CLOCK_RECOVERY_OFFSET_0,
		0xC5,  0x89,  0x12,  0x25,  0x4A,  0x25,  0x4A,  0x25,  0x6F,   0x4A,   0x4A,   0x86,   0x4A,  0x0D
	},
	{
		EZRADIOPRO_CLOCK_RECOVERY_TIMING_LOOP_GAIN_1,
		0x00,  0x00,  0x00,  0x02,  0x07,  0x07,  0x07,  0x07,  0x07,   0x07,   0x07,   0x05,   0x07,  0x07
	},
	{
		EZRADIOPRO_CLOCK_RECOVERY_TIMING_LOOP_GAIN_0,
		0x0A,  0x23,  0x85,  0x0E,  0xFF,  0xFF,  0xFF,  0xFF,  0xFF,   0xFF,   0xFF,   0x74,   0xFF,  0xFF
	},
	{
		EZRADIOPRO_AFC_LIMITER,
		0x0E,  0x0E,  0x0E,  0x0E,  0x0E,  0x0D,  0x0D,  0x0E,  0x12,   0x17,   0x31,   0x50,   0x50,  0x50
	},
	{
		EZRADIOPRO_TX_DATA_RATE_1,
		0x04,  0x08,  0x10,  0x20,  0x41,  0x4E,  0x83,  0x9D,  0xC4,   0x08,   0x10,   0x20,   0x31,  0x41
	},
	{
		EZRADIOPRO_TX_DATA_RATE_0,
		0x19,  0x31,  0x62,  0xC5,  0x89,  0xA5,  0x12,  0x49,  0x9C,   0x31,   0x62,   0xC5,   0x27,  0x89
	},
	{
		EZRADIOPRO_MODULATION_MODE_CONTROL_1,
		0x2D,  0x2D,  0x2D,  0x2D,  0x2D,  0x2D,  0x2D,  0x2D,  0x2D,   0x0D,   0x0D,   0x0D,   0x0D,  0x0D
	},
	{
		EZRADIOPRO_MODULATION_MODE_CONTROL_2,
		0x23,  0x23,  0x23,  0x23,  0x23,  0x23,  0x23,  0x23,  0x23,   0x23,   0x23,   0x23,   0x23,  0x23
	},
	{
		EZRADIOPRO_FREQUENCY_DEVIATION,
		0x06,  0x06,  0x06,  0x06,  0x06,  0x08,  0x0D,  0x0F,  0x13,   0x1A,   0x33,   0x66,   0x9A,  0xCD
	}
};


// configure radio based on the air data rate
//
bool
radio_configure(__pdata uint32_t air_rate)
{
	uint8_t i, rate_selection;

	if (air_rate < MIN_SPEED_ALLOWED) {
		air_rate = MIN_SPEED_ALLOWED;
	}
	if (air_rate > MAX_SPEED_ALLOWED) {
		air_rate = MAX_SPEED_ALLOWED;
	}

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
	register_write(EZRADIOPRO_GPIO2_CONFIGURATION, 0x14);	// RX data (output)
#else
	//set GPIOx to GND
	register_write(EZRADIOPRO_GPIO0_CONFIGURATION, 0x14);	// RX data (output)
	register_write(EZRADIOPRO_GPIO1_CONFIGURATION, 0x14);	// RX data (output)
	register_write(EZRADIOPRO_GPIO2_CONFIGURATION, 0x14);	// RX data (output)
#endif

	// set capacitance
	register_write(EZRADIOPRO_CRYSTAL_OSCILLATOR_LOAD_CAPACITANCE, EZRADIOPRO_OSC_CAP_VALUE);

	// see Si4432V2Errata.pdf point 9
	register_write(EZRADIOPRO_DIVIDER_CURRENT_TRIMMING, 0x00);
	register_write(EZRADIOPRO_VCO_CURRENT_TRIMMING, 0x03);
	register_write(EZRADIOPRO_LDO_LEVEL_SETTING, 0x02);

	// see Si4432V2Errata.pdf point 10
	register_write(EZRADIOPRO_LDO_CONTROL_OVERRIDE, 0xA1);

	// see Si4432V2Errata.pdf point 18
	if (air_rate > 100000UL) {
		register_write(EZRADIOPRO_CHARGEPUMP_CURRENT_TRIMMING_OVERRIDE, 0xC0);
	}

	// setup frequency and channel spacing
	set_frequency_registers(settings.frequency);
	register_write(EZRADIOPRO_FREQUENCY_HOPPING_STEP_SIZE, settings.channel_spacing);

	// no CRC
	register_write(EZRADIOPRO_DATA_ACCESS_CONTROL,
	               EZRADIOPRO_ENPACTX | EZRADIOPRO_ENPACRX);


	// set FIFO limits to allow for sending larger than 64 byte packets
	register_write(EZRADIOPRO_TX_FIFO_CONTROL_1, 0x3F);
	register_write(EZRADIOPRO_TX_FIFO_CONTROL_2, FIFO_THRESHOLD_LOW);
	register_write(EZRADIOPRO_RX_FIFO_CONTROL, FIFO_THRESHOLD_HIGH);

	// preamble setup
	register_write(EZRADIOPRO_PREAMBLE_LENGTH, 10); // 10 nibbles 
	register_write(EZRADIOPRO_PREAMBLE_DETECTION_CONTROL, 5<<3); //  5 nibbles

	// 2 sync bytes and no header bytes
	register_write(EZRADIOPRO_HEADER_CONTROL_2, EZRADIOPRO_HDLEN_0BYTE | EZRADIOPRO_SYNCLEN_2BYTE);

	// no header check
	register_write(EZRADIOPRO_HEADER_CONTROL_1, 0x00);

	// setup minimum output power during startup
	radio_set_transmit_power(0);

	// work out which register table column we will use
	for (i = 0; i < NUM_DATA_RATES - 1; i++) {
		if (air_data_rates[i] >= air_rate) break;
	}
	rate_selection = i;

	settings.air_data_rate = air_data_rates[rate_selection];

	// set the registers from the table
	for (i = 0; i < NUM_RADIO_REGISTERS; i++) {
		register_write(reg_table[i][0],
		               reg_table[i][rate_selection + 1]);
	}

	return true;
}

#define NUM_POWER_LEVELS 8
__code static const uint8_t power_levels[NUM_POWER_LEVELS] = { 1, 2, 5, 8, 11, 14, 17, 20 };

// set the radio transmit power (in dBm)
//
void radio_set_transmit_power(uint8_t power)
{
	uint8_t i;
	for (i=0; i<NUM_POWER_LEVELS; i++) {
		if (power <= power_levels[i]) break;
	}
	if (i == NUM_POWER_LEVELS) {
		i = NUM_POWER_LEVELS-1;
	}
	settings.transmit_power = power_levels[i];
	register_write(EZRADIOPRO_TX_POWER, i);
}

// get the currend transmit power (in dBm)
//
uint8_t radio_get_transmit_power(void)
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

/// read some bytes from the receive FIFO into receive_buffer[]
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
	return (value + (value >> 1)) / scale;
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

	if (status & EZRADIOPRO_IRXFFAFULL) {
		if (partial_packet_length + FIFO_THRESHOLD_HIGH > MAX_AIR_PACKET_LENGTH) {
			printf("rx pplen=%u\n", (unsigned)partial_packet_length);
			goto rxfail;
		}
		read_receive_fifo(FIFO_THRESHOLD_HIGH, &receive_buffer[partial_packet_length]);
		partial_packet_length += FIFO_THRESHOLD_HIGH;
	}

	if (status2 & EZRADIOPRO_IPREAVAL) {
		// a valid preamble has been detected
		preamble_detected = true;

		// read the RSSI register for logging
		last_rssi = register_read(EZRADIOPRO_RECEIVED_SIGNAL_STRENGTH_INDICATOR);
	}

	if (status & EZRADIOPRO_IPKVALID) {
		__data uint8_t len = register_read(EZRADIOPRO_RECEIVED_PACKET_LENGTH);
		if (len > MAX_AIR_PACKET_LENGTH || partial_packet_length > len) {
			printf("rx len=%u\n", (unsigned)len);
			goto rxfail;
		}
		if (partial_packet_length < len) {
			read_receive_fifo(len-partial_packet_length, &receive_buffer[partial_packet_length]);
		}
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
	if (errors.rx_errors != 255) {
		errors.rx_errors++;
	}
	radio_receiver_on();
}

