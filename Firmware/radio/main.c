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

///
/// @file	_start.c
///
/// Early startup code.
/// This file *must* be linked first for interrupt vector generation and main() to work.
/// XXX this may no longer be the case - it may be sufficient for the interrupt vectors
/// to be located in the same file as main()
///

#include <stdarg.h>

#include "radio.h"

////////////////////////////////////////////////////////////////////////////////
/// @name	Interrupt vector prototypes
///
/// @note these *must* be placed in this file for SDCC to generate the
/// interrupt vector table correctly.
//@{

/// Serial rx/tx interrupt handler.
///
extern void	serial_interrupt(void)	__interrupt(INTERRUPT_UART0) __using(1);

/// Radio event interrupt handler.
///
extern void	Receiver_ISR(void)	__interrupt(INTERRUPT_INT0);

/// Timer tick interrupt handler
///
/// @todo switch this and everything it calls to use another register bank?
///
static void	T3_ISR(void)		__interrupt(INTERRUPT_TIMER3);

//@}

__code const char g_banner_string[] = "SiK " stringify(APP_VERSION_HIGH) "." stringify(APP_VERSION_LOW) " on " BOARD_NAME;
__code const char g_version_string[] = stringify(APP_VERSION_HIGH) "." stringify(APP_VERSION_LOW);

__pdata enum BoardFrequency	g_board_frequency;	///< board info from the bootloader
__pdata uint8_t			g_board_bl_version;	///< from the bootloader

/// Counter used by delay_msec
///
static volatile uint8_t delay_counter;

// base tick counter. All of the TDM window calculations
// are based on this. It runs at 200Hz
static volatile uint8_t tick_counter;

// how many ticks are remaining in our transmit window
// this will be zero when we are receiving. We send this value in
// header byte 3 of every packet
static volatile uint8_t tx_window_remaining;

// the tick value of the start of our next transmit window. This is
// adjusted based on the header of any incoming packet, to keep the
// two radios in sync
static volatile uint8_t next_tx_window;

// the number of ticks we grab for each transmit window
// this is enough to hold at least 3 packets and is based 
// on the configured air data rate
static uint8_t tx_window_width;

// the silence period between transmit windows. This is calculated as
// the number of ticks it would take to transmit a full sized packet
static uint8_t silence_period;

// activity indication. When tick_counter wraps we 
// check if we have received a packet in the last 1.25seconds. If we
// have the green radio LED is held on. Otherwise it blinks every 1.25
// seconds. The received_packet flag is set for any received packet,
// whether it contains user data or not
static __bit blink_state;
static volatile __bit received_packet;

// we prefer to send packets in TX_CHUNK_SIZE byte chunks when 
// possible
#define TX_CHUNK_SIZE 64

// at startup we calculate how many milliticks a byte is expected
// to take to transmit with the configured air data rate. This is used
// to calculate the flight time of an incoming packet
static uint16_t milliticks_per_byte;

// how many bytes can we safely transmit in a single tick?
static uint8_t bytes_per_tick;

// number of ticks to wait for a preamble to turn into a packet. This
// is set when we get a preamble interrupt, and causes us to delay
// sending for the silence_period. This is used to make it more likely
// that two radios that happen to be exactly in sync in their sends
// will eventually get a packet through and get their transmit windows
// sorted out
static uint8_t preamble_wait;

// the overhead in bytes of a packet. It consists of 2 settle bytes, 5
// preamble bytes, 2 sync bytes, 3 header bytes, 2 CRC bytes
#define PACKET_OVERHEAD 14

/// Configure the Si1000 for operation.
///
static void hardware_init(void);

/// Initialise the radio and bring it online.
///
static void radio_init(void);

/*
  synchronise tx windows

  we receive a 8 bit header with each packet which indicates how many
  more ticks the sender has in their transmit window

  The job of this function is to adjust our own transmit window to
  match the other radio
 */
static void sync_tx_windows(uint8_t rxheader, uint8_t packet_length)
__critical {
	uint8_t flight_time = (512 + ((packet_length + PACKET_OVERHEAD) * milliticks_per_byte))>>10;
	uint8_t old_tx_window = next_tx_window;

	if (rxheader > tx_window_width) {
		// the other radio has more ticks than is usually
		// allowed, so must be using yielded ticks from us. To
		// prevent a storm of yields we just return now
		return;
	} else if (rxheader >= flight_time) {
		// we are still in the other radios transmit
		// window. We can adjust our transmit window
		next_tx_window = tick_counter + (rxheader - flight_time) + silence_period;
		tx_window_remaining = 0;
	} else if (flight_time - rxheader < silence_period) {
		// we're in the silence period between windows. Adjust
		// the transmit window, but don't start transmitting
		// just yet
		next_tx_window = tick_counter + silence_period - (flight_time - rxheader);
		tx_window_remaining = 0;
	} else {
		// we are in our transmit window. 
		tx_window_remaining = tx_window_width - (flight_time - rxheader);
		next_tx_window = tick_counter + tx_window_remaining + tx_window_width + silence_period*2;
	}

#if 0
	{
	uint8_t window_change;
	window_change = old_tx_window - next_tx_window;
	if (window_change > 1 && window_change != 255) {
		printf("otx=%d ntx=%d rx=%d ft=%d pl=%d\n",
		       (int)old_tx_window,
		       (int)next_tx_window,
		       (int)rxheader,
		       (int)flight_time,
		       (int)packet_length);
	}
	}
#endif

	// if the other end has sent a zero length packet and we don't
	// currently have any transmit window remaining then they are
	// yielding some ticks to us. 
	if (packet_length == 0 && tx_window_remaining == 0) {
		tx_window_remaining = (next_tx_window - tick_counter) + tx_window_width;
	}
}


// the main loop for the TDM based 
// transparent serial implementation
static void transparent_serial_loop(void)
{
	// the number of bytes currently in the send fifo
	uint8_t tx_fifo_bytes = 0;
	// a tick count when we will send a short packet
	uint8_t force_send_time = 0;

	for (;;) {
		__pdata uint8_t	rlen;
		__xdata uint8_t	rbuf[64];
		uint16_t slen;
		uint8_t rxheader;

		// see if we have received a packet
		if (radio_receive_packet(&rlen, rbuf, &rxheader)) {
			// sync our transmit windows based on
			// received header
			sync_tx_windows(rxheader, rlen);

			// updte the activity indication
			received_packet = 1;

			// we're not waiting for a preamble
			// any more
			preamble_wait = 0;

			if (rlen != 0) {
				//printf("rcv(%d,[", rlen);
				LED_ACTIVITY = LED_ON;
				serial_write_buf(rbuf, rlen);
				LED_ACTIVITY = LED_OFF;
				//printf("]\n");
			}
			continue;
		}

		// give the AT command processor a chance to handle a command
		at_command();

		// if we have received something via serial see how
		// much of it we could fit in the transmit FIFO
		slen = serial_read_available();
		if (slen + tx_fifo_bytes > 64) {
			slen = 64 - tx_fifo_bytes;
		}
		if (slen > 0 && serial_read_buf(rbuf, slen)) {
			// put it in the send FIFO
			radio_write_transmit_fifo(slen, rbuf);
			tx_fifo_bytes += slen;
		}

		if (tx_window_remaining * (uint16_t)bytes_per_tick < tx_fifo_bytes+PACKET_OVERHEAD) {
			// we can't fit the whole fifo in our remaining
			// window, so start receiving instead
			continue;
		}

		if (preamble_wait > 0) {
			// we saw a preamble previously and are now
			// waiting for a possible packet
			continue;
		}

		if (radio_preamble_detected()) {
			// a preamble has been detected. Don't
			// transmit for a while
			preamble_wait = silence_period;
			continue;
		}

		if (tx_fifo_bytes >= TX_CHUNK_SIZE || 
		    tick_counter >= force_send_time) {
			// we're at least half full or we have not
			// received any more serial bytes for at least
			// 1 tick, send now
			if (tx_fifo_bytes != 0) {
				// show the user that we're sending data
				LED_ACTIVITY = LED_ON;
			}

			// start transmitting the packet
			radio_transmit_start(tx_fifo_bytes, tx_window_remaining, tx_window_remaining+silence_period);
			if (tx_fifo_bytes == 0) {
				// sending a zero byte packet gives up
				// our window, but doesn't change the
				// start of the next window
				tx_window_remaining = 0;
			}

			// re-enable the receiver
			radio_receiver_on();

			// clear the transmit FIFO. This shouldn't
			// actually be needed, but I have seen some
			// strange situations where the FIFO gets out
			// of sync
			radio_clear_transmit_fifo();
			if (tx_fifo_bytes != 0) {
				LED_ACTIVITY = LED_OFF;
			}
			tx_fifo_bytes = 0;
			force_send_time = tick_counter+1;
			continue;
		}

		// mark a time when we will send regardless of how
		// many bytes we have in the FIFO
		force_send_time = tick_counter+1;
	}
}


void
main(void)
{
	// Stash board info from the bootloader before we let anything touch
	// the SFRs.
	//
	g_board_frequency = BOARD_FREQUENCY_REG;
	g_board_bl_version = BOARD_BL_VERSION_REG;

	// try to load parameters; set them to defaults if that fails.
	// this is done before hardware_init() to get the serial speed
	// XXX default parameter selection should be based on board info
	//
	if (!param_load())
		param_default();

	// Do hardware initialisation.
	hardware_init();

	// do radio initialisation
	radio_init();

	// turn on the receiver
	if (!radio_receiver_on()) {
		panic("failed to enable receiver");
	}

	transparent_serial_loop();
}

void
panic(char *fmt, ...)
{
	va_list ap;

	puts("\n**PANIC**");
	va_start(ap, fmt);
	vprintf(fmt, ap);
	puts("");
	for (;;)
		;
}

static void
hardware_init(void)
{
	__pdata uint16_t	i;

	// Disable the watchdog timer
	PCA0MD	&= ~0x40;

	// Select the internal oscillator, prescale by 1
	FLSCL	 =  0x40;
	OSCICN	 =  0x8F;
	CLKSEL	 =  0x00;

	// Configure the VDD brown out detector
	VDM0CN	 =  0x80;
	for (i = 0; i < 350; i++);	// Wait 100us for initialization
	RSTSRC	 =  0x06;		// enable brown out and missing clock reset sources

	// Configure crossbar for UART
	P0MDOUT	 =  0x10;		// UART Tx push-pull
	SFRPAGE	 =  CONFIG_PAGE;
	P0DRV	 =  0x10;		// UART TX
	SFRPAGE	 =  LEGACY_PAGE;
	XBR0	 =  0x01;		// UART enable

	// SPI1
	XBR1	|= 0x40;	// enable SPI in 3-wire mode
	P1MDOUT	|= 0x15;	// SCK1, MOSI1, MISO1 push-pull
	SFRPAGE	 = CONFIG_PAGE;
	P1DRV	|= 0x15;	// SPI signals use high-current mode
	SFRPAGE	 = LEGACY_PAGE;
	SPI1CFG	 = 0x40;	// master mode
	SPI1CN	 = 0x00;	// 3 wire master mode
	SPI1CKR	 = 0x00;	// Initialise SPI prescaler to divide-by-2 (12.25MHz, technically out of spec)
	SPI1CN	|= 0x01;	// enable SPI
	NSS1	 = 1;		// set NSS high

	// Clear the radio interrupt state
	IE0	 = 0;

	// 200Hz timer tick using timer3
	// Derive timer values from SYSCLK, just for laughs.
	TMR3RLL	 = (65536UL - ((SYSCLK / 12) / 200)) & 0xff;
	TMR3RLH	 = ((65536UL - ((SYSCLK / 12) / 200)) >> 8) & 0xff;
	TMR3CN	 = 0x04;	// count at SYSCLK / 12 and start
	EIE1	|= 0x80;

	// UART - set the configured speed
	serial_init(param_get(PARAM_SERIAL_SPEED));

	// global interrupt enable
	EA = 1;

	// Turn on the 'radio running' LED and turn off the bootloader LED
	LED_RADIO = LED_ON;
	LED_BOOTLOADER = LED_OFF;

	XBR2	 =  0x40;		// Crossbar (GPIO) enable
}

static void
radio_init(void)
{
	__pdata uint32_t	freq;

	// Do generic PHY initialisation
	if (!radio_initialise()) {
		panic("radio_initialise failed");
	}

	switch (g_board_frequency) {
	case FREQ_433:
		freq = 433000000UL;
		break;
	case FREQ_470:
		freq = 470000000UL;
		break;
	case FREQ_868:
		freq = 868000000UL;
		break;
	case FREQ_915:
		freq = 915000000UL;
		break;
	default:
		panic("bad board frequency %d", g_board_frequency);
		freq = 0;	// keep the compiler happy
		break;
	}

	// set the frequency and channel spacing
	radio_set_frequency(freq);
	radio_set_channel_spacing(100000UL);
	
	// And intilise the radio with them.
	if (!radio_configure(param_get(PARAM_AIR_SPEED)*1000UL)) {
		panic("radio_configure failed");
	}

	// setup network ID
	radio_set_network_id(param_get(PARAM_NETID));

	// work out how many milliticks a byte takes to come over the air
	milliticks_per_byte = 204800UL / (param_get(PARAM_AIR_SPEED)*125UL);

	// work out how many bytes we can safely transmit in one tick
	bytes_per_tick = 1024 / milliticks_per_byte;
	if (bytes_per_tick == 0) {
		bytes_per_tick = 1;
	}

	// work out the default transmit window in ticks
	tx_window_width = (3 * (TX_CHUNK_SIZE + PACKET_OVERHEAD)) / bytes_per_tick;
	if (tx_window_width < 3) {
		tx_window_width = 3;
	}

	// work out how long neither end will transmit for between windows
	silence_period = (TX_CHUNK_SIZE + PACKET_OVERHEAD) / bytes_per_tick;
	if (silence_period < 1) {
		silence_period = 1;
	}
}

///
/// Table of supported serial speed settings.
///
const __code U8 serial_baud_rates[][2] = {
	// TH1, CKCON
	{0x96, 0x00},	// B9600
	{0x60, 0x01},	// B19200
	{0xb0, 0x01},	// B38400
	{0x2b, 0x08},	// B57600
	{0x96, 0x08},	// B115200
	{0xcb, 0x08},	// B230400
};

void
serial_device_set_speed(uint8_t speed)
{
	if (speed < BMAX) {
		TH1 = serial_baud_rates[speed][0];
		CKCON = (CKCON & ~0x0b) | serial_baud_rates[speed][1];
	}
}

// blink the radio LED if we have not received any packets
static void link_update(void)
{
	if (received_packet) {
		LED_RADIO = LED_ON;
		received_packet = 0;
	} else {
		LED_RADIO = blink_state;
		blink_state = !blink_state;

		// randomise the next transmit window using the signal strength
		next_tx_window += radio_last_rssi() & 0x1F;
	}
}

static void
T3_ISR(void) __interrupt(INTERRUPT_TIMER3)
{

	// re-arm the interrupt
	TMR3CN = 0x04;

	// call the AT parser tick
	at_timer();

	tick_counter++;

	if (tick_counter == 0) {
		link_update();
	}

	// update the delay counter
	if (delay_counter > 0)
		delay_counter--;

	if (preamble_wait > 0)
		preamble_wait--;

	// update transmit windows
	if (tx_window_remaining > 0) {
		tx_window_remaining--;
	}
	if (tick_counter == next_tx_window) {
		if (tx_window_remaining < tx_window_width) {
			tx_window_remaining = tx_window_width;
		}
		next_tx_window += 2*(tx_window_width + silence_period);
	}
}

void
delay_set(uint16_t msec)
{
	if (msec >= 1250) {
		delay_counter = 255;
	} else {
		delay_counter = (msec + 4) / 5;
	}
}

void delay_set_ticks(uint8_t ticks)
{
	delay_counter = ticks;
}

bool
delay_expired()
{
	return delay_counter == 0;
}

void
delay_msec(uint16_t msec)
{
	delay_set(msec);
	while (!delay_expired())
		;
}
