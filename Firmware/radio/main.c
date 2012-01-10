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
static volatile uint8_t tick_counter;

// state of the TDM system
typedef union {
	struct {
		uint8_t slice_counter:6;
		uint8_t yield_timeslice:1;
		uint8_t am_odd_transmitter:1;
	} bits;
	uint8_t v;
} tdm_state_t;

#define SLICE_COUNTER_MASK 0x3F

static tdm_state_t tdm_state, other_tdm_state;
static uint8_t slice_counter_shift = 0;

// activity indication. When tick_counter wraps we 
// check if we have received a packet in the last 1.25seconds. If we
// have the radio light is solid. Otherwise it blinks
static __bit blink_state;
static volatile __bit received_packet;

// prefer to send packets in TX_CHUNK_SIZE byte chunks when 
#define TX_CHUNK_SIZE 32

// at startup we calculate how many milliticks a byte is expected
// to take to transmit with the configured air data rate
static uint16_t milliticks_per_byte;


/// Configure the Si1000 for operation.
///
static void hardware_init(void);

/// Initialise the radio and bring it online.
///
static void radio_init(void);

/*
  synchronise tx windows

  we receive a 8 bit header with each packet. See the tdm_state above
  for the format
 */
static void sync_tx_windows(uint8_t rxheader, uint8_t packet_length)
{
	uint8_t flight_time, other_tick_counter;

	other_tdm_state.v = rxheader;

	// update if we are the odd transmitter
	if (tdm_state.bits.am_odd_transmitter == other_tdm_state.bits.am_odd_transmitter) {
		tdm_state.bits.am_odd_transmitter = other_tdm_state.bits.am_odd_transmitter ^ 1;
	}

	// 5 preamble bytes, 2 sync bytes, 3 header bytes, 2 CRC bytes
	packet_length += (5 + 2 + 3 + 2);

	// work out how long this packet took to deliver to us
	flight_time = (packet_length * milliticks_per_byte) >> 10;
	other_tick_counter = (other_tdm_state.bits.slice_counter + flight_time) & SLICE_COUNTER_MASK;

	// possibly update our slice_counter to match the other end. We
	// allow for us to be 1 ahead, as we could have had the timer
	// interrupt since the packet was sent. This should keep the
	// two tick counters within 1 tick of each other. The sync
	// slices cope with that.
	if (tdm_state.bits.slice_counter != other_tick_counter &&
	    tdm_state.bits.slice_counter != ((other_tick_counter+1)&SLICE_COUNTER_MASK)) {
		tdm_state.bits.slice_counter = other_tick_counter;
		EA = 0;
		tick_counter = (uint8_t)tdm_state.bits.slice_counter;
		EA = 1;	
	}
}

/*
  see if its our turn to transmit

  We have 8 transmit slots. The first 3 are for the odd
  transmitter. The 4th one is a sync slot (no transmitter).  The next
  3 are for the even transmitter, and the final one is another sync
  slot
 */
static bool tx_window_open(void)
{
	enum {OUR_SLICE, OUR_SYNC, THEIR_SLICE, THEIR_SYNC} slice;

	switch ((tdm_state.bits.slice_counter >> slice_counter_shift) & 0x7) {
	case 0:
	case 1:
	case 2:
		// odd slice
		if (tdm_state.bits.am_odd_transmitter) {
			slice = OUR_SLICE;
		} else {
			slice = THEIR_SLICE;
		}
		break;
	case 3:
		if (tdm_state.bits.am_odd_transmitter) {
			slice = OUR_SYNC;
		} else {
			slice = THEIR_SYNC;
		}		
		break;
	case 4:
	case 5:
	case 6:
		if (tdm_state.bits.am_odd_transmitter) {
			slice = THEIR_SLICE;
		} else {
			slice = OUR_SLICE;
		}				
		break;
	default:
		if (tdm_state.bits.am_odd_transmitter) {
			slice = THEIR_SYNC;
		} else {
			slice = OUR_SYNC;
		}		
		break;
	}

	// clean our yield flag if its not our slice
	if (slice != OUR_SLICE && tdm_state.bits.yield_timeslice == 1) {
		tdm_state.bits.yield_timeslice = 0;
	}


	if (slice == OUR_SYNC) {
		// never transmit in our own sync slice
		return false;
	}
	
	if (slice == THEIR_SLICE || slice == THEIR_SYNC) {
		// we can transmit in their slice or sync slot if they
		// have set their yield bit
		return (other_tdm_state.bits.yield_timeslice == 1);
	}

	// otherwise its our slice

	// possibly clear their yield flag
	if (other_tdm_state.bits.yield_timeslice == 1) {
		other_tdm_state.bits.yield_timeslice = 0;
	}

	// we can transmit in our slice slot if we
	// have not set our yield bit
	return (tdm_state.bits.yield_timeslice == 0);
}

/*
  calculate the 8 bit transmit header
 */
static uint8_t tx_header(void)
{
	return tdm_state.v;
}

/*
  send some bytes out the radio, coalescing into uniformly sized
  chunks when possible
 */
static void send_bytes(uint8_t len, __xdata uint8_t *buf)
{
	static uint8_t tx_fifo_bytes;
	static uint8_t tx_last_tick;
	uint8_t ofs = 0;
	bool done_send = false;

	if (len == 0 && 
	    tx_fifo_bytes == 0 && 
	    tdm_state.bits.yield_timeslice == 0 &&
	    other_tdm_state.bits.yield_timeslice == 0) {
		// we have no use for our timeslice, give it up using
		// a zero byte packet with the yield bit set
		tdm_state.bits.yield_timeslice = 1;
		rtPhyTxStart(0, tx_header());
		rtPhyClearTxFIFO();
		rtPhyRxOn();
		return;
	}

	// try to send as TX_CHUNK_SIZE packets. This keeps the
	// overhead down, as every chunk also gets 2 sync bytes
	// a header byte and 2 bytes of CRC
	while (len + tx_fifo_bytes >= TX_CHUNK_SIZE) {
		uint8_t chunk = TX_CHUNK_SIZE - tx_fifo_bytes;
		phyWriteFIFO(chunk, &buf[ofs]);
		rtPhyTxStart(TX_CHUNK_SIZE, tx_header());
		rtPhyClearTxFIFO();
		tx_last_tick = tick_counter;
		done_send = true;
		len -= chunk;
		ofs += chunk;
		tx_fifo_bytes = 0;
	}
	if (len > 0) {
		phyWriteFIFO(len, &buf[ofs]);
		tx_last_tick = tick_counter;
		tx_fifo_bytes += len;
	}

	// don't let stray bytes sit around for more than 0.01 seconds
	if (tx_fifo_bytes != 0 && 
	    ((uint8_t)(tick_counter - tx_last_tick) > 1)) {
		rtPhyTxStart(tx_fifo_bytes, tx_header());
		rtPhyClearTxFIFO();
		done_send = true;
		tx_fifo_bytes = 0;
	}

	if (done_send) {
		rtPhyRxOn();
	}
}


// a simple transparent serial implementation
static void transparent_serial_loop(void)
{
	for (;;) {
		__pdata uint8_t	rlen;
		__xdata uint8_t	rbuf[64];
		uint16_t slen;

		tdm_state.bits.slice_counter = tick_counter & SLICE_COUNTER_MASK;

		// if we received something via the radio, turn around and send it out the serial port
		//
		if (RxPacketReceived) {
			uint8_t rxheader;
			LED_ACTIVITY = LED_ON;
			if (rtPhyGetRxPacket(&rlen, rbuf, &rxheader) == PHY_STATUS_SUCCESS) {
				sync_tx_windows(rxheader, rlen);
				received_packet = true;
				if (rlen != 0) {
					//printf("rcv(%d,[", rlen);
					serial_write_buf(rbuf, rlen);
					//printf("]\n");
				}
			}
			LED_ACTIVITY = LED_OFF;
			continue;
		}

		// give the AT command processor a chance to handle a command
		at_command();

		if (!tx_window_open()) {
			// our transmit window isn't open, don't check
			// for serial bytes as we wouldn't have
			// anywhere to put them			
			continue;
		}

		// if we have received something via serial, transmit it
		slen = serial_read_available();
		if (slen > 0) {
			LED_ACTIVITY = LED_ON;
			if (slen > sizeof(rbuf))
				slen = sizeof(rbuf);
			if (serial_read_buf(rbuf, slen)) {
				send_bytes(slen, rbuf);
			}
			LED_ACTIVITY = LED_OFF;
		} else {
			send_bytes(0, rbuf);			
		}
	}
}


void
main(void)
{
	__pdata PHY_STATUS	s;

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
	s = rtPhyRxOn();
	if (s != PHY_STATUS_SUCCESS)
		panic("rtPhyRxOn failed: %u", s);

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
	__pdata PHY_STATUS	s;
	__pdata uint32_t	freq;

	// Do generic PHY initialisation
	s = rtPhyInit();
	if (s != PHY_STATUS_SUCCESS)
		panic("rtPhyInit failed: %u", s);

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

	// Set PHY parameters for the initial operational state
	rtPhySet(TRX_FREQUENCY,		freq);
	rtPhySet(TRX_CHANNEL_SPACING,	100000UL);	// XXX
	
	// And intilise the radio with them.
	s = rtPhyInitRadio(param_get(PARAM_AIR_SPEED)*1000UL);
	if (s != PHY_STATUS_SUCCESS)
		panic("rtPhyInitRadio failed: %u", s);

	// setup network ID
	rtPhySetNetId(param_get(PARAM_NETID));

	// work out how many milliticks a byte takes to come over the air
	milliticks_per_byte = 204800UL / (param_get(PARAM_AIR_SPEED)*125UL);
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
		received_packet = false;
	} else {
		LED_RADIO = blink_state;
		blink_state = !blink_state;
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
