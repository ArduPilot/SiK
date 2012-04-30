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
/// @file	serial.c
///
/// MCS51 Serial port driver with flow control and AT command
/// parser integration.
///

#include "serial.h"
#include "packet.h"

// Serial rx/tx buffers.
//
// Note that the rx buffer is much larger than you might expect
// as we need the receive buffer to be many times larger than the
// largest possible air packet size for efficient TDM. Ideally it
// would be about 16x larger than the largest air packet if we have
// 8 TDM time slots
//
__xdata uint8_t rx_buf[2048] = {0};
__xdata uint8_t tx_buf[512] = {0};
__pdata const uint16_t  rx_mask = sizeof(rx_buf) - 1;
__pdata const uint16_t  tx_mask = sizeof(tx_buf) - 1;

// FIFO insert/remove pointers
static volatile __pdata uint16_t				rx_insert, rx_remove;
static volatile __pdata uint16_t				tx_insert, tx_remove;



// flag indicating the transmitter is idle
static volatile bool			tx_idle;

// FIFO status
#define BUF_FULL(_which)	(((_which##_insert + 1) & _which##_mask) == (_which##_remove))
#define BUF_NOT_FULL(_which)	(((_which##_insert + 1) & _which##_mask) != (_which##_remove))
#define BUF_EMPTY(_which)	(_which##_insert == _which##_remove)
#define BUF_NOT_EMPTY(_which)	(_which##_insert != _which##_remove)
#define BUF_USED(_which)	((_which##_insert - _which##_remove) & _which##_mask)
#define BUF_FREE(_which)	((_which##_remove - _which##_insert - 1) & _which##_mask)

// FIFO insert/remove operations
//
// Note that these are nominally interrupt-safe as only one of each
// buffer's end pointer is adjusted by either of interrupt or regular
// mode code.  This is violated if printing from interrupt context,
// which should generally be avoided when possible.
//
#define BUF_INSERT(_which, _c)	do { _which##_buf[_which##_insert] = (_c); \
		_which##_insert = ((_which##_insert+1) & _which##_mask); } while(0)
#define BUF_REMOVE(_which, _c)	do { (_c) = _which##_buf[_which##_remove]; \
		_which##_remove = ((_which##_remove+1) & _which##_mask); } while(0)
#define BUF_PEEK(_which)	_which##_buf[_which##_remove]
#define BUF_PEEK2(_which)	_which##_buf[(_which##_remove+1) & _which##_mask]

static void			_serial_write(register uint8_t c);
static void			serial_restart(void);
static void serial_device_set_speed(register uint8_t speed);

// save and restore serial interrupt. We use this rather than
// __critical to ensure we don't disturb the timer interrupt at all.
// minimal tick drift is critical for TDM
#define ES0_SAVE_DISABLE __bit ES_saved = ES0; ES0 = 0
#define ES0_RESTORE ES0 = ES_saved

// threshold for considering the rx buffer full
#define SERIAL_CTS_THRESHOLD_LOW  17
#define SERIAL_CTS_THRESHOLD_HIGH 34

void
serial_interrupt(void) __interrupt(INTERRUPT_UART0)
{
	register uint8_t	c;

	// check for received byte first
	if (RI0) {
		// acknowledge interrupt and fetch the byte immediately
		RI0 = 0;
		c = SBUF0;

		// if AT mode is active, the AT processor owns the byte
		if (at_mode_active) {
			// If an AT command is ready/being processed, we would ignore this byte
			if (!at_cmd_ready) {
				at_input(c);
			}
		} else {
			// run the byte past the +++ detector
			at_plus_detector(c);

			// and queue it for general reception
			if (BUF_NOT_FULL(rx)) {
				BUF_INSERT(rx, c);
			} else {
				if (errors.serial_rx_overflow != 0xFFFF) {
					errors.serial_rx_overflow++;
				}
			}
#ifdef SERIAL_CTS
			if (BUF_FREE(rx) < SERIAL_CTS_THRESHOLD_LOW) {
				SERIAL_CTS = true;
			}
#endif
		}
	}

	// check for anything to transmit
	if (TI0) {
		// acknowledge the interrupt
		TI0 = 0;

		// look for another byte we can send
		if (BUF_NOT_EMPTY(tx)) {
#ifdef SERIAL_RTS
			if (feature_rtscts && SERIAL_RTS && !at_mode_active) {
				// the other end doesn't have room in
				// its serial buffer
				tx_idle = true;
				return;
			}
#endif
			// fetch and send a byte
			BUF_REMOVE(tx, c);
			SBUF0 = c;
		} else {
			// note that the transmitter requires a kick to restart it
			tx_idle = true;
		}
	}
}


/// check if RTS allows us to send more data
///
void
serial_check_rts(void)
{
	if (BUF_NOT_EMPTY(tx) && tx_idle) {
		serial_restart();
	}
}

void
serial_init(register uint8_t speed)
{
	// disable UART interrupts
	ES0 = 0;

	// reset buffer state, discard all data
	rx_insert = 0;
	tx_remove = 0;
	tx_insert = 0;
	tx_remove = 0;
	tx_idle = true;

	// configure timer 1 for bit clock generation
	TR1 	= 0;				// timer off
	TMOD	= (TMOD & ~0xf0) | 0x20;	// 8-bit free-running auto-reload mode
	serial_device_set_speed(speed);		// device-specific clocking setup
	TR1	= 1;				// timer on

	// configure the serial port
	SCON0	= 0x10;				// enable receiver, clear interrupts

#ifdef SERIAL_CTS
	// setting SERIAL_CTS low tells the other end that we have
	// buffer space available
	SERIAL_CTS = false;
#endif

	// re-enable UART interrupts
	ES0 = 1;
}

bool
serial_write(register uint8_t c)
{
	if (serial_write_space() < 1)
		return false;

	_serial_write(c);
	return true;
}

static void
_serial_write(register uint8_t c) __reentrant
{
	ES0_SAVE_DISABLE;

	// if we have space to store the character
	if (BUF_NOT_FULL(tx)) {

		// queue the character
		BUF_INSERT(tx, c);

		// if the transmitter is idle, restart it
		if (tx_idle)
			serial_restart();
	} else if (errors.serial_tx_overflow != 0xFFFF) {
		errors.serial_tx_overflow++;
	}

	ES0_RESTORE;
}

// write as many bytes as will fit into the serial transmit buffer
void
serial_write_buf(__xdata uint8_t * __data buf, __pdata uint8_t count)
{
	__pdata uint16_t space;
	__pdata uint8_t n1;

	if (count == 0) {
		return;
	}

	// discard any bytes that don't fit. We can't afford to
	// wait for the buffer to drain as we could miss a frequency
	// hopping transition
	space = serial_write_space();	
	if (count > space) {
		count = space;
		if (errors.serial_tx_overflow != 0xFFFF) {
			errors.serial_tx_overflow++;
		}
	}

	// write to the end of the ring buffer
	n1 = count;
	if (n1 > sizeof(tx_buf) - tx_insert) {
		n1 = sizeof(tx_buf) - tx_insert;
	}
	memcpy(&tx_buf[tx_insert], buf, n1);
	buf += n1;
	count -= n1;
	__critical {
		tx_insert = (tx_insert + n1) & tx_mask;
	}

	// add any leftover bytes to the start of the ring buffer
	if (count != 0) {
		memcpy(&tx_buf[0], buf, count);
		__critical {
			tx_insert = count;
		}		
	}
	__critical {
		if (tx_idle) {
			serial_restart();
		}
	}
}

uint16_t
serial_write_space(void)
{
	register uint16_t ret;
	ES0_SAVE_DISABLE;
	ret = BUF_FREE(tx);
	ES0_RESTORE;
	return ret;
}

static void
serial_restart(void)
{
#ifdef SERIAL_RTS
	if (feature_rtscts && SERIAL_RTS && !at_mode_active) {
		// the line is blocked by hardware flow control
		return;
	}
#endif
	// generate a transmit-done interrupt to force the handler to send another byte
	tx_idle = false;
	TI0 = 1;
}

uint8_t
serial_read(void)
{
	register uint8_t	c;

	ES0_SAVE_DISABLE;

	if (BUF_NOT_EMPTY(rx)) {
		BUF_REMOVE(rx, c);
	} else {
		c = '\0';
	}

#ifdef SERIAL_CTS
	if (BUF_FREE(rx) > SERIAL_CTS_THRESHOLD_HIGH) {
		SERIAL_CTS = false;
	}
#endif

	ES0_RESTORE;

	return c;
}

uint8_t
serial_peek(void)
{
	register uint8_t c;

	ES0_SAVE_DISABLE;
	c = BUF_PEEK(rx);
	ES0_RESTORE;

	return c;
}

uint8_t
serial_peek2(void)
{
	register uint8_t c;

	ES0_SAVE_DISABLE;
	c = BUF_PEEK2(rx);
	ES0_RESTORE;

	return c;
}

// read count bytes from the serial buffer. This implementation
// tries to be as efficient as possible, while disabling interrupts
// for as short a time as possible
bool
serial_read_buf(__xdata uint8_t * __data buf, __pdata uint8_t count)
{
	__pdata uint16_t n1;
	// the caller should have already checked this, 
	// but lets be sure
	if (count > serial_read_available()) {
		return false;
	}
	// see how much we can copy from the tail of the buffer
	n1 = count;
	if (n1 > sizeof(rx_buf) - rx_remove) {
		n1 = sizeof(rx_buf) - rx_remove;
	}
	memcpy(buf, &rx_buf[rx_remove], n1);
	count -= n1;
	buf += n1;
	// update the remove marker with interrupts disabled
	__critical {
		rx_remove = (rx_remove + n1) & rx_mask;
	}
	// any more bytes to do?
	if (count > 0) {
		memcpy(buf, &rx_buf[0], count);
		__critical {
			rx_remove = count;
		}		
	}

#ifdef SERIAL_CTS
	__critical {
		if (BUF_FREE(rx) > SERIAL_CTS_THRESHOLD_HIGH) {
			SERIAL_CTS = false;
		}
	}
#endif
	return true;
}

uint16_t
serial_read_available(void)
{
	register uint16_t ret;
	ES0_SAVE_DISABLE;
	ret = BUF_USED(rx);
	ES0_RESTORE;
	return ret;
}

// return available space in rx buffer as a percentage
uint8_t
serial_read_space(void)
{
	uint16_t space = sizeof(rx_buf) - serial_read_available();
	space = (100 * (space/8)) / (sizeof(rx_buf)/8);
	return space;
}

void
putchar(char c) __reentrant
{
	if (c == '\n')
		_serial_write('\r');
	_serial_write(c);
}


///
/// Table of supported serial speed settings.
/// the table is looked up based on the 'one byte'
/// serial rate scheme that APM uses. If an unsupported
/// rate is chosen then 57600 is used
///
static const __code struct {
	uint8_t rate;
	uint8_t th1;
	uint8_t ckcon;
} serial_rates[] = {
	{1,   0x2C, 0x02}, // 1200
	{2,   0x96, 0x02}, // 2400
	{4,   0x2C, 0x00}, // 4800
	{9,   0x96, 0x00}, // 9600
	{19,  0x60, 0x01}, // 19200
	{38,  0xb0, 0x01}, // 38400
	{57,  0x2b, 0x08}, // 57600 - default
	{115, 0x96, 0x08}, // 115200
	{230, 0xcb, 0x08}, // 230400
};

//
// check if a serial speed is valid
//
bool 
serial_device_valid_speed(register uint8_t speed)
{
	uint8_t i;
	uint8_t num_rates = ARRAY_LENGTH(serial_rates);

	for (i = 0; i < num_rates; i++) {
		if (speed == serial_rates[i].rate) {
			return true;
		}
	}
	return false;
}

static 
void serial_device_set_speed(register uint8_t speed)
{
	uint8_t i;
	uint8_t num_rates = ARRAY_LENGTH(serial_rates);

	for (i = 0; i < num_rates; i++) {
		if (speed == serial_rates[i].rate) {
			break;
		}
	}
	if (i == num_rates) {
		i = 3; // 57600 default
	}

	// set the rates in the UART
	TH1 = serial_rates[i].th1;
	CKCON = (CKCON & ~0x0b) | serial_rates[i].ckcon;

	// tell the packet layer how fast the serial link is. This is
	// needed for packet framing timeouts
	packet_set_serial_speed(speed*125UL);	
}

