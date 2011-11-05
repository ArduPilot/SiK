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

// Serial rx/tx buffers.
//
// Buffer pointer handling depends on these being 256 bytes in size.
//
// Note that the __at() clauses here are not very portable; they presume
// 4K of __xram space, but using absolute addresses for the buffers lets
// the compiler optimise the array indexing code.
//
__xdata __at(0x0e00) uint8_t	rx_buf[256] = {0};
__xdata __at(0x0f00) uint8_t	tx_buf[256] = {0};

// FIFO insert/remove pointers
uint8_t				rx_insert, rx_remove;
uint8_t				tx_insert, tx_remove;

// flag indicating the transmitter is idle
volatile bool			tx_idle;

// FIFO status
#define BUF_FULL(_which)	((_which##_insert + 1) == (_which##_remove))
#define BUF_NOT_FULL(_which)	((_which##_insert + 1) != (_which##_remove))
#define BUF_EMPTY(_which)	(_which##_insert == _which##_remove)
#define BUF_NOT_EMPTY(_which)	(_which##_insert != _which##_remove)
#define BUF_USED(_which)	(_which##_insert - _which##_remove)
#define BUF_FREE(_which)	(_which##_remove - _which##_insert - 1)

// FIFO insert/remove operations
#define BUF_INSERT(_which, _c)	do { _which##_buf[_which##_insert++] = (_c); } while(0)
#define BUF_REMOVE(_which, _c)	do { (_c) = _which##_buf[_which##_remove++]; } while(0)

static void			_serial_write(uint8_t c);
static void			serial_restart(void);

void
serial_interrupt(void) __interrupt(INTERRUPT_UART0) __using(1)
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
			if (BUF_NOT_FULL(rx))
				BUF_INSERT(rx, c);

			// XXX use BUF_FREE here to determine flow control state
		}
	}

	// check for anything to transmit
	if (TI0) {
		// acknowledge the interrupt
		TI0 = 0;

		// look for another byte we can send
		if (BUF_NOT_EMPTY(tx)) {
			// fetch and send a byte
			BUF_REMOVE(tx, c);
			SBUF0 = c;
		} else {
			// note that the transmitter requires a kick to restart it
			tx_idle = true;
		}
	}
}

void
serial_init(uint8_t speed)
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

	// re-enable UART interrupts
	ES0 = 1;
}

bool
serial_write(uint8_t c)
{
	if (serial_write_space() < 1)
		return false;

	_serial_write(c);
	return true;
}

static void
_serial_write(uint8_t c)
{
	bool	istate;

	interrupt_disable(istate);

	// if we have space to store the character
	if (BUF_NOT_FULL(tx)) {

		// queue the character
		BUF_INSERT(tx, c);

		// if the transmitter is idle, restart it
		if (tx_idle)
			serial_restart();
	}

	interrupt_restore(istate);
}

bool
serial_write_buf(__xdata uint8_t *buf, uint8_t count)
{
	bool	istate;

	if (serial_write_space() < count)
		return false;

	interrupt_disable(istate);

	while (count--)
		BUF_INSERT(tx, *buf++);

	// if the transmitter is idle, restart it
	if (tx_idle)
		serial_restart();

	interrupt_restore(istate);

	return true;
}

uint8_t
serial_write_space(void)
{
	// If we are in AT mode, discourage anyone from sending bytes.
	// We don't necessarily want to stall serial_write callers, or
	// to block AT commands while their response bytes trickle out,
	// so we maintain ordering for outbound serial bytes and assume
	// that the receiver will drain the stream while waiting for the
	// OK response on AT mode entry.
	//
	if (at_mode_active)
		return 0;

	return BUF_FREE(tx);
}

static void
serial_restart(void)
{
	// XXX may be idle due to flow control - check signals before starting

	// generate a transmit-done interrupt to force the handler to send another byte
	tx_idle = false;
	TI0 = 1;
}

uint8_t
serial_read(void)
{
	bool		istate;
	uint8_t		c;

	interrupt_disable(istate);

	if (BUF_NOT_EMPTY(rx)) {
		BUF_REMOVE(rx, c);
	} else {
		c = '\0';
	}

	interrupt_restore(istate);

	return c;
}

bool
serial_read_buf(__xdata uint8_t *buf, uint8_t count)
{
	bool		istate;

	if (serial_read_available() < count)
		return false;

	interrupt_disable(istate);

	while (count--)
		BUF_REMOVE(rx, *buf++);

	interrupt_restore(istate);

	return true;
}

uint8_t
serial_read_available(void)
{
	return BUF_USED(rx);
}

void
putchar(char c)
{
	if (c == '\n')
		_serial_write('\r');
	_serial_write(c);
}
