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
/// @file	serial.h
///
/// Serial port driver with flow control and AT command
/// parser integration.
///

#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <stdint.h>
#include "radio.h"

/// Supported serial speeds
///
/// Note that this list is missing 19200, which is difficult to generate
/// from the 24.5MHz oscillator clock.
///
enum SerialSpeed {
        B9600,
        B19200,
        B38400,
        B57600,
        B115200,
        B230400,
        BMAX,
        BNOCHANGE
};

/// Set the serial device speed.
///
/// Since this typically depends on chip-specific register tweaking,
/// this function must be supplied by external code.
///
/// @param	speed		The serial speed to configure.
///
extern void	serial_device_set_speed(enum SerialSpeed speed);

/// Initialise the serial port.
///
/// @param	speed		The serial speed to configure, passed
///				to serial_device_set_speed at the appropriate
///				point during initialisation.
///
extern void	serial_init(enum SerialSpeed speed);

/// Write a byte to the serial port.
///
/// @param	c		The byte to write.
/// @return			True if the byte was written, false if the
///				FIFO is full.
///
extern bool	serial_write(uint8_t c);

/// Write bytes to the serial port.
///
/// @param	buf		Pointer to the data to write.
/// @param	count		The number of bytes to write.
/// @return			True if all of the data was written.
///				False if there is not enough room in the
///				buffer for @a count bytes (no bytes are
///				written in this case).
///
extern bool	serial_write_buf(__xdata uint8_t *buf, uint8_t count);

/// Check for space in the write FIFO
///
/// @return			The number of bytes that can be written.
///
extern uint8_t	serial_write_space(void);

/// Read a byte from the serial port.
///
/// @return			The next byte in the receive FIFO.
///				If no bytes are available, returns zero.
///
extern uint8_t	serial_read(void);

/// Read bytes from the serial port.
///
/// @param	buf		Buffer for read data.
/// @param	count		The number of bytes to read.
/// @return			True if @count bytes were read, false
///				if there is not enough data in the buffer
///				to satisfy the request (no bytes are read
///				in this case).
///
extern bool	serial_read_buf(__xdata uint8_t *buf, uint8_t count);

/// Check for bytes in the read FIFO
///
/// @return			The number of bytes available to be read
///				from the FIFO.
///
extern uint8_t	serial_read_available(void);

#endif // _SERIAL_H_
