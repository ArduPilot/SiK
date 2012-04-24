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

/// Initialise the serial port.
///
/// @param	speed		The serial speed to configure, passed
///				to serial_device_set_speed at the appropriate
///				point during initialisation.
///
extern void	serial_init(register uint8_t speed);

/// check if a serial speed is valid
///
/// @param	speed		The serial speed to configure
///
extern bool serial_device_valid_speed(register uint8_t speed);

/// Write a byte to the serial port.
///
/// @param	c		The byte to write.
/// @return			True if the byte was written, false if the
///				FIFO is full.
///
extern bool	serial_write(register uint8_t c);

/// Write bytes to the serial port.
///
/// @param	buf		Pointer to the data to write.
/// @param	count		The number of bytes to write.
///
extern void	serial_write_buf(__xdata uint8_t * __data buf, __pdata uint8_t count);

/// Check for space in the write FIFO
///
/// @return			The number of bytes that can be written.
///
extern uint16_t	serial_write_space(void);

/// Check for space in the read FIFO. Used to allow for software flow
/// control
///
/// @return			The percentage free space in the rx buffer
///
extern uint8_t	serial_read_space(void);

/// Read a byte from the serial port.
///
/// @return			The next byte in the receive FIFO.
///				If no bytes are available, returns zero.
///
extern uint8_t	serial_read(void);

/// peek at a byte from the serial port, without removing it
/// caller must ensure serial available is > 0
///
/// @return			The next byte in the receive FIFO.
///
extern uint8_t	serial_peek(void);

/// peek at the byte after next from the serial port, without removing it
/// caller must ensure serial available is > 1
///
/// @return			The byte after next in the receive FIFO.
///
extern uint8_t	serial_peek2(void);

/// Read bytes from the serial port.
///
/// @param	buf		Buffer for read data.
/// @param	count		The number of bytes to read.
/// @return			True if @count bytes were read, false
///				if there is not enough data in the buffer
///				to satisfy the request (no bytes are read
///				in this case).
///
extern bool	serial_read_buf(__xdata uint8_t * __data buf, __pdata uint8_t count);

/// Check for bytes in the read FIFO
///
/// @return			The number of bytes available to be read
///				from the FIFO.
///
extern uint16_t	serial_read_available(void);

/// check if RTS allows us to send more data
///
extern void serial_check_rts(void);

#endif // _SERIAL_H_
