/*-----------------------------------------------------------------
 printfl.c - source file for reduced version of printf

 Copyright (C) 1999, Sandeep Dutta . sandeep.dutta@usa.net
 2001060401: Improved by was@icb.snz.chel.su

 This library is free software; you can redistribute it and/or modify it
 under the terms of the GNU General Public License as published by the
 Free Software Foundation; either version 2.1, or (at your option) any
 later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this library; see the file COPYING. If not, write to the
 Free Software Foundation, 51 Franklin Street, Fifth Floor, Boston,
 MA 02110-1301, USA.

 As a special exception, if you link this library with other files,
 some of which are compiled with SDCC, to produce an executable,
 this library does not by itself cause the resulting executable to
 be covered by the GNU General Public License. This exception does
 not however invalidate any other reasons why the executable file
 might be covered by the GNU General Public License.
 -------------------------------------------------------------------------*/

/* following formats are supported :-
 format     output type       argument-type
 %d        decimal             int
 %ld       decimal             long
 %hd       decimal             char
 %u        decimal             unsigned int
 %lu       decimal             unsigned long
 %hu       decimal             unsigned char
 %x        hexadecimal         int
 %lx       hexadecimal         long
 %hx       hexadecimal         char
 %o        octal               int
 %lo       octal               long
 %ho       octal               char
 %c        character           char
 %s        character           generic pointer
 */
#include "radio.h"

static __data char radix;
static __bit long_flag = 0;
static __bit string_flag = 0;
static __bit char_flag = 0;
static __bit unsigned_flag = 0;
static char * __data str;
static __data long val;

// allow printf() output to be captured to a buffer
// for remote AT command control
static bool capture;
static __xdata uint8_t *capture_buffer;
static __pdata uint8_t capture_buffer_size;
static __pdata uint8_t captured_size;

static void 
output_char(register char c)
{
	if (!capture) {
		putchar(c);
		return;
	}
	if (captured_size < capture_buffer_size) {
		capture_buffer[captured_size++] = c;
	}
}

// start capturing bytes from printf()
void 
printf_start_capture(__xdata uint8_t *buf, uint8_t size)
{
	capture_buffer = buf;
	captured_size = 0;
	capture_buffer_size = size;
	capture = true;
}

// end capture, returning number of bytes that have been captured
uint8_t 
printf_end_capture(void)
{
	capture = false;
	return captured_size;
}

void 
vprintfl(const char * fmt, va_list ap) __reentrant
{

	for (; *fmt; fmt++) {
		if (*fmt == '%') {
			long_flag = string_flag = char_flag = unsigned_flag = 0;
			fmt++;
			switch (*fmt) {
			case 'l':
				long_flag = 1;
				fmt++;
				break;
			case 'h':
				char_flag = 1;
				fmt++;
			}

			switch (*fmt) {
			case 's':
				string_flag = 1;
				break;
			case 'd':
				radix = 10;
				break;
			case 'u':
				radix = 10;
				unsigned_flag = 1;
				break;
			case 'x':
				radix = 16;
				unsigned_flag = 1;
				break;
			case 'c':
				radix = 0;
				break;
			case 'o':
				radix = 8;
				unsigned_flag = 1;
				break;
			}

			if (string_flag) {
				str = va_arg(ap, char *);
				while (*str)
					output_char(*str++);
				continue;
			}

			if (unsigned_flag) {
				if (long_flag) {
					val = va_arg(ap,unsigned long);
				} else if (char_flag) {
					val = va_arg(ap,unsigned char);
				} else {
					val = va_arg(ap,unsigned int);
				}
			} else {
				if (long_flag) {
					val = va_arg(ap,long);
				} else if (char_flag) {
					val = va_arg(ap,char);
				} else {
					val = va_arg(ap,int);
				}
			}

			if (radix) {
				static char __idata buffer[12]; /* 37777777777(oct) */
				char __idata * stri;

				if (unsigned_flag) {
					_ultoa(val, buffer, radix);
				} else {
					_ltoa(val, buffer, radix);
				}
				stri = buffer;
				while (*stri) {
					output_char(*stri);
					stri++;
				}
			} else {
				output_char((char) val);
			}

		} else {
			output_char(*fmt);
		}
	}
}

void 
printfl(const char *fmt, ...) __reentrant
{
	va_list ap;

	va_start(ap,fmt);
	vprintfl(fmt, ap);
}

