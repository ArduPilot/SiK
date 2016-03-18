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

// ******************** includes *************************************
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>
#include "printfl.h"
#include "serial.h"


// ******************** defines and typedefs *************************
// ******************** local variables ******************************

static char radix;
static uint8_t long_flag = 0;
static uint8_t string_flag = 0;
static uint8_t char_flag = 0;
static uint8_t unsigned_flag = 0;
static int8_t pad_length = 0;
static char *  str;
static  long val;

// allow printf() output to be captured to a buffer
// for remote AT command control
static bool capture;
static uint8_t *capture_buffer;
static uint16_t capture_buffer_size;
static uint16_t captured_size;
// ******************** local function prototypes ********************
static char *ultoapad(unsigned long num, char *str, int radix,int8_t displaydigits);
#define ultoa(num,str,radix) ultoapad(num,str,radix,0)
static char *ltoa(long num, char *str, int radix);
// ********************* Implementation ******************************

static void output_char(char c)
{
	if (!capture) {
		putChar(c);
		return;
	}
	if (captured_size < capture_buffer_size) {
		capture_buffer[captured_size++] = c;
	}
}

// start capturing bytes from printf()
void printf_start_capture(uint8_t *buf, uint16_t size)
{
	capture_buffer = buf;
	captured_size = 0;
	capture_buffer_size = size;
	capture = true;
}

// end capture, returning number of bytes that have been captured
uint16_t
printf_end_capture(void)
{
	capture = false;
	return captured_size;
}

void vprintfl(const char * fmt, va_list ap)
{
	for (; *fmt; fmt++) {
		if (*fmt == '%') {
			long_flag = string_flag = char_flag = unsigned_flag = pad_length = 0;
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
			case 'X':
				radix = 16;
				unsigned_flag = 1;
				pad_length = (long_flag)?(8):(4);
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
				}
				else if (char_flag) {
					val = va_arg(ap,unsigned int);
				} else {
					val = va_arg(ap,unsigned int);
				}
			} else {
				if (long_flag) {
					val = va_arg(ap,long);
				} else if (char_flag) {
					val = va_arg(ap,int);
				} else {
					val = va_arg(ap,int);
				}
			}

			if (radix) {
				static char buffer[12]; /* 37777777777(oct) */
				char * stri;

				if (unsigned_flag) {
					ultoapad(val, buffer, radix,pad_length);
				} else {
					ltoa(val, buffer, radix);
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

void printfl(const char *fmt, ...)
{
	va_list ap;

	va_start(ap,fmt);
	vprintfl(fmt, ap);
}

static char *ultoapad(unsigned long num, char *str, int radix,int8_t displaydigits)
{
    char temp[33];  //an int can only be 16 bits long
                    //at radix 2 (binary) the string
                    //is at most 16 + 1 null long.
    int temp_loc = 0;
    int digit;
    int str_loc = 0;

    //construct a backward string of the number.
    do {
        digit = (unsigned long)num % radix;
        if (digit < 10)
            temp[temp_loc++] = digit + '0';
        else
            temp[temp_loc++] = digit - 10 + 'A';
        num = ((unsigned long)num) / radix;
    } while ((unsigned long)num > 0);

    temp_loc--;
    displaydigits --;
    if(displaydigits < temp_loc){displaydigits = temp_loc;}
    while(displaydigits >= 0)
    {
    	if(displaydigits > temp_loc)
    	{
    		str[str_loc++] = '0';
    	}
    	else
    	{
    		str[str_loc++] = temp[displaydigits];
    	}
    	displaydigits--;
    }
    str[str_loc] = 0; // add null termination.

    return str;
}

static char *ltoa(long num, char *str, int radix)
{
    char sign = 0;
    char temp[33];  //an int can only be 32 bits long
                    //at radix 2 (binary) the string
                    //is at most 16 + 1 null long.
    int temp_loc = 0;
    int digit;
    int str_loc = 0;

    //save sign for radix 10 conversion
    if (radix == 10 && num < 0) {
        sign = 1;
        num = -num;
    }

    //construct a backward string of the number.
    do {
        digit = (unsigned long)num % radix;
        if (digit < 10)
            temp[temp_loc++] = digit + '0';
        else
            temp[temp_loc++] = digit - 10 + 'A';
        num = ((unsigned long)num) / radix;
    } while ((unsigned long)num > 0);

    //now add the sign for radix 10
    if (radix == 10 && sign) {
        temp[temp_loc] = '-';
    } else {
        temp_loc--;
    }


    //now reverse the string.
    while ( temp_loc >=0 ) {// while there are still chars
        str[str_loc++] = temp[temp_loc--];
    }
    str[str_loc] = 0; // add null termination.

    return str;
}
// ********************* end of printfl.c ****************************
