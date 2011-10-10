/* -*- Mode: C; c-basic-offset: 8; -*- */
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

/*
 * A simple AT command parser.
 *
 * Bytes arriving on the serial port should be passed to 
 * at_input.  If it returns true, the character has been consumed
 * by the parser and should not be sent onwards.
 */

#include "radio.h"

bool	at_mode_active;
static void	at_plus_detector(uint8_t c);
static void	at_command(void);

static void	at_ok(void);
static void	at_error(void);
static void	at_i(void);
static void	at_s(void);

/* AT command buffer */
#define AT_CMD_MAXLEN	16
__xdata char	at_cmd[AT_CMD_MAXLEN + 1];
__pdata uint8_t	at_cmd_len;

bool
at_input(uint8_t c)
{
	/* if not active, offer the character to the +++ detector */
	if (!at_mode_active) {
		at_plus_detector(c);
		return false;
	}

	/* convert everything to uppercase */
	if (isalpha(c))
		c = toupper(c);

	switch (c) {
		/* CR - submits command for processing */
	case '\r':
		putchar('\r');
		putchar('\n');
		at_cmd[at_cmd_len] = 0;
		at_command();
		at_cmd_len = 0;
		break;

		/* backspace - delete a character */
	case '\b':
		if (at_cmd_len > 0) {
			putchar('\b');
			putchar(' ');
			putchar('\b');
			at_cmd_len--;
		}
		break;

		/* character - add to buffer if valid */
	default:
		if (isprint(c) && (at_cmd_len < AT_CMD_MAXLEN))
			at_cmd[at_cmd_len++] = c;
		putchar(c);
		break;
	}
	return true;
}

static void
at_command(void)
{
	/* require the AT prefix */
	if ((at_cmd_len < 2) || (at_cmd[0] != 'A') || (at_cmd[1] != 'T'))
		return;

	/* look at the next byte to determine what to do */
	switch (at_cmd[2]) {
	case '\0':		// no command -> OK
		at_ok();
		break;
	case 'I':
		at_i();
		break;
	case 'O':		// O -> go online (exit command mode)
		at_mode_active = 0;
		break;
	case 'S':
		at_s();
		break;
	default:
		at_error();
	}
}

static void
at_ok(void)
{
	puts("OK");
}

static void
at_error(void)
{
	puts("ERROR");
}

static void
at_i(void)
{
	switch (at_cmd[3]) {
	case '0':
		puts("SiK");
		break;
	case '1':
		puts(stringify(VERSION_MAJOR) "." stringify(VERSION_MINOR));
		break;
	default:
		at_error();
		break;
	}
}

struct ATSParameter {
	uint8_t		scode;
	enum ParamID	id;
};
#define ATS_READONLY	0x80
#define ATS_NO_PARAM	PARAM_MAX

/* XXX update per enum ParamID */
const uint8_t __code at_s_parameters[] = {
	PARAM_FORMAT | ATS_READONLY,
	PARAM_NODE_ID,
	PARAM_PEER_ID,
	PARAM_TRX_FREQUENCY,
	PARAM_TRX_CHANNEL_SPACING,
	PARAM_TRX_DEVIATION,
	PARAM_TRX_DATA_RATE,
	PARAM_RX_BAND_WIDTH,
	PARAM_SERIAL_SPEED,
};
#define ATS_MAX_PARAM	(sizeof(at_s_parameters) / sizeof(at_s_parameters[0]))

static void
at_s(void)
{
	__pdata uint8_t		idx;
	__pdata uint8_t		reg;
	__pdata uint16_t	val;
	__pdata uint8_t		c;

	reg = 0;
	idx = 3;	// first character of the sreg number

	for (;;) {
		c = at_cmd[idx];
		if (c == '?') {
			if ((reg >= ATS_MAX_PARAM) ||
				(ATS_NO_PARAM == (c = at_s_parameters[reg]))) {
				at_error();
			} else {
				val = param_get16(c);
				//printf("%u\n", val);
			}
			return;
		}
		if (c == '=') {
			c++;
			break;
		}
		if (!isdigit(c)) {
			at_error();
			break;
		}
		reg = (reg * 10) + (c - '0');
	}

	val = 0;
	for (;;) {
		c = at_cmd[idx];
		if (c == '\0') {
			/* XXX set sreg to val */
			return;
		}
		if (!isdigit(c)) {
			at_error();
			break;
		}
		val = (val * 10) + (c - '0');
	}
}


/*
 * +++ detector state machine
 *
 * states:
 *
 * wait_for_idle:	-> wait_for_plus1, count = 0 after 1s
 *
 * wait_for_plus:	-> wait_for_idle if char != +
 *			-> wait_for_plus, count++ if count < 3
 * wait_for_enable:	-> enabled after 1s
 *			-> wait_for_idle if any char
 */

#define	ATP_WAIT_FOR_IDLE	0
#define ATP_WAIT_FOR_PLUS1	1
#define ATP_WAIT_FOR_PLUS2	2
#define ATP_WAIT_FOR_PLUS3	3
#define ATP_WAIT_FOR_ENABLE	4

__data uint8_t	at_plus_state;
__data uint8_t	at_plus_counter;
#define ATP_COUNT_1S		100	/* 100 ticks of the 100Hz timer */

static void
at_plus_detector(uint8_t c)
{
	bool	istate;

	interrupt_disable(istate);

	/*
	 * If we get a character that's not '+', unconditionally
	 * reset the state machine to wait-for-idle
	 */
	if (c != '+')
		at_plus_state = ATP_WAIT_FOR_IDLE;

	/*
	 * We got a plus; handle it based on our current state.
	 */
	switch (at_plus_state) {

	case ATP_WAIT_FOR_PLUS3:
		at_plus_state = ATP_WAIT_FOR_ENABLE;
		at_plus_counter = ATP_COUNT_1S;
		/* FALLTHROUGH */
	case ATP_WAIT_FOR_PLUS2:
	case ATP_WAIT_FOR_PLUS1:
		at_plus_state++;
		break;

	default:
		at_plus_state = ATP_WAIT_FOR_IDLE;
		/* FALLTHROUGH */
	case ATP_WAIT_FOR_IDLE:
	case ATP_WAIT_FOR_ENABLE:
		at_plus_counter = ATP_COUNT_1S;
		break;
	}

	interrupt_restore(istate);
}

/*
 * 100Hz timer callout
 */
void
at_timer(void)
{
	/* if the counter is running */
	if (at_plus_counter > 0) {

		/* if it reaches zero, the timeout has expired */
		if (--at_plus_counter == 0) {

			/* make the relevant state change */
			switch (at_plus_state) {
			case ATP_WAIT_FOR_IDLE:
				at_plus_state = ATP_WAIT_FOR_PLUS1;
				break;
			case ATP_WAIT_FOR_ENABLE:
				at_mode_active = true;
				at_plus_state = ATP_WAIT_FOR_IDLE;
				break;
			default:
				/* should never happen, but otherwise harmless */
			}
		}
	}
}
