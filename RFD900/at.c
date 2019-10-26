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
/// @file	at.c
///
/// A simple AT command parser.
///
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "radio_old.h"
#include "tdm.h"
//#include "flash_layout.h"
#include "at.h"
#include "board_rfd900e.h"
#include "pins_user.h"
#include "parameters.h"
#include "serial.h"
#include "printfl.h"
#include "flash.h"
#include "PWM.h"
#include "aes.h"
#include "ppm.h"

// AT command buffer
char at_cmd[AT_CMD_MAXLEN + 1];
uint8_t	at_cmd_len;

// mode flags
bool		at_mode_active;	///< if true, incoming bytes are for AT command
bool		at_cmd_ready;	///< if true, at_cmd / at_cmd_len contain valid data

// test bits
uint8_t		at_testmode;    ///< test modes enabled (AT_TEST_*)

// command handlers
static void	at_ok(void);
static void	at_error(void);
static void	at_i(void);
static void	at_s(void);
static void	at_r(void);
static void	at_ampersand(void);
static void	at_p(void);
static void	at_plus(void);

void
at_input(register uint8_t c)
{
    // AT mode is active and waiting for a command
    switch (c) {
    // CR - submits command for processing
    case '\r':
        putChar('\n');
        at_cmd[at_cmd_len] = 0;
        at_cmd_ready = true;
        break;

    // backspace - delete a character
    // delete - delete a character
    case '\b':
    case '\x7f':
        if (at_cmd_len > 0) {
            putChar('\b');
            putChar(' ');
            putChar('\b');
            at_cmd_len--;
        }
        break;

    // character - add to buffer if valid
    default:
        if (at_cmd_len < AT_CMD_MAXLEN) {
            if (isprint(c)) {
                c = toupper(c);
                at_cmd[at_cmd_len++] = c;
                putChar(c);
            }
            break;
        }

        // If the AT command buffer overflows we abandon
        // AT mode and return to passthrough mode; this is
        // to minimise the risk of locking up on reception
        // of an accidental escape sequence.

        at_mode_active = 0;
        at_cmd_len = 0;
        break;
    }
}

// +++ detector state machine
//
// states:
//
// wait_for_idle:	-> wait_for_plus1, count = 0 after 1s
//
// wait_for_plus:	-> wait_for_idle if char != +
//			-> wait_for_plus, count++ if count < 3
// wait_for_enable:	-> enabled after 1s
//			-> wait_for_idle if any char
//

#define	ATP_WAIT_FOR_IDLE	0
#define ATP_WAIT_FOR_PLUS1	1
#define ATP_WAIT_FOR_PLUS2	2
#define ATP_WAIT_FOR_PLUS3	3
#define ATP_WAIT_FOR_ENABLE	4

#define ATP_COUNT_1S		10	// 10 ticks of the 10Hz timer

static uint8_t	at_plus_state;
static uint8_t	at_plus_counter = ATP_COUNT_1S;

void at_plus_detector(register uint8_t c)
{
    // If we get a character that's not '+', unconditionally
    // reset the state machine to wait-for-idle; this will
    // restart the 1S timer.
    //
    if (c != (uint8_t)'+')
        at_plus_state = ATP_WAIT_FOR_IDLE;

    // We got a plus; handle it based on our current state.
    //
    switch (at_plus_state) {

    case ATP_WAIT_FOR_PLUS1:
    case ATP_WAIT_FOR_PLUS2:
        at_plus_state++;
        break;

    case ATP_WAIT_FOR_PLUS3:
        at_plus_state = ATP_WAIT_FOR_ENABLE;
        at_plus_counter = ATP_COUNT_1S;
        break;

    // FALLTHROUGH
    case ATP_WAIT_FOR_IDLE:
    case ATP_WAIT_FOR_ENABLE:
        at_plus_counter = ATP_COUNT_1S;
        break;
    default:
        at_plus_state = ATP_WAIT_FOR_IDLE;
        at_plus_counter = ATP_COUNT_1S;
        break;
    }
}

void at_timer(void)
{
    // if the counter is running
    if (at_plus_counter > 0) {

        // if it reaches zero, the timeout has expired
        if (--at_plus_counter == 0) {

            // make the relevant state change
            switch (at_plus_state) {
            case ATP_WAIT_FOR_IDLE:
                at_plus_state = ATP_WAIT_FOR_PLUS1;
                break;

            case ATP_WAIT_FOR_ENABLE:
                at_mode_active = true;
                at_plus_state = ATP_WAIT_FOR_IDLE;

                // stuff an empty 'AT' command to get the OK prompt
                at_cmd[0] = 'A';
                at_cmd[1] = 'T';
                at_cmd[2] = '\0';
                at_cmd_len = 2;
                at_cmd_ready = true;
                break;
            default:
                // should never happen, but otherwise harmless
                break;
            }
        }
    }
}

void at_command(void)
{
    // require a command with the AT prefix
    if (at_cmd_ready) {
        if ((at_cmd_len >= 2) && (at_cmd[0] == 'R') && (at_cmd[1] == 'T')) {
            // remote AT command - send it to the tdm
            // system to send to the remote radio
            tdm_remote_at();
            at_cmd_len = 0;
            at_cmd_ready = false;
            return;
        }

        if ((at_cmd_len >= 2) && (at_cmd[0] == 'A') && (at_cmd[1] == 'T')) {

            // look at the next byte to determine what to do
            switch (at_cmd[2]) {
            case '\0':		// no command -> OK
                at_ok();
                break;
            case '&':
                at_ampersand();
                break;
            case '+':
                at_plus();
                break;
            case 'I':
                at_i();
                break;
            case 'P':
                at_p();
                break;
            case 'O':		// O -> go online (exit command mode)
                at_plus_counter = ATP_COUNT_1S;
                at_mode_active = 0;
                break;
            case 'S':
                at_s();
                break;
            case 'R':
                at_r();
                break;

            case 'Z':
                for (;;)			//watchdog will reset in 1 second
                    ;
                break;
            default:
                at_error();
            }
        }

        // unlock the command buffer
        at_cmd_len = 0;
        at_cmd_ready = false;
    }
}

static void at_ok(void)
{
    printf("%s\n", "OK");
}

static void at_error(void)
{
    printf("%s\n", "ERROR");
}

uint8_t		idx;
uint32_t	at_num;

/*
  parse a number at idx putting the result in at_num
 */
static void at_parse_number()
{
    register uint8_t c;

    at_num = 0;
    for (;;) {
        c = at_cmd[idx];
        if (!isdigit(c))
            break;
        at_num = (at_num * 10) + (c - '0');
        idx++;
    }
}
static const char TypeStr[PT_Last] = {'N','B','L'};

static void ShowATParamExt(void)
{
    uint8_t id;
    const char** List;
    // convenient way of showing all parameters
    for (id = 0; id < PARAM_S_MAX; id++)
    {
        printf("S%u:%s(%c)[%lu..%lu]=%lu",(unsigned)id,param_s_name(id),TypeStr[param_s_Type(id)],
               param_s_Min(id),param_s_Max(id),(unsigned long)param_s_get(id));
        List = param_s_nameList(id);
        if(NULL != List)
        {
            printf("{");
            while(NULL != *List)
            {
                printf("%s,",*List);
                List++;
            }
            printf("}");
        }
        printf("\n");
    }
}

static void at_i(void)
{
    switch (at_cmd[3]) {
    case '\0':
    case '0':
        printf("%s\n", g_banner_string);
        return;
    case '1':
        printf("%s\n", g_version_string);
        return;
    case '2':
        printf("%u\n", BOARD_ID);
        break;
    case '3':
        printf("%u\n", g_board_frequency);
        break;
    case '4':
        printf("%u\n", g_board_bl_version);
        return;
    case '5': {
        //register enum ParamID id;
        uint16_t id;
        register uint8_t start = 0;
        register uint8_t end = PARAM_S_MAX-1;

        if (at_cmd[4] == '?')
        {
            ShowATParamExt();
            return;
        }
        if (at_cmd[4] == ':' && isdigit((uint16_t)(at_cmd[5]))) {
            idx = 5;
            at_parse_number();
            start = at_num;

            if (at_cmd[idx] == ':' && isdigit((uint16_t)(at_cmd[idx+1]))) {
                idx++;
                at_parse_number();
                end = at_num;
            }
        }
        // convenient way of showing all parameters
        for (id = start; id <= end; id++) {
            printf("S%u:%s=%lu\n",
                   (unsigned)id,
                   param_s_name(id),
                   (unsigned long)param_s_get(id));
        }
        // convenient way of showing all parameters
        for (id = 0; id < PARAM_R_MAX; id++) {
            printf("R%u:%s=%lu\n",
                   (unsigned)id,
                   param_r_name(id),
                   (unsigned long)param_r_get(id));
        }
        return;
    }
    case '6':
        tdm_report_timing();
        return;
    case '7':
        tdm_show_rssi();
        return;
    case '8':
        printf("0x%lX%lX\n",DEVINFO->UNIQUEH,DEVINFO->UNIQUEL);
        return;
    default:
        at_error();
        return;
    }
}

static void at_s(void)
{
    uint8_t		sreg;

    // get the register number first
    idx = 3;
    at_parse_number();
    sreg = at_num;
    // validate the selected sreg
    if (sreg >= PARAM_S_MAX) {
        at_error();
        return;
    }

    switch (at_cmd[idx]) {
    case '?':
        at_num = param_s_get(sreg);
        printf("%lu\n", at_num);
        return;

    case '=':
        if (sreg > 0) {
            idx++;
            at_parse_number();
            if (param_s_set(sreg, at_num)) {
                at_ok();
                return;
            }
        }
        break;
    }
    at_error();
}

static void at_r(void)
{
    uint8_t		sreg;

    // get the register number first
    idx = 3;
    at_parse_number();
    sreg = at_num;
    // validate the selected sreg
    if (sreg >= PARAM_R_MAX) {
        at_error();
        return;
    }

    switch (at_cmd[idx]) {
    case '?':
        at_num = param_r_get(sreg);
        printf("%lu\n", at_num);
        return;

    case '=':
        idx++;
        at_parse_number();
        if (param_r_set(sreg, at_num)) {
            at_ok();
            return;
        }
        break;
    }
    at_error();
}

static void at_ampersand(void)
{
    switch (at_cmd[3]) {
    case 'F':
        param_default();
        at_ok();
        break;
    case 'W':
        param_save();
        at_ok();
        break;

    case 'U':
        if (!strcmp(at_cmd + 4, "PDATE")) {
            NVIC_SystemReset(); // we will read the reset cause on reset to determine this has been called
            while(1);
        }
        at_error();
        break;

    case 'P':
        tdm_change_phase();
        break;

    case 'R':
        (PPMRecordDefault())?(at_ok()):(at_error());
        break;

    case 'T':
        // enable test modes
        if (!strcmp(at_cmd + 4, "")) {
            // disable all tests
            at_testmode = 0;
        } else if (!strcmp(at_cmd + 4, "=RSSI")) {
            // display RSSI stats
            at_testmode ^= AT_TEST_RSSI;
        } else if (!strcmp(at_cmd + 4, "=TDM")) {
            // display TDM debug
            at_testmode ^= AT_TEST_TDM;
        } else {
            at_error();
        }
        break;

    case 'E':
        switch (at_cmd[4]) {
        case '?':
            print_encryption_key();
            return;

        case '=':
            if (param_set_encryption_key((unsigned char *)&at_cmd[5])) {
                at_ok();
                return;
            }
            break;
        }
        break;

    default:
        at_error();
        break;
    }
}

static void at_p (void)
{
#if PIN_MAX > 0
    uint8_t pinId;
    if(at_cmd[3] == 'P')
    {
        for (pinId = 0; pinId < PIN_MAX; pinId++)
        {
            printf("Pin:%u ", pinId);
            if (pins_user_get_io(pinId))
                printf("Output ");
            else
                printf("Input  ");
            printf("Val: %u\n",pins_user_get_value(pinId));
        }
        return;
    }
    else if(at_cmd[4] != '=' || !isdigit((uint16_t)(at_cmd[5])))
    {
        at_error();
        return;
    }

    pinId = at_cmd[5] - '0';

    switch (at_cmd[3]) {

    // Set pin to output, turn mirroring off pulling pin to ground
    case 'O':
        pins_user_set_io(pinId, PIN_OUTPUT);
        break;

    // Need to figure out how to set pins to Input/Output
    case 'I':
        pins_user_set_io(pinId, PIN_INPUT);
        break;

    // Set pin to mirror input
    case 'M':
        if(!pins_user_set_mirror(pinId, PIN_MIRROR))
        {
            at_error();
        }
        break;

    case 'R':
        if(pins_user_get_io(pinId) == PIN_INPUT)
            printf("val:%u\n", pins_user_get_adc(pinId));
        else
            at_error();
        return;
        break;

    case 'C':
        if(!isdigit((uint16_t)(at_cmd[7])) || !pins_user_set_value(pinId, (at_cmd[7]-'0')?1:0))
        {
            at_error();
            return;
        }
        break;
    default:
        at_error();
        return;
    }

    at_ok();
#else
    at_error();
#endif
}

static void
at_plus(void)
{
//#if defined BOARD_rfd900a || defined BOARD_rfd900p || defined BOARD_rfd900e
    uint8_t		creg;

    // get the register number first
    idx = 4;
    at_parse_number();
    creg = at_num;

    switch (at_cmd[3])
    {
    case 'P': // AT+P=x set power level pwm to x immediately
        if (at_cmd[4] != '=')
        {
            break;
        }
        idx = 5;
        at_parse_number();
        SetPwmDuty(at_num & 0xFF);
        radio_set_diversity(false);
        at_ok();
        return;
    case 'C': // AT+Cx=y write calibration value
        switch (at_cmd[idx])
        {
        case '?':
            at_num = calibration_get(creg);
            printf("%lu\n",at_num);
            return;
        case '=':
            idx++;
            at_parse_number();
            if (calibration_set(creg, at_num&0xFF))
            {
                at_ok();
            } else {
                at_error();
            }
            return;
        }
        break;
    case 'L': // AT+L lock bootloader area if all calibrations written
        if (calibration_lock())
        {
            at_ok();
        } else {
            at_error();
        }
        return;
    }
//#endif //BOARD_rfd900a / BOARD_rfd900p
    at_error();
}

