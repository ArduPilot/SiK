// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2025 Alexis Nicole Benini, Ph.D.
// www.alexisbenini.com
// All Rights Reserved
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
/// @file	ham.h
///
/// Function and parameters for transmission of HAM radio callsign
///

#ifndef _HAM_H_
#define _HAM_H_

#include <stdint.h>
#include <stdbool.h>
#include "cdt.h"

#define MAX_CALLSIGN_INTERVAL_SEC 10*60       // According to FCC rules, the call sign must be transmitted at least every 10 minutes.
#define CALLSIGN_MAX_LENGTH 8                 // This includes the initial "DE" sequence
extern __xdata char callsign[CALLSIGN_MAX_LENGTH];   // This assumes that the operator is using a standard callsign length (6).
                                              // However, some regions might use 7 characters. The length of this array includes also the \0 at the end
extern __pdata uint16_t callsign_tx_interval_secs;   // This is set to 600 seconds (10 minutes) by default
extern __pdata bool enable_callsign;                 // Set to false by default (no callsign transmission)

void initialize_ham_callsign_tx_params();

#endif	// _HAM_H_
