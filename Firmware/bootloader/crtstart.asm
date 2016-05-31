	;;
	;; Copyright (c) 2011 Michael Smith, All Rights Reserved
	;;
	;;
	;; Redistribution and use in source and binary forms, with or without
	;; modification, are permitted provided that the following conditions
	;; are met:
	;;
	;;  o Redistributions of source code must retain the above copyright 
	;;    notice, this list of conditions and the following disclaimer.
	;;  o Redistributions in binary form must reproduce the above copyright 
	;;    notice, this list of conditions and the following disclaimer in 
	;;    the documentation and/or other materials provided with the distribution.
	;;
	;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	;; "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	;; LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	;; FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	;; COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
	;; INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	;; (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
	;; SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
	;; HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
	;; STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
	;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
	;; OF THE POSSIBILITY OF SUCH DAMAGE.
	;;

	;;
	;; Modified crtstart.asm for SDCC and Si1000 bootloader
	;;
	;; Inspired by a post on the SiLabs forum by Tsuneo
	;;
	
	.module	crtstart

	.area	HOME	(CODE)
	.area	GSINIT0	(CODE)
	.area	GSINIT	(CODE)
	.area	GSFINAL	(CODE)
	.area	CSEG	(CODE)

	;; Stack segment in internal RAM

	.area	SSEG	(DATA)

L__stack:
	.ds	1

	;; Reset vector and interrupt redirection table

	.area	HOME	(CODE)

__offset = 0x400		; XXX would be nice to get this from somewhere

	ljmp	L__start	; reset vector
	ljmp	. + __offset	; /INT0
	.ds	5
	ljmp	. + __offset	; Timer0 overflow
	.ds	5
	ljmp	. + __offset	; /INT1
	.ds	5
	ljmp	. + __offset	; Timer1 overflow
	.ds	5
	ljmp	. + __offset	; UART0
	.ds	5
	ljmp	. + __offset	; Timer2 overflow
	.ds	5
	ljmp	. + __offset	; SPI0
	.ds	5
	ljmp	. + __offset	; SMB0
	.ds	5
	ljmp	. + __offset	; RTC alarm
	.ds	5
	ljmp	. + __offset	; ADC0 comparator
	.ds	5
	ljmp	. + __offset	; ADC0 conversion
	.ds	5
	ljmp	. + __offset	; PCA
	.ds	5
	ljmp	. + __offset	; Comparator0
	.ds	5
	ljmp	. + __offset	; Comparator1
	.ds	5
	ljmp	. + __offset	; Timer3 overflow
	.ds	5
	ljmp	. + __offset	; VDD_MCU early warning
	.ds	5
	ljmp	. + __offset	; Port Match
	.ds	5
	ljmp	. + __offset	; RTC oscillator fail
	.ds	5
	ljmp	. + __offset	; SPI1
	.ds	5
	ljmp	. + __offset	; Pulse Counter
	.ds	5
	ljmp	. + __offset	; DMA0
	.ds	5
	ljmp	. + __offset	; Encoder0
	.ds	5
	ljmp	. + __offset	; AES
	.ds	5

	;; bootloader entry
	.globl	_bl_main

	.area	GSINIT0	(CODE)

L__start:
	mov	sp, #L__stack - 1
	lcall	_bl_main
	sjmp	.
