#
# Copyright (c) 2011 Michael Smith, All Rights Reserved
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  o Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
#  o Redistributions in binary form must reproduce the above copyright 
#    notice, this list of conditions and the following disclaimer in 
#    the documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.
#
#
# Makefile for the Si1000 UART bootloader.
#

VERSION		 =	1
PRODUCT		 =	bootloader~$(BOARD)
PRODUCT_DIR	:=	$(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
PRODUCT_INSTALL	 =	$(foreach frequency,$(FREQUENCIES), $(OBJROOT)/$(PRODUCT)~$(frequency).hex)

CFLAGS		+=	-DBL_VERSION=$(VERSION)
CFLAGS		+=	--model-small --no-xinit-opt --opt-code-size --Werror
#CFLAGS		+=	--fverbose-asm

# Set limits for the low (CSEG) and high (HIGHCSEG) code segments.
#
CSEG_LIMIT	 =	$(shell printf %d 0x0400)	# start of application space
HIGHCSEG_LIMIT	 =	$(shell printf %d 0xfbdc)	# room for 32 calibration and four patch bytes at the end

# Note that code is split into two parts; low code at 0, and high code at 0xf800.
#
LDFLAGS		 =	--iram-size 256 --xram-size 4096 --stack-size 64 --nostdlib \
			-Wl -bHIGHCSEG=0xf800

include $(SRCROOT)/include/rules.mk

#
# Patch the frequency code into the hex file.
#
# Note that we have secret knowledge here that the frequency code byte is 
# located at 0xfbfe, and its specific encoding.
#
$(PRODUCT_INSTALL):	frequency = $(basename $(word 3, $(subst ~, ,$(notdir $@))))
$(PRODUCT_INSTALL):	$(PRODUCT_HEX)
	@echo PATCH $@
	$(v)mkdir -p $(dir $@)
	$(v)$(SRCROOT)/tools/hexpatch.py --patch 0xfbfe:0x`expr $(frequency) / 10` $(PRODUCT_HEX) > $@

#
# Check that the bootloader has not overflowed its allocated space
#
$(PRODUCT_INSTALL):	sizecheck

sizecheck:	mapfile = $(subst .ihx,.map,$(PRODUCT_HEX))
sizecheck:	cseg_base = $(shell printf %d 0x`grep ^CSEG $(mapfile) | cut -c 41-44`)
sizecheck:	cseg_size = $(shell printf %d 0x`grep ^CSEG $(mapfile) | cut -c 53-56`)
sizecheck:	cseg_end  = $(shell expr $(cseg_base) + $(cseg_size))
sizecheck:	highcseg_base = $(shell printf %d 0x`grep ^HIGHCSEG $(mapfile) | cut -c 41-44`)
sizecheck:	highcseg_size = $(shell printf %d 0x`grep ^HIGHCSEG $(mapfile) | cut -c 53-56`)
sizecheck:	highcseg_end  = $(shell expr $(highcseg_base) + $(highcseg_size))
sizecheck:	$(PRODUCT_HEX)
	@echo SIZECHECK $<: CSEG $(cseg_size) HIGHCSEG $(highcseg_size)
	$(v)test $(cseg_end)     -lt $(CSEG_LIMIT)     || (echo error: CSEG too large; exit 1)
	$(v)test $(highcseg_end) -lt $(HIGHCSEG_LIMIT) || (echo error: HIGHCSEG too large; exit 1)
