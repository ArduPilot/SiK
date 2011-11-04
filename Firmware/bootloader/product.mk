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
PRODUCT		 =	bootloader.$(BOARD)
PRODUCT_DIR	:=	$(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
PRODUCT_INSTALL = $(foreach frequency,$(FREQUENCIES), $(OBJROOT)/$(PRODUCT).$(frequency).hex)				

CFLAGS		+=	-DBL_VERSION=$(VERSION)
CFLAGS		+=	--model-small --no-xinit-opt --opt-code-size --Werror
#CFLAGS		+=	--fverbose-asm

LDFLAGS		 =	--iram-size 256 --xram-size 4096 --code-size 0x000400 --stack-size 64 --nostdlib

include $(SRCROOT)/include/rules.mk

#
# Patch the frequency code into the hex file.
#
# This depends on a long chain of careful hackery.
# - The search pattern depends on the linker emitting a separate line
#   in the hex file for the frequency code.  This is normally safe 
#   because it's alone the end of the first flash page and the linker
#   doesn't emit un-written bytes.
# - The frequency codes are, in hex, the first two digits of the decimal
#   frequency (i.e. frequency in tens of MHz).
#
# XXX it's busted.  The checksum isn't being computed correctly.
#
$(PRODUCT_INSTALL):	frequency = $(word 3, $(subst ., ,$(notdir $@)))
$(PRODUCT_INSTALL):	$(PRODUCT_HEX)
	@echo PATCH $@
	$(v)mkdir -p $(dir $@)
	$(v)$(SRCROOT)/tools/hexpatch --patch 0x3ff:0x`expr $(frequency) / 10` $(PRODUCT_HEX) > $@
	