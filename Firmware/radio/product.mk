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
# Makefile for the Si1000 radio application
#

VERSION_MAJOR	 =	1
VERSION_MINOR	 =	6

PRODUCT		 =	radio~$(BOARD)
PRODUCT_DIR	:=	$(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))

CFLAGS		+=	-DAPP_VERSION_HIGH=$(VERSION_MAJOR) -DAPP_VERSION_LOW=$(VERSION_MINOR)
CFLAGS		+=	--model-large --opt-code-speed --Werror --std-sdcc99 --fomit-frame-pointer
#CFLAGS		+=	--fverbose-asm 

LDFLAGS		+=	 --model-large --iram-size 256 --xram-size 4096 --code-loc 0x400 --code-size 0x00f400 --stack-size 64

include $(SRCROOT)/include/rules.mk
