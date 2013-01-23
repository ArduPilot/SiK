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
# Common rules and targets for the HopeRF radio apps
#

ifeq ($(BOARD),)
$(error Must define BOARD before attempting to build)
endif
ifeq ($(SRCROOT),)
$(error Must define SRCROOT before attempting to build)
endif

#
# Get board-specific definitions
#
include $(SRCROOT)/include/rules_$(BOARD).mk

#
# Common build options.
#
CFLAGS		+=	-DBOARD_$(BOARD)

#
# Paths.
#
OBJROOT		?=	$(SRCROOT)/obj/$(BOARD)/$(PRODUCT)
DSTROOT		?=	$(SRCROOT)/dst

#
# Buildable and installable products
#
PRODUCT_HEX	 =	$(OBJROOT)/$(PRODUCT).ihx
PRODUCT_INSTALL	?=	$(PRODUCT_HEX)

#
# Compiler and tools
#
ifeq ($(shell which sdcc),)
$(error Could not find SDCC on your path - cannot build)
endif
CC		 =	sdcc -mmcs51
AS		 =	sdas8051 -jloscp
LD		 =	sdcc
INCLUDES	 =	$(SRCROOT)/include
CFLAGS		+=	$(addprefix -I,$(INCLUDES))
DEPFLAGS	 =	-MM $(CFLAGS)

#
# Assembly source/objects must come first to ensure startup files
# can be in front.
#
ASRCS		+=	$(sort $(wildcard $(PRODUCT_DIR)/*.asm))
OBJS		+=	$(patsubst $(PRODUCT_DIR)/%.asm,$(OBJROOT)/%.rel,$(ASRCS))

CSRCS		+=	$(wildcard $(PRODUCT_DIR)/*.c)
OBJS		+=	$(patsubst $(PRODUCT_DIR)/%.c,$(OBJROOT)/%.rel,$(CSRCS))

ifeq ($(VERBOSE),)
v		 =	@
else
CFLAGS		+=	-V
endif

#
# Build rules
#
build:	$(PRODUCT_HEX)

$(PRODUCT_HEX):	$(OBJS)
	@echo LD $@
	@mkdir -p $(dir $@)
	$(v)$(LD) -o $@ $(LDFLAGS) $(OBJS)

$(OBJROOT)/%.rel: $(PRODUCT_DIR)/%.c
	@echo CC $<
	@mkdir -p $(dir $@)
	$(v)(/bin/echo -n $(OBJROOT)/ && $(CC) $(DEPFLAGS) $<) > $(subst .rel,.dep,$@)
	$(v)$(CC) -c -o $@ $(CFLAGS) $<

$(OBJROOT)/%.rel: $(PRODUCT_DIR)/%.asm
	@echo AS $<
	@mkdir -p $(dir $@)
	$(v)cp $< $(subst $(PRODUCT_DIR),$(OBJROOT),$<)
	$(v)$(AS) $(ASFLAGS) $(subst $(PRODUCT_DIR),$(OBJROOT),$<)

clean:
	$(v)rm -rf $(OBJROOT)

install:	$(PRODUCT_INSTALL)
	@echo INSTALL $^
	$(v)mkdir -p $(DSTROOT)
	$(v)cp $(PRODUCT_INSTALL) $(DSTROOT)/

#
# Dependencies
#
GLOBAL_DEPS	+=	$(MAKEFILE_LIST)
$(OBJS):	$(GLOBAL_DEPS)
-include $(wildcard $(OBJROOT)/*.dep)
