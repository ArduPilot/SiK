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

SDCC		 =	/usr/local
EC2TOOLS	 =	/Users/msmith/bin

CC		 =	$(SDCC)/bin/sdcc -mmcs51
AS		 =	$(SDCC)/bin/sdas8051 -jloscp
LD		 =	$(SDCC)/bin/sdcc
EC2FLASH	 =	$(EC2TOOLS)/ec2writeflash

DEPFLAGS	 =	-MM $(CFLAGS)

INCLUDES	 =	$(SRCROOT)/../include
CFLAGS		+=	$(addprefix -I,$(INCLUDES))

EC2FLASH_ARGS	 =	--port=USB --mode=c2 --hex

ifeq ($(VERBOSE),)
v		=	@
else
CFLAGS		+=	-V
endif

all:	$(PRODUCT)

$(PRODUCT):	$(OBJS)
	@echo LD $@
	$(v)$(LD) -o $@ $(LDFLAGS) $(OBJS)

$(OBJROOT)/%.rel: $(SRCROOT)/%.c
	@echo CC $<
	$(v)$(CC) $(DEPFLAGS) $< | sed s%^%$(OBJROOT)/% > $(subst .rel,.dep,$@)
	$(v)$(CC) -c -o $@ $(CFLAGS) $<

$(OBJROOT)/%.rel: $(SRCROOT)/%.asm
	@echo AS $<
	$(v)cp $< $(OBJROOT)/$<
	$(v)$(AS) $(ASFLAGS) $(OBJROOT)/$<

.PHONY:	upload
upload:	$(PRODUCT)
	@echo UPLOAD $<
	$(v)$(EC2FLASH) $(EC2FLASH_ARGS) $<

clean:
	$(v)rm -rf $(OBJROOT) $(SRCROOT)/*~

-include $(wildcard $(OBJROOT)/*.dep)
