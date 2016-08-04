
TOOLPATH :=  $(subst :, ,$(PATH))
FIND_TOOL = $(firstword $(wildcard $(addsuffix /$(1),$(TOOLPATH))))
CCACHE :=  $(call FIND_TOOL,ccache)

CC=$(CCACHE) arm-none-eabi-gcc
OPTFLAGS=-g -gdwarf-2 -Os
CPUFLAGS=-mcpu=cortex-m3 -mthumb -Wall -fmessage-length=0 -ffunction-sections -fdata-sections -std=c99
DEPFLAGS=-MD -MP -MT $@
EZRFLAGS=-DEZRADIO_DRIVER_EXTENDED_SUPPORT=1 -DEFM32_HFXO_FREQ=28000000UL -DEZRADIO_DRIVER_FULL_SUPPORT=1 -DRADIO_USE_GENERATED_CONFIGURATION=1 -DEZR32LG330F128R68=1
INCFLAGS=-Isrc -IAES -IRFD900 -IRadioConfig -IDriversLocal/inc -ISDK/EZR32LG/Include -ISDK/CMSIS/Include -ISDK/emlib/inc -ISDK/emdrv/common/inc -ISDK/emdrv/ezradiodrv/common/inc/si4x6x -ISDK/emdrv/rtcdrv/inc -ISDK/emdrv/config -ISDK/emdrv/spidrv/inc -ISDK/emdrv/dmadrv/inc -ISDK/emdrv/gpiointerrupt/inc -ISDK/emdrv/ezradiodrv/si4463/inc -ISDK/emdrv/ezradiodrv/common/inc -ISDK/common/drivers -ISDK/emdrv/ustimer/inc

CFLAGS=$(OPTFLAGS) $(CPUFLAGS) $(EZRFLAGS) $(INCFLAGS) $(DEPFLAGS)

UTILSRC=src/PWM.c src/RCOComp.c src/main.c src/xprintf.c
EMLIBSRC=SDK/emlib/src/em_adc.c SDK/emlib/src/em_assert.c SDK/emlib/src/em_cmu.c SDK/emlib/src/em_dma.c SDK/emlib/src/em_emu.c SDK/emlib/src/em_gpio.c SDK/emlib/src/em_int.c emlib/em_letimer.c emlib/em_msc.c SDK/emlib/src/em_pcnt.c SDK/emlib/src/em_prs.c SDK/emlib/src/em_rtc.c SDK/emlib/src/em_system.c SDK/emlib/src/em_timer.c emlib/em_usart.c SDK/emlib/src/em_wdog.c
RFD900SRC=RFD900/at.c RFD900/crc.c RFD900/flash.c RFD900/freq_hopping.c RFD900/golay.c RFD900/mavlink.c RFD900/packet.c RFD900/parameters.c RFD900/pins_user.c RFD900/printfl.c RFD900/radio_old.c RFD900/serial2.c RFD900/tdm.c RFD900/timer.c RFD900/ppm.c RFD900/relay.c
DRIVERSSRC=DriversLocal/dmactrl.c DriversLocal/dmadrv.c DriversLocal/ezradio_api_lib.c DriversLocal/ezradio_auto_ack_plugin.c DriversLocal/ezradio_comm.c DriversLocal/ezradio_crcerror_plugin.c DriversLocal/ezradio_direct_receive_plugin.c DriversLocal/ezradio_direct_transmit_plugin.c DriversLocal/ezradio_hal.c DriversLocal/ezradio_plugin_manager.c DriversLocal/ezradio_pn9_plugin.c DriversLocal/ezradio_receive_plugin.c DriversLocal/ezradio_transmit_plugin.c DriversLocal/ezradio_unmod_carrier_plugin.c DriversLocal/gpiointerrupt.c DriversLocal/rtcdriver.c DriversLocal/spidrv.c DriversLocal/udelay.c DriversLocal/si4x6x/ezradio_api_lib_add.c
SYSTEMSRC=SDK/EZR32LG/Source/system_ezr32lg.c
AESSRC=AES/aes.c AES/aes_ctr_128.c

ASMSRC=SDK/EZR32LG/Source/GCC/startup_ezr32lg.S

ALL_CSRC=$(UTILSRC) $(EMLIBSRC) $(RFD900SRC) $(DRIVERSSRC) $(SYSTEMSRC) $(AESSRC)
ALL_ASMSRC=$(ASMSRC)

ALLOBJ=$(ALL_CSRC:%.c=%.o) $(ALL_ASMSRC:%.S=%.o)

.S.o:
	@echo Assembling $*.S
	@$(CC) $(CFLAGS) -x assembler-with-cpp -c -o $*.o $*.S

.c.o:
	@echo Compiling $*.c
	@$(CC) $(CFLAGS) -c -o $*.o $*.c

all: SiK900x.hex SiK900x.bin

SiK900x.axf: $(ALLOBJ)
	@echo Linking $@
	@$(CC) $(CFLAGS) -Tloader.ld -Xlinker --gc-sections -Xlinker -Map=SiK900x.map -lm -lgcc -lc -o $@ $(ALLOBJ) -lm -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@arm-none-eabi-size $@

SiK900x.hex: SiK900x.axf
	@echo Creating $@
	@arm-none-eabi-objcopy -O ihex $^ $@

SiK900x.bin: SiK900x.axf
	@echo Creating $@
	@arm-none-eabi-objcopy -O binary $^ $@

clean:
	/bin/rm -f $(ALLOBJ) SiK900x.axf SiK900x.bin SiK900x.hex
