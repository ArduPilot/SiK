SiK
=====
Firmware for SiLabs Si1000 - Si102x/3x/6x ISM radios

[![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/ArduPilot/SiK?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

SiK is a collection of firmware and tools for radios based on the cheap, versatile SiLabs Si1000 and Si1060 SoC.

## Branch Build Status
[![Build Status](http://jenkins.hovo.id.au/buildStatus/icon?job=SiK)](http://jenkins.hovo.id.au/job/SiK/)

## Documentation
For user documentation please see [ardupilot docs](http://ardupilot.org/copter/docs/common-sik-telemetry-radio.html)

Currently, it supports the following boards:

 - HopeRF HM-TRP
 - HopeRF RF50-DEMO
 - RFD900
 - RFD900u
 - RFD900p
 - [MLAB ISM01A](https://www.mlab.cz/module/ISM01A)

Adding support for additional boards should not be difficult.

Currently the firmware components include:

 - A bootloader with support for firmware upgrades over the serial interface.
 - Radio firmware with support for parsing AT commands, storing parameters and FHSS/TDM functionality

### AT commands

|Command| Variants| Function |
|-------|--------|----------|
|+++    | |Entering bootloader mode. Could be tested by sending AT, reply should be OK|
|RT     | |remote AT command - send it to the tdm, system to send to the remote radio |
|AT&F   | |  Restore default parameters |
|AT&W| | Write parameters to the flash memory | 
|AT&U | | Erase Flash signature forcing it into reprogram mode next reset |
|AT&P | | TDM change phase |
|AT&T | AT&T <br> AT&T=RSSI <br> AT&T=TDM |  disable all test modes <br> display RSSI stats <br> display TDM debug ) |
|AT&E | AT&E?  <br> AT&E= | Print_encryption_key <br> Set encryption key | 
|AT+ | AT+P= <br> AT+Cx=y <br> AT+Fx? <br> AT+L <br> AT+A |  set power level pwm to x immediately <br>  write calibration value <br> get calibration value <br> lock bootloader area if all calibrations written <br> RFD900 antenna diversity  |
|ATI0| | banner_string |
|ATI1| | version_string  |
|ATI2| | BOARD_ID |
|ATI3| | Board design frequency|
|ATI4| | Board boot loader version|
|ATI5| | Parameters |
|ATI6| | TDM timing |
|ATI7| | Show RSSI |
|ATP |  ATPx=O <br> ATPx=I <br> ATPx=R <br> ATPx=Cx | Set pin to output, turn mirroring off pulling pin to ground    |
|ATO | |    |
|ATS | ATS? <br> ATS= | <br> Set a parameter  |
|ATZ | | Generate a software reset    |

Up to date AT command processig is located in [at.c](Firmware/radio/at.c) source code.

## What You Will Need

 - A Mac OS X or Linux system for building.  Mac users will need the Developer Tools (Xcode) installed.
 - At least two Si1000 - Si102x/3x - based radio devices (just one radio by itself is not very useful).
 - A [SiLabs USB debug adapter](http://www.silabs.com/products/mcu/Pages/USBDebug.aspx).
 - [SDCC](http://sdcc.sourceforge.net/), version 3.1.0 or later.
 - [EC2Tools](https://github.com/SamwelOpiyo/ec2)
 - [Mono](http://www.mono-project.com/) to build and run the GUI firmware updater.
 - Python to run the command-line firmware updater.

Note that at this time, building on Windows systems is not supported.  If someone wants to contribute and maintain the necessary pieces that would be wonderful.

## Building Things

Type `make install` in the Firmware directory.  If all is well, this will produce a folder called `dst` containing bootloader and firmware images.

If you want to fine-tune the build process, `make help` will give you more details.

Building the SiK firmware generates bootloaders and firmware for each of the supported boards. Many boards are available tuned to specific frequencies, but have no way for software on the Si1000 to detect which frequency the board is configured for. In this case, the build will produce different versions of the bootloader for each board. It's important to select the correct bootloader version for your board if this is the case.

## Flashing and Uploading

The SiLabs debug adapter can be used to flash both the bootloader and the firmware. Alternatively, once the bootloader has been flashed the updater application can be used to update the firmware (it's faster than flashing, too).

The `Firmware/tools/ec2upload` script can be used to flash either a bootloader or firmware to an attached board with the SiLabs USB debug adapter.  Further details on the connections required to flash a specific board should be found in the `Firmware/include/board_*.h` header for the board in question.

To use the updater application, open the `SiKUploader/SikUploader.sln` Mono solution file, build and run the application. Select the serial port connected to your radio and the appropriate firmware `.hex` file for the firmware you wish to uploader.  You will need to get the board into the bootloader; how you do this varies from board to board, but it will normally involve either holding down a button or pulling a pin high or low when the board is reset or powered on.

For the supported boards:

 - HM-TRP: hold the CONFIG pin low when applying power to the board.
 - RF50-DEMO: hold the ENTER button down and press RST.
 - RFD900x: hold the BOOT/CTS pin low when applying power to the board.

The uploader application contains a bidirectional serial console that can be used for interacting with the radio firmware.

As an alternative to the Mono uploader, there is a Python-based command-line upload tool in `Firmware/tools/uploader.py`.

## Supporting New Boards

Take a look at `Firmware/include/board_*.h` for the details of what board support entails.  It will help to have a schematic for your board, and in the worst case, you may need to experiment a little to determine a suitable value for EZRADIOPRO_OSC_CAP_VALUE.  To set the frequency codes for your board, edit the corresponding `Firmware/include/rules_*.mk` file.

## Resources

SiLabs have an extensive collection of documentation, application notes and sample code available online.

Start at the [Si1000 product page](http://www.silabs.com/products/wireless/wirelessmcu/Pages/Si1000.aspx) or [Si102x/3x product page](http://www.silabs.com/products/wireless/wirelessmcu/Pages/Si102x-3x.aspx)

## Reporting Problems

Please use the GitHub issues link at the top of the [project page](http://github.com/tridge/SiK) to report any problems with, or to make suggestions about SiK.  I encourage you to fork the project and make whatever use you may of it.

## What does SiK mean?

It should really be Sik, since 'K' is the SI abbreviation for Kelvin, and what I meant was 'k', i.e. 1000.  Someday I might change it.
