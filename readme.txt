EZRadio simple TRx example using the EZRadio device in the EZR32 device.

This example project uses the EZR32LG CMSIS including emlib, emdrv and the
board support package support functions/drivers to demonstrate driving
the EZRadio or EZRadioPRO device in the EZR32 device.

It is advised to use two boards with same configuration and firmware, so
both packet transmission and reception functionality can be tested.

The user can send one packet with the radio by pushing the PB0 button. The
data counter in the packet is incremented and shown on the LCD. If the
packet is received by the other device, the received data is shown on its
LCD. If the packet is received with CRC error this is printed as well.

The user can send specified or unlimited number of packets by pushing PB1.
The user can cease the transmission by pushing PB1 again.
The number of transmitted packets is configurable.

The application implements Packet Trace functionality, that means that the
user can observe transmitted and received packets with the Network Analyzer Tool
of Simplicity studio using the built-in default radio configuration header files.

The sample app is able to consume radio configuration header files generated
with either Simplicity Studio or Wireless Development Suite. In order to use
the generated header file instead of the default one enable
RADIO_USE_GENERATED_CONFIGURATION in ezradio_plugin_manager.h.

For more information please refer to the Quick Start Guide of the application.

Board:  Silicon Labs SLWSTK6202A_EZR32LG Development Kit
Device: EZR32LG330F256R63
