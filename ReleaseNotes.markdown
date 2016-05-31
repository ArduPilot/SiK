#Release Notes:

##SiK 1.11:

###Alterations
* Adding support for new rfd900p (plus) radio modem

##SiK 1.10:

###Alterations
* Fixed bug that could have caused stability issues

###NEW FEATURES!!
* Added new parameter segment for AT Parameters
    
    ATS will be used for Andrew Tridgell branch
    
    ATR will be used for RFDesign's new features

* ATR0 Target RSSI, This will change the TXPower (ATS4) until target is found. Set ATS4 to the maximum power allowed (Default 255).
* ATR1 Hysteresis RSSI, The level of change from the target before the power level is changed again.

##SiK 1.9:

###Alterations
* Golay and ECC restored using new efficent code

##SiK 1.8:

###Alterations
* Golay removed, can be added back using uncommenting #define in golay23.h
* ECC - Setting ECC now causes a error due to it being removed

###NEW FEATURES!!
* Added support for new CPU Si102x/3x (For new Product RFD900u)
* Users can now controll unused pins. This can be preformed by the following commands

Command       | Function | Description
------------- | ---------|-------------
ATPP          | Print    | Print All Pins Settings
ATPI=1        | Input    | Set Pin 1 to Input
ATPR=1        | Read     | Read Pin 1 value (When set to input)
ATPO=2        | Output   | Set Pin 2 to Output (Output's by Default can only be controlled by AT cmd)
ATPC=2,1      | Control  | Turn pin 2 on  - Output Mode / Set internal pull up resistor - Input Mode 
ATPC=2,0      | Control  | Turn pin 2 off - Output Mode / Set internal pull down resistor - Input Mode

Mapping between the pin numbers above and the port number are below

######RFD900
Pin  | Port
---- | ----
0    | 2.3
1    | 2.2
2    | 2.1
3    | 2.0
4    | 2.6
5    | 0.1

######RFD900u
Pin  | Port
---- | ----
0    | 1.0
1    | 1.1

##SiK 1.7:

###Improvements
* Altered timing for better throughput
* Removed support for MAVLink 0.9 support freeing up code space
* Updated Config to use a CRC instead of the XOR 
