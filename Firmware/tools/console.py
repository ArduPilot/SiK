#!/usr/bin/env python
# a trivial serial console

import serial, sys, optparse, fdpexpect

parser = optparse.OptionParser("console")
parser.add_option("--baudrate", type='int', default=115200, help='baud rate')

opts, args = parser.parse_args()

if len(args) != 1:
    print("usage: console.py <DEVICE>")
    sys.exit(1)

device = args[0]

port = serial.Serial(device, opts.baudrate, timeout=0,
                     dsrdtr=False, rtscts=False, xonxoff=False)

ser = fdpexpect.fdspawn(port.fileno(), maxread=1)
ser.delaybeforesend = 0
print("Connecting (use ^] to exit)")
ser.interact()

