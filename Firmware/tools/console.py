#!/usr/bin/env python
# a trivial serial console

import serial, sys, optparse, fdpexpect

parser = optparse.OptionParser("console")
parser.add_option("--baudrate", type='int', default=57600, help='baud rate')
parser.add_option("--rtscts", action='store_true', default=False, help='enable rtscts')
parser.add_option("--dsrdtr", action='store_true', default=False, help='enable dsrdtr')
parser.add_option("--xonxoff", action='store_true', default=False, help='enable xonxoff')

opts, args = parser.parse_args()

if len(args) != 1:
    print("usage: console.py <DEVICE>")
    sys.exit(1)

device = args[0]

port = serial.Serial(device, opts.baudrate, timeout=0,
                     dsrdtr=opts.dsrdtr, rtscts=opts.rtscts, xonxoff=opts.xonxoff)

ser = fdpexpect.fdspawn(port.fileno(), maxread=1)
ser.delaybeforesend = 0
print("Connecting (use ^] to exit)")
ser.interact()

