#!/usr/bin/env python
# a trivial serial console

import serial, sys, optparse

parser = optparse.OptionParser("sercat")
parser.add_option("--baudrate", type='int', default=57600, help='baud rate')

opts, args = parser.parse_args()

if len(args) != 1:
    print("usage: console.py <DEVICE>")
    sys.exit(1)

device = args[0]

port = serial.Serial(device, opts.baudrate, timeout=1,
                     dsrdtr=False, rtscts=False, xonxoff=False)
while True:
    count = port.inWaiting()
    if count == 0:
        count = 1
    buf = port.read(count)
    sys.stdout.write(buf)
    sys.stdout.flush()
    


