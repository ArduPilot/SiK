#!/usr/bin/env python
# reflect input bytes to output, printing as it goes

import serial, sys, optparse

parser = optparse.OptionParser("reflector")
parser.add_option("--baudrate", type='int', default=115200, help='baud rate')
parser.add_option("--echo", action='store_true', default=False, help='echo to stdout')

opts, args = parser.parse_args()

if len(args) != 1:
    print("usage: reflector.py <DEVICE>")
    sys.exit(1)

device = args[0]

port = serial.Serial(device, opts.baudrate, timeout=0,
                     dsrdtr=False, rtscts=False, xonxoff=False)

while True:
    try:
        buf = port.read()
        if opts.echo:
            sys.stdout.write(buf)
        port.write(buf)
    except KeyboardInterrupt:
        sys.exit(0)
        
