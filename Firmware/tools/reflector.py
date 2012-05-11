#!/usr/bin/env python
# reflect input bytes to output, printing as it goes

import serial, sys, optparse

parser = optparse.OptionParser("reflector")
parser.add_option("--baudrate", type='int', default=57600, help='baud rate')
parser.add_option("--echo", action='store_true', default=False, help='echo to stdout')
parser.add_option("--rtscts", action='store_true', default=False, help='enable rtscts')
parser.add_option("--dsrdtr", action='store_true', default=False, help='enable dsrdtr')
parser.add_option("--xonxoff", action='store_true', default=False, help='enable xonxoff')

opts, args = parser.parse_args()

if len(args) != 1:
    print("usage: reflector.py <DEVICE>")
    sys.exit(1)

device = args[0]

port = serial.Serial(device, opts.baudrate, timeout=1,
                     dsrdtr=opts.dsrdtr, rtscts=opts.rtscts, xonxoff=opts.xonxoff)

while True:
    try:
        count = port.inWaiting()
        if count == 0:
            count = 1
        buf = port.read(count)
        if len(buf) == 0:
            continue
        if opts.echo:
            sys.stdout.write(buf)
            sys.stdout.flush()
        port.write(buf)
        port.flush()
    except KeyboardInterrupt:
        sys.exit(0)
        
