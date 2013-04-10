#!/usr/bin/env python
# reflect input bytes to output, printing as it goes

import serial, sys, optparse, time

parser = optparse.OptionParser("davis_log")
parser.add_option("--baudrate", type='int', default=57600, help='baud rate')

opts, args = parser.parse_args()

if len(args) != 2:
    print("usage: reflector.py <DEVICE> <logfile>")
    sys.exit(1)

device = args[0]
logfile = args[1]

port = serial.Serial(device, opts.baudrate, timeout=5, dsrdtr=False, rtscts=False, xonxoff=False)

log = open(logfile, mode="a")

while True:
    line = port.readline()
    line = line.rstrip()
    out = "%s %.2f\n" % (line, time.time())
    log.write(out);
    log.flush()
    sys.stdout.write(out)
    sys.stdout.flush()
