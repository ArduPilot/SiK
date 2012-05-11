#!/usr/bin/env python
# put a HopeRF into update mode

import serial, sys, optparse, time, fdpexpect

parser = optparse.OptionParser("update_mode")
parser.add_option("--baudrate", type='int', default=57600, help='baud rate')
parser.add_option("--rtscts", action='store_true', default=False, help='enable rtscts')
parser.add_option("--dsrdtr", action='store_true', default=False, help='enable dsrdtr')
parser.add_option("--xonxoff", action='store_true', default=False, help='enable xonxoff')

opts, args = parser.parse_args()

if len(args) == 0:
    print("usage: update_mode.py <DEVICE...>")
    sys.exit(1)


def update_mode(device):
    '''put a HM-TRP into update mode'''
    port = serial.Serial(device, opts.baudrate, timeout=0,
                     dsrdtr=opts.dsrdtr, rtscts=opts.rtscts, xonxoff=opts.xonxoff)
    ser = fdpexpect.fdspawn(port.fileno(), logfile=sys.stdout)
    ser.send('+++')
    time.sleep(1)
    ser.send('\r\nATI\r\n')
    try:
        ser.expect(['OK','SiK .* on HM-TRP'], timeout=2)
    except fdpexpect.TIMEOUT:
        print("timeout")
        return
    ser.send('AT&UPDATE\r\n')
    try:
        buf = ser.read_nonblocking(100, timeout=1)
    except pexpect.TIMEOUT:
        pass
    port.close()


for d in args:
    print("Putting %s into update mode" % d)
    update_mode(d)
    
