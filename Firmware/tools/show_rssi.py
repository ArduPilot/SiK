#!/usr/bin/env python
# set air data rate

import serial, sys, optparse, time, fdpexpect

parser = optparse.OptionParser("show_rssi")
parser.add_option("--baudrate", type='int', default=57600, help='baud rate')
parser.add_option("--rtscts", action='store_true', default=False, help='enable rtscts')
parser.add_option("--dsrdtr", action='store_true', default=False, help='enable dsrdtr')
parser.add_option("--xonxoff", action='store_true', default=False, help='enable xonxoff')

opts, args = parser.parse_args()

if len(args) == 0:
    print("usage: show_rssi.py <DEVICE...>")
    sys.exit(1)


def show_rssi(device):
    '''show RSSI'''
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
    ser.send('AT&T=RSSI\r\n')
    time.sleep(1)
    ser.read_nonblocking(300)
    ser.send('AT&T\r\n')
    time.sleep(0.5)
    ser.send('ATO\r\n')
    ser.read_nonblocking(300)
    print("")
    time.sleep(1)
    port.close()

for d in args:
    print("showing registers on %s" % (d))
    show_rssi(d)
    
