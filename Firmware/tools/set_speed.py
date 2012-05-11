#!/usr/bin/env python
# set air data rate

import serial, sys, optparse, time, fdpexpect

parser = optparse.OptionParser("set_speed")
parser.add_option("--baudrate", type='int', default=57600, help='baud rate')
parser.add_option("--speed", type='int', default=128, help='air speed')
parser.add_option("--rtscts", action='store_true', default=False, help='enable rtscts')
parser.add_option("--dsrdtr", action='store_true', default=False, help='enable dsrdtr')
parser.add_option("--xonxoff", action='store_true', default=False, help='enable xonxoff')

opts, args = parser.parse_args()

if len(args) == 0:
    print("usage: set_speed.py <DEVICE...>")
    sys.exit(1)


def set_speed(device):
    '''set air speed'''

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
    ser.send('ATS2=%u\r\n' % opts.speed)
    ser.expect('OK')
    ser.send('AT&W\r\n')
    ser.expect('OK')
    ser.send('ATI5\r\n')
    time.sleep(0.2)
    ser.read_nonblocking(100)
    ser.send('ATZ\r\n')
    time.sleep(1)
    port.close()

for d in args:
    print("Setting speed %u on %s" % (opts.speed, d))
    set_speed(d)
    
