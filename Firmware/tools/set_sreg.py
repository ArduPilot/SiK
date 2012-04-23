#!/usr/bin/env python
# set air data rate

import serial, sys, optparse, time, fdpexpect

parser = optparse.OptionParser("set_speed")
parser.add_option("--baudrate", type='int', default=57600, help='baud rate')
parser.add_option("--cmd", action='append', default=[], help='at command')
parser.add_option("--reset", action='store_true', help='reset after set')
parser.add_option("--write", action='store_true', help='write after set')

opts, args = parser.parse_args()

if len(args) == 0:
    print("usage: set_sreg.py <DEVICE...>")
    sys.exit(1)


def set_speed(device):
    '''set some registers'''
    port = serial.Serial(device, opts.baudrate, timeout=0,
                         dsrdtr=False, rtscts=True, xonxoff=False)
    ser = fdpexpect.fdspawn(port.fileno(), logfile=sys.stdout)
    ser.send('+++')
    time.sleep(1)
    ser.send('\r\nATI\r\n')
    try:
        ser.expect(['OK','SiK .* on HM-TRP'], timeout=2)
    except fdpexpect.TIMEOUT:
        print("timeout")
        return
    for cmd in opts.cmd:
        ser.send('%s\r\n' % cmd)
    ser.expect('OK')
    if opts.write:
        ser.send('AT&W\r\n')
        ser.expect('OK')
    if opts.reset:
        ser.send('ATZ\r\n')
    else:
        ser.send('ATO\r\n')
        ser.expect('OK')
    time.sleep(1)
    ser.read_nonblocking(300)
    port.close()

for d in args:
    print("Setting %s on %s" % (opts.cmd, d))
    set_speed(d)
    
