#!/usr/bin/env python

'''
test sending of binary data
'''

import sys, time, os, serial

from optparse import OptionParser
parser = OptionParser("mavtester.py [options]")

parser.add_option("--baudrate", type='int',
                  help="connection baud rate", default=57600)
parser.add_option("--port1", default=None, help="serial port 1")
parser.add_option("--port2", default=None, help="serial port 2")
parser.add_option("--blocksize", type='int', default=5000, help="blocksize")
parser.add_option("--timeout", type='int', default=8, help="timeout")

(opts, args) = parser.parse_args()


port = []
port.append(serial.Serial(opts.port1, opts.baudrate, timeout=5, dsrdtr=False, rtscts=True, xonxoff=False))
port.append(serial.Serial(opts.port2, opts.baudrate, timeout=5, dsrdtr=False, rtscts=True, xonxoff=False))

r = open("/dev/urandom")

def read_timeout(p, length, timeout):
    '''read len bytes with given timeout'''
    t_start = time.time()
    ret = ''
    while (time.time() < t_start + timeout):
        b = p.read(length)
        if len(b) == 0:
            print("short read")
            break
        ret += b
        length -= len(b)
        if length == 0:
            break
    return ret

while True:
    b = r.read(opts.blocksize)
    t0 = time.time()
    port[0].write(b)
    b2 = read_timeout(port[1], opts.blocksize, opts.timeout)
    port[1].write(b)
    b3 = read_timeout(port[0], opts.blocksize, opts.timeout)
    t1 = time.time()
    print(len(b), len(b2), len(b3), ' %.3f kByte/sec' % (1.0e-3*opts.blocksize*2/(t1-t0)))
    if b != b2:
        print("data error b2", b[:8], b2[:8])
        break
    if b != b3:
        print("data error b3", b[:8], b3[:8])
        break
    time.sleep(1)
