#!/usr/bin/env python

import serial, sys, optparse, time

parser = optparse.OptionParser("davis_show")
parser.add_option("--type", type='int', default=None, help='msg type to show')

opts, args = parser.parse_args()

logfile = args[0]

def swap_bit_order(b):
    b = ((b & 0xF0) >> 4) | ((b & 0x0F) << 4)
    b = ((b & 0xCC) >> 2) | ((b & 0x33) << 2)
    b = ((b & 0xAA) >> 1) | ((b & 0x55) << 1)
    return b



def crc16_ccitt(buf):
    crc = 0
    for b in buf:
        crc ^= b << 8
        for i in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
        crc &= 0xFFFF
    return crc

def wind_speed(b):
    return b ^ 0xE

def wind_direction(b):
    n1 = [0xD, 0xE, 0xF, 0x0, 0x9, 0xA, 0xB, 0xC, 0x5, 0x6, 0x7, 0x8, 0x1, 0x2, 0x3, 0x4]
    n2 = [0xD, 0xC, 0xF, 0xE, 0x9, 0x8, 0xB, 0xA, 0x5, 0x4, 0x7, 0x6, 0x1, 0x0, 0x3, 0x2]
    v = (n1[b>>4]<<4) | n2[b&0xF]
    return v * 360 / 255

def decode(b1, b2):
    '''decode a 12 bit value from bytes 2 and 3'''
    n1 = [ 0xF, 0xE, 0xD, 0xC, 0xB, 0xA, 0x9, 0x8, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0 ]
    n2 = [ 0x6, 0x7, 0x4, 0x5, 0x2, 0x3, 0x0, 0x1, 0xE, 0xF, 0xC, 0xD, 0xA, 0xB, 0x8, 0x9 ]
    n3 = [ 0xC, 0xD, 0xE, 0xF, 0x8, 0x9, 0xA, 0xB, 0x4, 0x5, 0x6, 0x7, 0x0, 0x1, 0x2, 0x3 ]
    v = (n1[b1>>4]<<8) | (n2[b1&0xF]<<4) | n3[b2>>4]
    return v

def temperature(b1, b2):
    v = decode(b1, b2)
    farenheit = v / 7.0 - 30.0
    celcius = (farenheit - 32) * 5 / 9
    return celcius

def process_line(line):
    a=line.split()
    bytes=[]
    for i in range(10):
        try:
            bytes.append(int(a[i],16))
        except Exception:
            print "INVALID %s" % line
            bytes.append(0xFF)
    t = float(a[10])
    if opts.type is None:
        print "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %04x %.2f speed=%u wdirection=%u" % (
            bytes[0], bytes[1],
            bytes[2], bytes[3],
            bytes[4], bytes[5],
            bytes[6], bytes[7],
            bytes[8], bytes[9],
            crc16_ccitt(bytes[0:8]),
            t,
            wind_speed(bytes[1]),
            wind_direction(bytes[2]))
        return
    type = bytes[0] >> 4
    if type == opts.type or opts.type == -1:
        v = -1
        if type == 7:
            v = temperature(bytes[3], bytes[4])
        else:
            v = decode(bytes[3], bytes[4])
        print("%02X %X %X %X %X %X %X %.2f %s" % (
            bytes[0],
            bytes[3]>>4,
            bytes[3]&0xF,
            bytes[4]>>4,
            bytes[4]&0xF,
            bytes[5]>>4,
            bytes[5]&0xF,
            v,
            time.ctime(t)))

log = open(logfile, mode="r")
for line in log:
    line = line.lstrip('.')
    line = line.rstrip()
    if line.startswith('Search'):
        continue
    try:
        process_line(line)
    except Exception:
        pass
    
