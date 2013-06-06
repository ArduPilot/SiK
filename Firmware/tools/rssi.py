#!/usr/bin/env python
# RSSI production test

import serial, sys, optparse, time, fdpexpect

parser = optparse.OptionParser("update_mode")
parser.add_option("--baudrate", type='int', default=57600, help='baud rate')
parser.add_option("--rtscts", action='store_true', default=False, help='enable rtscts')
parser.add_option("--dsrdtr", action='store_true', default=False, help='enable dsrdtr')
parser.add_option("--xonxoff", action='store_true', default=False, help='enable xonxoff')

opts, args = parser.parse_args()

if len(args) == 0:
    print("usage: rssi.py <DEVICE...>")
    sys.exit(1)


def rssi(device):
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
    ser.send('AT&F\r\n')
    try:
        ser.expect(['OK'], timeout=2)
    except fdpexpect.TIMEOUT:
        print("timeout")
        return
    
    ser.send('AT&T=RSSI\r\n')
    
    ctr = 0
    
    while ctr < 200:
    	try:
	        count = port.inWaiting()
	        if count == 0:
        	    	count = 1
        	buf = port.read(count)
        	if len(buf) == 0:
            		continue
        	sys.stdout.write(buf)
            	sys.stdout.flush()
        	ctr = ctr + 1
        	
    	except KeyboardInterrupt:
	        sys.exit(0)
	        
    port.close()


for d in args:
    print("Putting %s into rssi test mode" % d)
    rssi(d)
    
