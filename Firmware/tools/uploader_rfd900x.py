#!/usr/bin/env python3
#
# Serial firmware uploader for the SiK900x xmodem bootloader
#

import sys, argparse, binascii, serial, glob, time, os
import xmodem
import math

class uploader(object):
    '''Uploads a firmware file to the SiK900x bootloader'''
    def __init__(self, portname, atbaudrate=57600, use_mavlink=False, mavport=0, debug=0):
        print("Connecting to %s" % portname)
        self.atbaudrate = atbaudrate
        self._debug = debug
        if use_mavlink:
            from pymavlink import mavutil
            self.port = mavutil.MavlinkSerialPort(portname, 115200, devnum=mavport,
                                                  timeout=3, debug=debug)
        else:
            self.port = serial.Serial(portname, atbaudrate, timeout=3)
        self.set_rtscts(False)
        self.setBaudrate(atbaudrate)
        self.fw_size = 0
        self.percent = 0

    def debug(self, s, level=1):
        '''write some debug text'''
        if self._debug >= level:
            print(s)

    def __send(self, s):
        for c in s:
            time.sleep(0.01)
            self.port.write(c.encode('utf-8'))

    def __recv(self, raise_error=True):
        start_time = time.time()
        c = self.port.read(1)
        if (len(c) < 1):
            self.debug("timeout waiting for data (%.2f seconds)" % (time.time() - start_time))
            if raise_error:
                raise RuntimeError("timeout waiting for data")
            return None
        #print("recv " + binascii.hexlify(c))
        return c

    def __recvline(self, raise_error=True):
        line = ''
        while True:
            c = self.__recv(raise_error=raise_error)
            if c is None or c in ['\r', '\n']:
                break
            line += c
        return line


    def __getSync(self, raise_error=True):
        return self.expect('ChipID:', 0.5)

    # attempt to get back into sync with the bootloader
    def __sync(self):
        self.debug("trying __sync")
        self.__send('\rCHIPID\r')
        return self.__getSync(raise_error=False)

    def expect(self, pattern, timeout):
        '''wait for a pattern with timeout, return True if found, False if not'''
        import re
        prog = re.compile(pattern)
        start = time.time()
        s = ''
        while time.time() < start + timeout:
            b = self.port.read(1).decode('utf-8')
            if len(b) > 0:
                sys.stdout.write(b)
                s += b
                if prog.search(s) is not None:
                    return True
            else:
                time.sleep(0.01)
        return False

    def send(self, s):
        '''write a string to port and stdout'''
        self.__send(s)
        sys.stdout.write(s)

    def set_rtscts(self, enable):
        '''enable/disable RTS/CTS if applicable'''
        try:
            self.port.setRtsCts(enable)
        except Exception:
            self.port.rtscts = enable

    def setBaudrate(self, baudrate):
        try:
            self.port.setBaudrate(baudrate)
        except Exception:
            # for pySerial 3.0, which doesn't have setBaudrate()
            self.port.baudrate = baudrate

    def autosync(self):
        '''use AT&UPDATE to put modem in update mode'''
        self.setBaudrate(self.atbaudrate)
        print("Trying autosync")
        self.send('\r')
        time.sleep(1.0)
        self.send('+++')
        self.expect('OK', timeout=2.0)
        for i in range(5):
            self.send('ATI\r')
            if not self.expect('RFD', timeout=2):
                print("Failed to get SiK banner")
                continue
            self.send('\r\n')
            time.sleep(0.2)
            self.port.flushInput()
            self.send('AT&UPDATE\r\n')
            time.sleep(0.7)
            self.port.flushInput()
            print("Sent update command")
            return True
        return False


    # verify whether the bootloader is present and responding
    def check(self):
        for i in range(3):
            try:
                if self.__sync():
                    print("Got sync")
                    return True
                self.autosync()
            except RuntimeError:
                self.autosync()
        return False

    def putc(self, c, timeout=1):
        return self.port.write(c)

    def getc(self, size, timeout=1):
        try:
            return self.port.read(size)
        except Exception:
            return None

    def progress(self, count, total):
        '''show progress bar'''
        pct = int((100*count) / total)
        if pct != self.percent:
            self.percent = pct
            hlen = (pct/2)
            hashes = '#' * math.floor(hlen)
            spaces = ' ' * math.floor(50-hlen)
            sys.stdout.write("[%s] %u/%u (%u%%)\r" % (
                    hashes + spaces,
                    count,
                    total,
                    pct))
            sys.stdout.flush()
        if count == total:
            print("")

    def callback(self, total_packets, success_count, error_count):
        self.progress(success_count, self.fw_size / 128)

    def upload(self, fw):
        print("Uploading %s" % fw)
        self.fw_size = os.path.getsize(fw)
        self.__send('UPLOAD\r')
        self.expect("Ready", 2)
        self.expect("\r\n", 1)
        f = open(fw, 'rb')
        xm = xmodem.XMODEM(self.getc, self.putc)
        xm.send(f, callback=self.callback)
        f.close()
        print("Booting new firmware")
        self.__send('BOOTNEW\r')

if __name__ == '__main__':
    # Parse commandline arguments
    parser = argparse.ArgumentParser(description="Firmware uploader for the SiK radio system.")
    parser.add_argument('--port', action="store", help="port to upload to")
    parser.add_argument("--baudrate", type=int, default=57600, help='baud rate')
    parser.add_argument("--debug", type=int, default=0, help='debug level')
    parser.add_argument("--mavlink", action='store_true', default=False, help='update over MAVLink')
    parser.add_argument("--mavport", type=int, default=0, help='MAVLink port number')
    parser.add_argument('firmware', action="store", help="Firmware file to be uploaded")
    args = parser.parse_args()

    import logging
    logging.basicConfig()

    ports = glob.glob(args.port)
    if not ports:
        print("No matching ports for %s" % args.port)
        sys.exit(1)
    # Connect to the device and identify it
    for port in glob.glob(args.port):
        print("uploading to port %s" % port)
        up = uploader(port, atbaudrate=args.baudrate,
                      use_mavlink=args.mavlink,
                      mavport=args.mavport,
                      debug=args.debug)
        if not up.check():
            print("Failed to contact bootloader")
            sys.exit(1)
        up.upload(args.firmware)
