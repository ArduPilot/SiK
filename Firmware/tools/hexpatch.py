#!/usr/bin/env python3
#
# IntelHex patcher.
#
#   hexpatch --patch <address>:<data>[,<address>:<data>...]
# 
# where <address> is the address at which the byte <data> should be written.  Results are
# written to stdout.
#

import argparse, sys, binascii, struct

class ihrange(object):
    '''Comprehend a line of IntelHex'''
    def __init__(self, line):
        # parse the header off the line
        self.line = line
        self.hexstr = line.rstrip()[1:-2]
        self.binstr = binascii.unhexlify(self.hexstr)
        self.count, self.address, self.command = struct.unpack(">BHB", self.binstr[:4])

        # regular data line
        if self.command == 0:
            self.bytes = bytearray(self.binstr[4:])
        else:
            self.bytes = None

    def __str__(self):
        # only data lines can be patched
        if (self.command != 0):
            return self.line

        # reconstruct the line from the patched version
        hexstr = struct.pack(">BHB", self.count, self.address, self.command) + bytearray(self.bytes)
        sum = 0
        for c in hexstr:
            sum += c
        sum = (256 - (sum % 256)) % 256
        hexstr += bytearray([sum])
        return ":" + hexstr.hex().upper()

    def patch(self, address, value):
        # only worth patching if command is zero
        if (self.command != 0):
            return
        if address >= self.address and address < self.address + self.count:
            self.bytes[address - self.address] = value


# grab patches from the commandline
parser = argparse.ArgumentParser(description="IntelHex patcher")
parser.add_argument("--patch", action = "store", required = True, help = "comma-separated list of <address>:<value> byte patches")
parser.add_argument("inputfile", action = "store", help = "File to patch")
parser.add_argument("outputfile", action = "store", help = "outputfile")
args = parser.parse_args()

patches = args.patch.split(",")
fin = open(args.inputfile, mode="r")
fout = open(args.outputfile, mode='w')

# iterate the file
for line in fin:
    range = ihrange(line)
    for p in patches:
        address, value = tuple(p.split(":"))
        range.patch(int(address, 0), int(value, 0))
    fout.write(str(range)+"\n")
