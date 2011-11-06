#!/usr/bin/env python
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
		self.count = ord(self.binstr[0])
		self.address = (ord(self.binstr[1]) << 8) + ord(self.binstr[2])
		self.command = ord(self.binstr[3])

		# regular data line
		if (self.command == 0):
			self.bytes = list(self.binstr[4:])

	def __str__(self):
		# only data lines can be patched
		if (self.command != 0):
			return self.line

		# reconstruct the line from the patched version
		hexstr = ""
		hexstr += chr(self.count)
		hexstr += chr(self.address >> 8)
		hexstr += chr(self.address & 0xff)
		hexstr += chr(self.command)
		hexstr += "".join(self.bytes)
		sum = 0
		for c in hexstr:
			sum += ord(c)
		sum = (256 - (sum % 256)) % 256
		hexstr += chr(sum)
		return ":" + binascii.hexlify(hexstr).upper()

	def patch(self, address, value):
		# only worth patching if command is zero
		if (self.command != 0):
			return
		if ((address >= self.address) and (address < (self.address + self.count))):
			self.bytes[address - self.address] = chr(value)


# grab patches from the commandline
parser = argparse.ArgumentParser(description="IntelHex patcher")
parser.add_argument("--patch", action = "store", required = True, help = "comma-separated list of <address>:<value> byte patches")
parser.add_argument("inputfile", action = "store", help = "File to patch (results written to stdout)")
args = parser.parse_args()

patches = args.patch.split(",")
f = open(args.inputfile, mode="r")

# iterate the file
for line in f:
	range = ihrange(line)
	for p in patches:
		address, value = tuple(p.split(":"))
		range.patch(int(address, 0), int(value, 0))
	print(range)
