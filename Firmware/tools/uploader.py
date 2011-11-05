#!/usr/bin/env python
#
# Serial firmware uploader for the SiK bootloader
#

import sys, argparse, binascii, serial

class firmware(object):
	'''Loads a firmware file'''

	# parse a single IntelHex line and obtain the byte array and address
	def __parseline(self, line):
		# ignore lines not beginning with :
		if (line[0] != ":"):
			return;
		# parse the header off the line
		hexstr = line.rstrip()[1:-2]
		binstr = binascii.unhexlify(hexstr)
		command = ord(binstr[3])
		
		# only type 0 records are interesting
		if (command == 0):
			address = (ord(binstr[1]) << 8) + ord(binstr[2])
			bytes   = bytearray(binstr[4:])
			self.__insert(address, bytes)

	# insert the byte array into the ranges dictionary, merging as we go
	def __insert(self, address, bytes):
		# look for a range that immediately follows this one
		candidate = address + len(bytes)
		if (candidate in self.ranges):
			# found one, remove from ranges and merge it
			nextbytes = self.ranges.pop(candidate)
			bytes.extend(nextbytes)

		# iterate the existing ranges looking for one that precedes this
		for candidate in self.ranges.keys():
			prevlen = len(self.ranges[candidate])
			if ((candidate + prevlen) == address):
				self.ranges[candidate].extend(bytes)
				return
		# just insert it
		self.ranges[address] = bytes

	def __init__(self, path):
		self.ranges = dict()

		# read the file
		# XXX should have some file metadata here too ...
		f = open(path, "r")
		for line in f:
			self.__parseline(line)

	def code(self):
		return self.ranges


class uploader(object):
	'''Uploads a firmware file to the SiK bootloader'''

	OK		= chr(0x10)
	FAILED		= chr(0x11)
	INSYNC		= chr(0x12)
	EOC		= chr(0x20)
	GET_SYNC	= chr(0x21)
	GET_DEVICE	= chr(0x22)
	CHIP_ERASE	= chr(0x23)
	LOAD_ADDRESS	= chr(0x24)
	PROG_FLASH	= chr(0x25)
	READ_FLASH	= chr(0x26)
	PROG_MULTI	= chr(0x27)
	READ_MULTI	= chr(0x28)
	REBOOT		= chr(0x30)
	
	PROG_MULTI_MAX	= 64
	READ_MULTI_MAX	= 255

	def __init__(self, portname):
		self.port = serial.Serial(portname, 115200, timeout=5)

	def __getSync(self):
		c = self.port.read()
		if (c != self.INSYNC):
			raise RuntimeError("unexpected 0x%x instead of INSYNC" % ord(c))
		c = self.port.read()
		if (c != self.OK):
			raise RuntimeError("unexpected 0x%x instead of OK" % ord(c))

	# attempt to get back into sync with the bootloader
	def __sync(self):
		self.port.flushInput()
		self.port.write(uploader.GET_SYNC 
				+ uploader.EOC)
		self.__getSync()

	# send the CHIP_ERASE command and wait for the bootloader to become ready
	def __erase(self):
		self.port.write(uploader.CHIP_ERASE 
				+ uploader.EOC)
		self.__getSync()

	# send a LOAD_ADDRESS command
	def __set_address(self, address):
		self.port.write(uploader.LOAD_ADDRESS
				+ chr(address & 0xff)
				+ chr(address >> 8)
				+ uploader.EOC)
		self.__getSync()

	# send a PROG_FLASH command to program one byte
	def __program(self, data):
		self.port.write(uploader.PROG_FLASH
				+ chr(data)
				+ uploader.EOC)
		self.__getSync()

	# send a PROG_MULTI command to write a collection of bytes
	def __program_multi(self, data):
		self.port.write(uploader.PROG_MULTI
				+ chr(len(data)))
		self.port.write(data)
		self.port.write(uploader.EOC)
		self.__getSync()
		
	# verify a byte in flash
	def __verify(self, data):
		self.port.write(uploader.READ_FLASH
				+ uploader.EOC)
		if (self.port.read() != chr(data)):
			return False
		self.__getSync()
		return True
		
	# verify multiple bytes in flash
	def __verify_multi(self, data):
		self.port.write(uploader.READ_MULTI
				+ chr(len(data)))
		for i in data:
			if (self.port.read() != chr(i)):
				return False
		self.__getSync()
		return True
		
	# send the reboot command
	def __reboot(self):
		self.port.write(uploader.REBOOT)

	# split a sequence into a list of size-constrained pieces
	def __split_len(seq, length):
    		return [seq[i:i+length] for i in range(0, len(seq), length)]

	# upload code
	def __program(self, code):
		for address in sorted(code.keys()):
			__set_address(address)
			groups = __split_len(code[address], uploader.PROG_MULTI_MAX)
			for bytes in groups:
				__program_multi(bytes)

	# verify code
	def __verify(self, code):
		for address in sorted(code.keys()):
			__set_address(address)
			groups = __split_len(code[address], uploader.READ_MULTI_MAX)
			for bytes in groups:
				if (not __verify_multi(bytes)):
					raise RuntimeError("Verification failed in group at 0x%x" % address)

	# verify whether the bootloader is present and responding
	def check(self):
		self.__sync()

	def identify(self):
		self.port.write(uploader.GET_DEVICE
				+ uploader.EOC)
		board_id = ord(self.port.read()[0])
		board_freq = ord(self.port.read()[0])
		return board_id, board_freq

	def upload(self, code):
		__erase()
		__program(code)
		__verify(code)
		__reboot()
	

# Parse commandline arguments
parser = argparse.ArgumentParser(description="Firmware uploader for the SiK radio system.")
parser.add_argument('--port', action="store", required=True, help="Serial port to which the SiK radio is attached.")
parser.add_argument('firmware', action="store", help="Firmware file to be uploaded")
args = parser.parse_args()

# Load the firmware file
fw = firmware(args.firmware)

# Connect to the device and identify it
up = uploader(args.port)
up.check()
id, freq = up.identify()

print("board %x  freq %x" % (id, freq))

