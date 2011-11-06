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

	NOP		= chr(0x00)
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
	
	PROG_MULTI_MAX	= 32 # 64 causes serial hangs with some USB-serial adapters
	READ_MULTI_MAX	= 255

	def __init__(self, portname):
		self.port = serial.Serial(portname, 115200, timeout=5)

	def __send(self, c):
		#print("send " + binascii.hexlify(c))
		self.port.write(str(c))

	def __recv(self):
		c = self.port.read()
		if (len(c) < 1):
			raise RuntimeError("timeout waiting for data")
		#print("recv " + binascii.hexlify(c))
		return c

	def __getSync(self):
		c = self.__recv()
		if (c != self.INSYNC):
			raise RuntimeError("unexpected 0x%x instead of INSYNC" % ord(c))
		c = self.__recv()
		if (c != self.OK):
			raise RuntimeError("unexpected 0x%x instead of OK" % ord(c))

	# attempt to get back into sync with the bootloader
	def __sync(self):
		# send a stream of ignored bytes longer than the longest possible conversation
		# that we might still have in progress
		self.__send(uploader.NOP * (uploader.PROG_MULTI_MAX + 2))
		self.port.flushInput()
		self.__send(uploader.GET_SYNC 
				+ uploader.EOC)
		self.__getSync()

	# send the CHIP_ERASE command and wait for the bootloader to become ready
	def __erase(self):
		self.__send(uploader.CHIP_ERASE 
				+ uploader.EOC)
		self.__getSync()

	# send a LOAD_ADDRESS command
	def __set_address(self, address):
		self.__send(uploader.LOAD_ADDRESS
				+ chr(address & 0xff)
				+ chr(address >> 8)
				+ uploader.EOC)
		self.__getSync()

	# send a PROG_FLASH command to program one byte
	def __program(self, data):
		self.__send(uploader.PROG_FLASH
				+ chr(data)
				+ uploader.EOC)
		self.__getSync()

	# send a PROG_MULTI command to write a collection of bytes
	def __program_multi(self, data):
		self.__send(uploader.PROG_MULTI
				+ chr(len(data)))
		self.__send(data)
		self.__send(uploader.EOC)
		self.__getSync()
		
	# verify a byte in flash
	def __verify(self, data):
		self.__send(uploader.READ_FLASH
				+ uploader.EOC)
		if (self.__recv() != chr(data)):
			return False
		self.__getSync()
		return True
		
	# verify multiple bytes in flash
	def __verify_multi(self, data):
		self.__send(uploader.READ_MULTI
				+ chr(len(data))
				+ uploader.EOC)
		for i in data:
			if (self.__recv() != chr(i)):
				return False
		self.__getSync()
		return True
		
	# send the reboot command
	def __reboot(self):
		self.__send(uploader.REBOOT)

	# split a sequence into a list of size-constrained pieces
	def __split_len(self, seq, length):
    		return [seq[i:i+length] for i in range(0, len(seq), length)]

	# upload code
	def __program(self, fw):
		code = fw.code()
		for address in sorted(code.keys()):
			self.__set_address(address)
			groups = self.__split_len(code[address], uploader.PROG_MULTI_MAX)
			for bytes in groups:
				self.__program_multi(bytes)

	# verify code
	def __verify(self, fw):
		code = fw.code()
		for address in sorted(code.keys()):
			self.__set_address(address)
			groups = self.__split_len(code[address], uploader.READ_MULTI_MAX)
			for bytes in groups:
				if (not self.__verify_multi(bytes)):
					raise RuntimeError("Verification failed in group at 0x%x" % address)

	# verify whether the bootloader is present and responding
	def check(self):
		self.__sync()

	def identify(self):
		self.__send(uploader.GET_DEVICE
				+ uploader.EOC)
		board_id = ord(self.__recv()[0])
		board_freq = ord(self.__recv()[0])
		self.__getSync()
		return board_id, board_freq

	def upload(self, fw):
		print("erase...")
		self.__erase()
		print("program...")
		self.__program(fw)
		print("verify...")
		self.__verify(fw)
		print("done.")
		self.__reboot()
	

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

# XXX here we should check the firmware board ID against the board we're connected to

up.upload(fw)