using System;
using System.IO.Ports;
using System.Collections.Generic;

namespace uploader
{
	public class Uploader : SerialPort
	{
		Progress	progress;
		Log			log;
		
		private enum Code : byte
		{
			OK				= 0x10,
			FAILED			= 0x11,
			INSYNC			= 0x12,
			
			EOC				= 0x20,
			GET_SYNC		= 0x21,
			GET_DEVICE		= 0x22,
			CHIP_ERASE		= 0x23,
			LOAD_ADDRESS	= 0x24,
			PROG_FLASH		= 0x25,
			READ_FLASH		= 0x26,
			
			DEVICE_ID		= 0x4d,	// XXX should come with the firmware image...
		};
		
		public Uploader (string port_name, Progress progress_delegate, Log log_delegate)
		{
			progress = progress_delegate;
			log = log_delegate;
			init (port_name);
		}

		public Uploader (string port_name, Progress progress_delegate)
		{
			progress = progress_delegate;
			log = no_log;
			init (port_name);
		}
		
		/// <summary>
		/// Initializes a new instance of the <see cref="uploader.Uploader"/> class.
		/// </summary>
		/// <param name='port_name'>
		/// Serial port name.
		/// </param>
		/// <param name='progress_delegate'>
		/// Progress indicator.
		/// </param>
		/// <exception cref='NoSync'>
		/// Is thrown if we cannot establish sync with the bootloader.
		/// </exception>
		private void init (string port_name)
		{

			// configure the port
			PortName = port_name;
			BaudRate = 115200;
			DataBits = 8;
			StopBits = StopBits.One;
			Parity = Parity.None;
			ReadTimeout = 2000;			// must be longer than full flash erase time (~1s)
			
			Open ();
			log (string.Format ("opened {0}\n", port_name));
			
			// synchronise with the bootloader
			//
			// The second sync attempt here is mostly laziness, though it does verify that we 
			// can send more than one packet.
			//
			for (int i = 0; i < 3; i++) {
				if (cmdSync ())
					break;
				log (string.Format ("sync({0}) failed\n", i), 1);
			}
			if (!cmdSync ())
				throw new Exception ("could not synchronise with the bootloader");
			checkDevice ((byte)Code.DEVICE_ID);
			log ("connected to bootloader\n");
		}
		
		/// <summary>
		/// Upload the specified image_data.
		/// </summary>
		/// <param name='image_data'>
		/// Image_data to be uploaded.
		/// </param>
		public void upload (IHex image_data)
		{
			progress (0);
			
			// erase the program area first
			log ("erasing program flash\n");
			cmdErase ();
			
			// progress fractions
			int bytes_to_process = 0;
			foreach (byte[] bytes in image_data.Values) {
				bytes_to_process += bytes.Length;
			}
			bytes_to_process *= 2;		// once to program, once to verify
			int bytes_processed = 0;
			
			// program the flash blocks
			log ("programming\n");
			foreach (KeyValuePair<UInt16, byte[]> kvp in image_data) {
				// move the program pointer to the base of this block
				cmdSetAddress (kvp.Key);
				log (string.Format ("prog 0x{0:X}/{1}\n", kvp.Key, kvp.Value.Length), 1);
				
				foreach (byte b in kvp.Value) {
					cmdProgram (b);
					progress ((double)(++bytes_processed) / bytes_to_process);
				}
			}
			
			// and read them back to verify that they were programmed
			log ("verifying\n");
			foreach (KeyValuePair<UInt16, byte[]> kvp in image_data) {
				// move the program pointer to the base of this block
				cmdSetAddress (kvp.Key);
				log (string.Format ("prog 0x{0:X}/{1}\n", kvp.Key, kvp.Value.Length), 1);
				
				foreach (byte b in kvp.Value) {
					cmdVerify (b);
					progress ((double)(++bytes_processed) / bytes_to_process);
				}
			}
		}
		
		/// <summary>
		/// Requests a sync reply.
		/// </summary>
		/// <returns>
		/// True if in sync, false otherwise.
		/// </returns>
		private bool cmdSync ()
		{
			DiscardInBuffer ();
			
			send (Code.GET_SYNC);
			send (Code.EOC);
			
			try {
				getSync ();
			} catch {
				return false;
			}
			
			return true;
		}

		/// <summary>
		/// Erases the device.
		/// </summary>
		private void cmdErase ()
		{
			send (Code.CHIP_ERASE);
			send (Code.EOC);
			
			getSync ();
		}
		
		/// <summary>
		/// Set the address for the next program or read operation.
		/// </summary>
		/// <param name='address'>
		/// Address to be set.
		/// </param>
		private void cmdSetAddress (UInt16 address)
		{
			send (Code.LOAD_ADDRESS);
			send (address);
			send (Code.EOC);
			
			getSync ();
		}	
		
		/// <summary>
		/// Programs a byte and advances the program address by one.
		/// </summary>
		/// <param name='data'>
		/// Data to program.
		/// </param>
		private void cmdProgram (byte data)
		{
			send (Code.PROG_FLASH);
			send (data);
			send (Code.EOC);
			
			getSync ();
		}
		
		/// <summary>
		/// Verifies the byte at the current program address.
		/// </summary>
		/// <param name='data'>
		/// Data expected to be found.
		/// </param>
		/// <exception cref='VerifyFail'>
		/// Is thrown when the verify fail.
		/// </exception>
		private void cmdVerify (byte data)
		{
			send (Code.READ_FLASH);
			send (Code.EOC);
			
			if (recv () != data)
				throw new Exception ("flash verification failed");
			
			getSync ();
		}
		
		private void checkDevice (byte device_id)
		{
			send (Code.GET_DEVICE);
			send (Code.EOC);
			
			if (recv () != device_id)
				throw new Exception ("bootloader device ID mismatch");
			
			getSync ();
		}
		
		/// <summary>
		/// Expect the two-byte synchronisation codes within the read timeout.
		/// </summary>
		/// <exception cref='NoSync'>
		/// Is thrown if the wrong bytes are read.
		/// <exception cref='TimeoutException'>
		/// Is thrown if the read timeout expires.
		/// </exception>
		private void getSync ()
		{
			try {
				Code c;
				
				c = (Code)recv ();
				if (c != Code.INSYNC) {
					log (string.Format ("got {0:X} when expecting {1:X}\n", (int)c, (int)Code.INSYNC), 2);
					throw new Exception ("bad sync byte from bootloader");
				}
				c = (Code)recv ();
				if (c != Code.OK) {
					log (string.Format ("got {0:X} when expecting {1:X}\n", (int)c, (int)Code.EOC), 2);
					throw new Exception ("bad status byte from bootloader");
				}
			} catch (TimeoutException) {
				log ("timeout waiting for sync");
				throw new Exception ("timeout waiting for sync from bootloader");
			}
			log ("in sync\n", 2);
		}
		
		/// <summary>
		/// Send the specified code to the bootloader.
		/// </summary>
		/// <param name='code'>
		/// Code to send.
		/// </param>
		private void send (Code code)
		{
			byte[] b = new byte[] { (byte)code };
			
			log ("send ", 2);
			foreach (byte x in b) {
				log (string.Format (" {0:X}", x), 2);
			}
			log ("\n", 2);
			
			Write (b, 0, 1);
		}
		
		/// <summary>
		/// Send the specified byte to the bootloader.
		/// </summary>
		/// <param name='data'>
		/// Data byte to send.
		/// </param>
		private void send (byte data)
		{
			byte[] b = new byte[] { data };
			
			log ("send ", 2);
			foreach (byte x in b) {
				log (string.Format (" {0:X}", x), 2);
			}
			log ("\n", 2);

			Write (b, 0, 1);
		}
		
		/// <summary>
		/// Send the specified 16-bit value, LSB first.
		/// </summary>
		/// <param name='data'>
		/// Data value to send.
		/// </param>
		private void send (UInt16 data)
		{
			byte[] b = new byte[2] { (byte)(data & 0xff), (byte)(data >> 8) };
			
			log ("send ", 2);
			foreach (byte x in b) {
				log (string.Format (" {0:X}", x), 2);
			}
			log ("\n", 2);

			Write (b, 0, 2);
		}
		
		/// <summary>
		/// Receive a byte.
		/// </summary>
		private byte recv ()
		{
			byte b;
			
			b = (byte)ReadByte ();
			
			log (string.Format ("recv {0:X}\n", b), 2);
			
			return b;
		}
		
		private void no_log (string message, int level = 0)
		{
		}
	}
}

