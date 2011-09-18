using System;
using System.Collections.Generic;
using System.IO;

namespace uploader
{
	public class IHex : SortedList<UInt16, byte[]>
	{
		Log		log;
		
		private void no_log (string message, int level = 0)
		{
		}
		
		public IHex (string fromPath)
		{
			log = no_log;
			parse (fromPath);
		}

		public IHex (string fromPath, Log logger)
		{
			log = logger;
			parse (fromPath);
		}
		
		private void parse (string fromPath)
		{
			StreamReader sr = new StreamReader (fromPath);
			
			log (string.Format ("ihex: reading from {0}\n", fromPath));
			
			while (!sr.EndOfStream) {
				string line = sr.ReadLine ();
				
				// every line must start with a :
				if (!line.StartsWith (":"))
					throw new Exception ("invalid IntelHex file");
				
				// parse the record type and data length, assume ihex8
				// ignore the checksum
				byte length = Convert.ToByte (line.Substring (1, 2), 16);
				UInt16 address = Convert.ToUInt16 (line.Substring (3, 4), 16);
				byte rtype = Convert.ToByte (line.Substring (7, 2), 16);
				
				// handle type zero (data) records
				if (rtype == 0) {
					byte[] b = new byte[length];
					string hexbytes = line.Substring (9, length * 2);
					
					// convert hex bytes
					for (int i = 0; i < length; i++) {
						b [i] = Convert.ToByte (hexbytes.Substring (i * 2, 2), 16);
					}
					
					// and add to the list of ranges
					Add (address, b);
					
					log (string.Format ("ihex: 0x{0:X}: {1}\n", address, length), 1);
				}
			}
			if (Count < 1)
				throw new Exception ("no data in IntelHex file");
		}		
	}
}

