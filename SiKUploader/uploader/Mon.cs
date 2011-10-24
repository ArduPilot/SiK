// 
//  Author:
//    Michael Smith msmith@purgatory.org
// 
//  Copyright (c) 2011, Michael Smith
// 
//  All rights reserved.
// 
//  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// 
//     * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in
//       the documentation and/or other materials provided with the distribution.
//     * Neither the name of the [ORGANIZATION] nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
// 
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
//  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
//  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
//  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
//  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
//  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
using Gtk;
using Gdk;
using GLib;
using Pango;
using System;
using System.IO.Ports;

namespace uploader
{
	public partial class Mon : Gtk.Window
	{
		public event LogEventHandler LogEvent;
		public event QuitEventHandler QuitEvent;
		
		private SerialPort	port;
		private bool		is_deleted;
		
		/// <summary>
		/// Maximum number of scrollback lines in the monitor window.
		/// </summary>
		private const int	max_lines = 1000;

		public Mon (SerialPort on_port) : 
				base(Gtk.WindowType.Toplevel)
		{
			this.Build ();
			
			// pick a font that works
			text_Monitor.ModifyFont (FontDescription.FromString ("Courier 12"));
			
			// add the port data poll timeout
			GLib.Timeout.Add (100, check_for_data);
			
			// default the statusbar
			status_Monitor.Push (1, "idle");
		}
		
		private bool check_for_data ()
		{
			try {
				if ((port != null) &&
					port.IsOpen && 
					(port.BytesToRead > 0))
					addchars (port.ReadExisting ());
			} catch {
				// harmless if this fails
			}
			if (is_deleted)
				return false;
			return true;
		}
		
		public void connect (SerialPort to_port)
		{
			port = to_port;
			status_Monitor.Pop (1);
			status_Monitor.Push (1, port.PortName);
		}
		
		public void disconnect ()
		{
			port = null;
			status_Monitor.Pop (1);
			status_Monitor.Push (1, "not connected");
		}
		
		/// <summary>
		/// Adds the string s to the textview at the cursor.
		/// </summary>
		/// <param name='s'>
		/// String to add.
		/// </param>
		private void addchars (string s)
		{
			
			// Add the text
			text_Monitor.Buffer.InsertAtCursor (s);
			
			// If there is too much text, remove some
			int linecount = text_Monitor.Buffer.LineCount;
			if (linecount > max_lines) {
				Gtk.TextIter startpoint = text_Monitor.Buffer.StartIter;
				Gtk.TextIter endpoint = text_Monitor.Buffer.GetIterAtLine (linecount - max_lines);
				text_Monitor.Buffer.Delete (ref startpoint, ref endpoint);
			}
			
			// And if we are autoscrolling, do the right thing and keep the cursor visible
			if (check_Autoscroll.Active)
				text_Monitor.ScrollMarkOnscreen (text_Monitor.Buffer.InsertMark);
		}

		/// <summary>
		/// Log the specified message with the given level.
		/// </summary>
		/// <param name='message'>
		/// Message to log
		/// </param>
		/// <param name='level'>
		/// Level at which to log.
		/// </param>
		private void log (string message, int level = 0)
		{
			if (LogEvent != null)
				LogEvent (message, level);
		}

		protected void clear_pressed (object sender, System.EventArgs e)
		{
			text_Monitor.Buffer.Clear ();
		}

		protected void delete_event (object o, Gtk.DeleteEventArgs args)
		{
			// stop the timer the next time it runs
			is_deleted = true;
			
			// close the port if it's open
			disconnect ();
			
			if (QuitEvent != null)
				QuitEvent ();
		}
		
		[GLib.ConnectBefore]
		protected void keypressed (object o, Gtk.KeyPressEventArgs args)
		{
			uint key = args.Event.KeyValue;
			byte[] sendbytes = new byte[1];
			
			switch (args.Event.Key) {
			case Gdk.Key.Return:
			case Gdk.Key.KP_Enter:
				key = 13;
				break;
			case Gdk.Key.BackSpace:
			case Gdk.Key.Delete:
				key = 8;
				break;
			default:
				// don't handle anything that's not 7-bit ascii
				if (key > 128) {
					log (string.Format ("ignoring {0}\n", key), 2);
					return;
				}
				break;
			}
			
			sendbytes [0] = (byte)key;
			port.Write (sendbytes, 0, 1);
			log (string.Format ("sending {0}\n", key), 2);
			
			// we have handled the event - don't pass it any further
			args.RetVal = true;
		}
	}
}