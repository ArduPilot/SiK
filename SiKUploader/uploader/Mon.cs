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
using GLib;
using Pango;
using System;
using System.IO.Ports;

namespace uploader
{
	public partial class Mon : Gtk.Window
	{
		public event LogEventHandler LogEvent;
		
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
			
			// get a callback when we are being deleted
			DeleteEvent += deleted;
			
			// hook up the Clear button
			button_Clear.Clicked += clear_text;
			
			// default the statusbar
			status_Monitor.Push (1, "idle");
		}
		
		protected void deleted (object sender, DeleteEventArgs args)
		{
			// stop the timer the next time it runs
			is_deleted = true;
			
			// close the port if it's open
			disconnect ();
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
		
		private void clear_text (object sender, EventArgs args)
		{
			text_Monitor.Buffer.Clear ();
		}
		
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

		private void log (string message, int level = 0)
		{
			if (LogEvent != null)
				LogEvent (message, level);
		}
	}
	
}