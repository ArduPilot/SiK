using Gtk;
using System;
using System.IO;
using System.IO.Ports;

namespace uploader
{
	public partial class MainWindow: Gtk.Window
	{	
		public event UploadEventHandler		UploadEvent;
		public event MonitorEventHandler	MonitorEvent;
		public event LogEventHandler		LogEvent;
		public event QuitEventHandler		QuitEvent;
	
		private string default_port_name;
	
		public MainWindow (): base (Gtk.WindowType.Toplevel)
		{
			// construct the UI
			Build ();
		
			// wire up the Upload button
			button_Upload.Clicked += new EventHandler (do_upload);
		
			// wire up the Console button
			button_Console.Clicked += new EventHandler (do_console);
		
			// configure the file chooser
			FileFilter filter = new FileFilter ();
			filter.Name = "IntelHex files (*.hex)";
			filter.AddPattern ("*.hex");
			chooser_Hex.AddFilter (filter);
			filter = new FileFilter ();
			filter.Name = "All Files (*.*)";
			filter.AddPattern ("*.*");
			chooser_Hex.AddFilter (filter);
			DeleteEvent += delete;
		
			// get serial port names and populate the combo box
			foreach (string port in SerialPort.GetPortNames ()) {
				if (port.StartsWith ("/dev/tty") && 
					(!port.StartsWith ("/dev/tty.") || port.StartsWith ("/dev/tty.Bluetooth")))
					continue;				// ignore this, it's a pty or a Mac Bluetooth interface
			
				combo_Port.AppendText (port);
			}
		
			// start by defaulting to the first element in the combo box
			TreeIter iter;
			if (combo_Port.Model.GetIterFirst (out iter))
				combo_Port.SetActiveIter (iter);
		
			// Set up the status bar
			status_Bar.Push (1, "Init...");		
		}

		/// <summary>
		/// Handles the window DeleteEvent by instigating the termination of the application.
		/// </summary>
		/// <param name='sender'>
		/// Sender.
		/// </param>
		/// <param name='a'>
		/// A.
		/// </param>
		private void delete (object sender, DeleteEventArgs a)
		{
			Application.Quit ();
			a.RetVal = true;
			
			if (QuitEvent != null)
				QuitEvent ();
		}
		
		/// <summary>
		/// Flush any updates to the window to the display.
		/// </summary>
		private void flush ()
		{
			while (Gtk.Application.EventsPending ())
				Gtk.Application.RunIteration ();
		}
	
		/// <summary>
		/// Handle the button click event for the Console button.
		/// </summary>
		/// <param name='obj'>
		/// Object.
		/// </param>
		/// <param name='args'>
		/// Arguments.
		/// </param>
		private void do_console (object obj, EventArgs args)
		{
			if (MonitorEvent != null) {
				MonitorEvent (PortName);
			}
		}
	
		/// <summary>
		/// Handle the button click for the Upload button.
		/// </summary>
		/// <param name='obj'>
		/// Object.
		/// </param>
		/// <param name='args'>
		/// Arguments.
		/// </param>
		private void do_upload (object obj, EventArgs args)
		{	
			if (UploadEvent != null) {		
				UploadEvent (PortName, FileName);
			}
		}
	
		public string PortName {
			get {
				TreeIter iter;
				string port = "";
				if (combo_Port.GetActiveIter (out iter))
					port = (string)combo_Port.Model.GetValue (iter, 0);
				if (port.Equals ("")) {
					log ("Please select the serial port connected to your radio.");
					throw new Exception ("no port selected");
				}
				return port;
			}
			set {
				default_port_name = value;
				combo_Port.Model.Foreach (default_port_compare);
			}
		}
	
		private bool default_port_compare (TreeModel model, TreePath path, TreeIter iter)
		{
			string port = model.GetValue (iter, 0) as string;
		
			if (port != null) {
				Console.WriteLine ("considering " + port + " while looking for " + default_port_name);
				if (port.Equals (default_port_name)) {
					combo_Port.SetActiveIter (iter);
					return true;
				}
			}
			return false;
		}

		public string FileName {
			get {
				string s = chooser_Hex.Filename;
		
				if (s == null) {
					log ("Please select an IntelHex (*.hex) file to flash.");
					throw new Exception ("no file selected to upload");
				}
				return s;
			}
			set {
				if (File.Exists (value))
					chooser_Hex.SetFilename (value);
			}
		}
	
		private void log (string message, int level = 0)
		{
			if (LogEvent != null)
				LogEvent (message, 0);
		}
	
		public void set_status (string msg)
		{
			status_Bar.Pop (1);
			status_Bar.Push (1, msg);
			flush ();
		}
	
		public void set_progress (double completed)
		{
			// silence an otherwise stupid Gtk warning in case we overflow slightly
			if (completed > 1.0)
				completed = 1.0;
			progress_Bar.Fraction = completed;
			flush ();
		}
	}
}