using System;
using System.Configuration;
using System.Configuration.Install;
using System.IO;
using System.IO.Ports;
using Gtk;
using uploader;

public delegate void MonitorEventHandler (string portname);

public delegate void UploadEventHandler (string portname,string filename);

public partial class MainWindow: Gtk.Window
{	
	string								config_name = "SiKUploader";
	System.Configuration.Configuration	config;
	ConfigSection 						config_section;

	public event UploadEventHandler		UploadEvent;
	public event MonitorEventHandler	MonitorEvent;
	public event LogEventHandler		LogEvent;
	
	public MainWindow (): base (Gtk.WindowType.Toplevel)
	{
		// construct the UI
		Build ();
		
		// Get the current configuration file.
		config = ConfigurationManager.OpenExeConfiguration (ConfigurationUserLevel.None);
		
		// Look for our settings and add/create them if missing
		//
		if (config.Sections [config_name] == null) {
			config_section = new ConfigSection ();
			config.Sections.Add (config_name, config_section);
			config_section.SectionInformation.ForceSave = true;
			config.Save (ConfigurationSaveMode.Full);
		}
		config_section = config.GetSection (config_name) as ConfigSection;
		
		// wire up the Upload button
		button_Upload.Clicked += new EventHandler (do_upload);
		button_Upload.Sensitive = false;		
		
		// wire up the Console button
		button_Console.Clicked += new EventHandler (do_console);
		button_Console.Sensitive = true;
		
		// configure the file chooser
		FileFilter filter = new FileFilter ();
		filter.Name = "IntelHex files (*.hex)";
		filter.AddPattern ("*.hex");
		chooser_Hex.AddFilter (filter);
		filter = new FileFilter ();
		filter.Name = "All Files (*.*)";
		filter.AddPattern ("*.*");
		chooser_Hex.AddFilter (filter);
		chooser_Hex.FileSet += new EventHandler (file_selected);
		if (File.Exists (config_section.lastPath))
			chooser_Hex.SetFilename (config_section.lastPath);
		
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
		combo_Port.Model.Foreach (default_port_compare);
		
		// Set up the status bar
		status_Bar.Push (1, "Init...");		
	}
	
	private bool default_port_compare (TreeModel model, TreePath path, TreeIter iter)
	{
		string port = model.GetValue (iter, 0) as string;
		
		if (port != null) {
			Console.WriteLine ("considering " + port + " while looking for " + config_section.lastPort);
			if (port.Equals (config_section.lastPort)) {
				combo_Port.SetActiveIter (iter);
				return true;
			}
		}
		return false;
	}
	
	protected void OnDeleteEvent (object sender, DeleteEventArgs a)
	{
		config.Save (ConfigurationSaveMode.Modified);
		
		Application.Quit ();
		a.RetVal = true;
	}
	
	private void file_selected (object sender, EventArgs args)
	{
		string filename = chooser_Hex.Filename;
		
		if (File.Exists (filename))
			button_Upload.Sensitive = true;
	}
	
	private void flush ()
	{
		while (Gtk.Application.EventsPending ())
			Gtk.Application.RunIteration ();
	}
	
	private void do_console (object obj, EventArgs args)
	{
		if (MonitorEvent != null) {
			string p = get_port_name ();
			
			MonitorEvent (p);
		}
	}
	
	private void do_upload (object obj, EventArgs args)
	{	
		if (UploadEvent != null) {
			string p = get_port_name ();
			string f = get_file_name ();
		
			UploadEvent (p, f);
		}
	}
	
	public string get_port_name ()
	{
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
	
	public string get_file_name ()
	{
		string s = chooser_Hex.Filename;
		
		if (s == null) {
			log ("Please select an IntelHex (*.hex) file to flash.");
			throw new Exception ("no file selected to upload");
		}
		return s;
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
		progress_Bar.Fraction = completed;
		flush ();
	}
}

public sealed class ConfigSection : ConfigurationSection
{
	public ConfigSection ()
	{
	}

	[ConfigurationProperty("lastPath", 
		DefaultValue = ""
		)]
	public string lastPath {
		get {
			Console.WriteLine ("fetching lastPath : " + (string)this ["lastPath"]);
			return (string)this ["lastPath"];
		}
		set {
			Console.WriteLine ("setting lastPath : " + value);
			this ["lastPath"] = value;
		}
	}
	
	[ConfigurationProperty("lastPort",
		DefaultValue = ""
		)]
	public string lastPort {
		get {
			return (string)this ["lastPort"];
		}
		set {
			this ["lastPort"] = value;
		}		
	}
}
