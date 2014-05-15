#!/usr/bin/env python
#
# Provide command line access to AT command set on radios
#

import serial, sys, argparse, time, fdpexpect

class ATCommandSet(object):
    ''' Interface to the AT command set '''
    
    ### AT Command Constants ###
    
    # Prefix determines if commanding attached or linked radio
    AT_LOCAL_PREFIX	= 'AT'
    AT_REMOTE_PREFIX	= 'RT'
    
    # AT commands that are implemented in this class
    AT_SHOW_BRD_TYPE	= 'I2'
    AT_SHOW_BRD_FREQ	= 'I3'
    AT_SHOW_BRD_VER	= 'I4'
    AT_SHOW_PARAM	= 'I5'
    AT_EXIT		= 'O'
    AT_PARAM		= 'S'
    AT_REBOOT		= 'Z'
    AT_PARAM_WRITE	= '&W'
    # AT commands yet to be implemented here
    AT_SHOW_VER_LONG	= 'I0'
    AT_SHOW_VER		= 'I1'
    AT_SHOW_TDM		= 'I6'
    AT_SHOW_RSSI	= 'I7'
    AT_PARAM_FACTORY	= '&F'
    AT_DEBUG_RSSI	= '&T=RSSI'
    AT_DEBUG_TDM	= '&T=TDM'
    AT_DEBUG_OFF	= '&T'
    
    # Parameters are gotten with AT_PARAM + PARAM_* + '?"
    # Parameters are set with AT_PARAM + PARAM_* + '=' + value
    PARAM_FORMAT	= 0
    PARAM_SERIAL_SPEED	= 1
    PARAM_AIR_SPEED	= 2
    PARAM_NETID		= 3
    PARAM_TXPOWER	= 4
    PARAM_ECC		= 5
    PARAM_MAVLINK	= 6
    PARAM_OPPRESEND	= 7
    PARAM_MIN_FREQ	= 8
    PARAM_MAX_FREQ	= 9
    PARAM_NUM_CHANNELS	= 10
    PARAM_DUTY_CYCLE	= 11
    PARAM_LBT_RSSI	= 12
    PARAM_MANCHESTER	= 13
    PARAM_RTSCTS	= 14
    
    ### Internals ###
    
    # Create object and immediate attempt to connect to radio
    def __init__(self, device, baudrate=57600, debug=False,
                 dsrdtr=False, rtscts=False, xonxoff=False):
        # Initialize object data members
        self.is_command = False		# Track if we've entered command mode
        self.is_remote = False		# Track if operating on remote radio
        self.read_timeout = 5		# Max time to wait for data
        
        logfile=None
        if debug:
            logfile=sys.stdout
        
        # Initialize the serial connection
        # Note: we pass the buck on raised exceptions
        self.port = serial.Serial(device, baudrate=baudrate, timeout=0, 
                                  dsrdtr=dsrdtr, rtscts=rtscts, xonxoff=xonxoff)
        self.ser = fdpexpect.fdspawn(self.port.fileno(), logfile=logfile)
    
    # Send raw text to radio
    def __send(self, text):
        if (self.port is None) or (self.ser is None):
            return False
        
        try:
            res = self.ser.send(text)
            time.sleep(0.2)  # Let the serial line catch up
            return res
        except:
            return False
    
    # Form an AT command and send to radio
    def __send_at(self, command):
        # Don't send bytes if in "normal" mode, other radio is listening!
        if not self.is_command:
            return False
        
        prefix = ATCommandSet.AT_LOCAL_PREFIX
        if self.is_remote and (command != ATCommandSet.AT_EXIT):
            prefix = ATCommandSet.AT_REMOTE_PREFIX
        text = '\r\n' + prefix + str(command) + '\r\n'
        return not not self.__send(text)
    
    # Look for 'pattern' (string RE) and return MatchObject if seen before read_timeout
    def __expect(self, pattern_list):
        if (self.port is None) or (self.ser is None):
            return False
        
        try:
            self.ser.expect(pattern_list, timeout=self.read_timeout)
            res = self.ser.match
            time.sleep(0.2)  # Let the serial line catch up
            return res
        except:
            return False
    
    # Send AT command, then look for pattern
    def __query(self, command, pattern):
        if not self.__send_at(command):
            return False
        val = self.__expect(pattern)
        return val
    
    # Query for an int
    def __query_int(self, command):
        val = self.__query(command, ['(\d+)\r\n'])
        if val:
            return int(val.group(0))
        return False
    
    # Query for a float
    def __query_float(self, command):
        val = self.__query(command, ['(\d+\.\d+)\r\n'])
        if val:
            return float(val.group(0))
        return False
    
    # Query for literal text (return True if found)
    def __query_exact(self, command, text):
        return not not self.__query(command, ['%s\r\n' % text])
    
    
    ### Manage command mode ###
    
    def enter_command_mode(self):
        # Technically okay to resend this command, but won't see an 'OK' back
        if self.is_command:
            return False
        
        # Will raise a timeout exception if already in command mode
        # (due to another process leaving it that way?)
        time.sleep(1)
        if not self.__send('+++'):
            return False
        time.sleep(1)
        if not self.__expect(['OK']):
            return False
        
        self.is_command = True
        return True
    
    def leave_command_mode(self):
        # Don't send bytes if in "normal" mode, other radio is listening!
        self.__send_at(ATCommandSet.AT_EXIT)
        self.is_command = False
    
    def leave_command_mode_force(self):
        # Overrides mode check, use only if radio is "stuck" in command mode
        self.is_command = True
        self.leave_command_mode()
    
    def is_command_mode(self):
        return self.is_command
    
    ### Select local or remote operation ###
    
    def set_remote_mode(self, remote=False):
        # True = remote (linked) radio, False = local (attached) radio
        self.is_remote = remote
    
    def is_remote_mode(self):
        return self.is_remote
    
    ### Get general info ###
    
    def get_radio_version(self):
        return self.__query_float(ATCommandSet.AT_SHOW_VER)
    
    def get_board_type(self):
        return self.__query_int(ATCommandSet.AT_SHOW_BRD_TYPE)
    
    def get_board_frequency(self):
        return self.__query_int(ATCommandSet.AT_SHOW_BRD_FREQ)
    
    def get_board_version(self):
        return self.__query_int(ATCommandSet.AT_SHOW_BRD_VER)
    
    # Return a multi-line string containing all parameters, for display
    def get_params_text(self):
        res = self.__query(ATCommandSet.AT_SHOW_PARAM, ['(S0:.*S14:.*)\r\n'])
        if res:
            return res.group(0)
        else:
            return "** Could not access parameters **"
    
    ### Parameters (settings) access ###
    
    def get_param(self, p_id):
        # Assumes all params are ints
        return self.__query_int(ATCommandSet.AT_PARAM + str(p_id) + '?')
    
    def set_param(self, p_id, p_val):
        return self.__query_exact(ATCommandSet.AT_PARAM + str(p_id) + '=' + str(p_val), 'OK')
    
    # Stores params to EEPROM (necessary after 1+ set_param() calls)
    def write_params(self):
        return self.__query_exact(ATCommandSet.AT_PARAM_WRITE, 'OK')
    
    ### Miscellaneous ###
    
    # Reboot the radio (necessary for settings to take effect)
    def reboot(self):
        if not self.__send_at(ATCommandSet.AT_REBOOT):
            return False
        # The local radio leaves command mode upon reboot
        if not self.is_remote:
            self.is_command = False
        return True

### User Interface ###

if __name__ == '__main__':
    param_map = { 'format' : ATCommandSet.PARAM_FORMAT, 
                  'serialspeed' : ATCommandSet.PARAM_SERIAL_SPEED, 
                  'airspeed' : ATCommandSet.PARAM_AIR_SPEED, 
                  'netid' : ATCommandSet.PARAM_NETID, 
                  'txpower' : ATCommandSet.PARAM_TXPOWER, 
                  'ecc' : ATCommandSet.PARAM_ECC, 
                  'mavlink' : ATCommandSet.PARAM_MAVLINK, 
                  'oppresend' : ATCommandSet.PARAM_OPPRESEND, 
                  'minfreq' : ATCommandSet.PARAM_MIN_FREQ, 
                  'maxfreq' : ATCommandSet.PARAM_MAX_FREQ, 
                  'channels' : ATCommandSet.PARAM_NUM_CHANNELS, 
                  'duty' : ATCommandSet.PARAM_DUTY_CYCLE, 
                  'lbtrssi' : ATCommandSet.PARAM_LBT_RSSI, 
                  'manchester' : ATCommandSet.PARAM_MANCHESTER, 
                  'rtscts' : ATCommandSet.PARAM_RTSCTS }
    
    # Grok arguments
    parser = argparse.ArgumentParser(description='Change settings on local and remote radio.',
                                     epilog="Settable parameters (can use multiple --set-*): %s" % \
                                            " ".join(sorted(param_map.keys())))
    parser.add_argument("--baudrate", type=int, default=57600, help='connect at baud rate')
    parser.add_argument("--rtscts", action='store_true', default=False, help='connect using rtscts')
    parser.add_argument("--dsrdtr", action='store_true', default=False, help='connect using dsrdtr')
    parser.add_argument("--xonxoff", action='store_true', default=False, help='connect using xonxoff')
    parser.add_argument("--force", action='store_true', default=False, help='cycle command mode first')
    parser.add_argument("--debug", action='store_true', default=False, help='intermix raw AT traffic')
    parser.add_argument("--list-local", action='store_true', default=False, 
                        help='list local parameters and exit')
    parser.add_argument("--list-remote", action='store_true', default=False, 
                        help='list remote parameters and exit')
    parser.add_argument("--set-local", nargs=2, action='append', metavar=('PARAM', 'VALUE'), 
                        help='set local parameter (will reboot radio at end)')
    parser.add_argument("--set-remote", nargs=2, action='append', metavar=('PARAM', 'VALUE'), 
                        help='set remote parameter (will reboot radio at end)')
    parser.add_argument("--set-both", nargs=2, action='append', metavar=('PARAM', 'VALUE'), 
                        help='set on BOTH radios (takes precedence)')
    parser.add_argument("device", help='locally attached radio device')
    args = parser.parse_args()
    
    # If no get/set was requested, then bail
    if not (args.list_local or args.list_remote or \
            args.set_local or args.set_remote or args.set_both):
        print "Please specify a --list-* or --set-* operation (try -h if unsure)"
        sys.exit(0)
    # Also bail if attempting to get and set (we could, but we don't)
    if (args.list_local or args.list_remote) and \
       (args.set_local or args.set_remote or args.set_both):
        print "We don't support listing and setting in the same command"
        sys.exit(0)
    
    # Parse any --set-* args and build dictionaries of parameters to set
    # Note: --set-both overrides --set-local and --set-remote. Beyond that,
    # we don't guard against the user specifying strange combinations.
    def _parse_set(params, myset):
        for pair in params:
            prm, val = pair
            if prm not in param_map:
                print "Parameter not valid: %s" % prm
                sys.exit(-1)
            try:
                myset[prm] = int(val)
            except:
                print "Param '%s' value must be an integer: %s" % (prm, val)
                sys.exit(-1)
        return myset
    local_set = {}
    remote_set = {}
    if args.set_local:
        local_set = _parse_set(args.set_local, local_set)
    if args.set_remote:
        remote_set = _parse_set(args.set_remote, remote_set)
    if args.set_both:
        local_set = _parse_set(args.set_both, local_set)
        remote_set = _parse_set(args.set_both, remote_set)
    
    # Initialize the serial connection
    at = ATCommandSet(args.device, baudrate=args.baudrate, dsrdtr=args.dsrdtr,
                      rtscts=args.rtscts, xonxoff=args.xonxoff, debug=args.debug)
    
    # In case the radio was left in command mode, we can force it out
    # (Could just not "enter" command mode, but this seems safer somehow)
    if args.force:
        print "Forcing out of command mode first..."
        at.leave_command_mode_force()
    
    # Try to enter command mode, bail if radio doesn't give expected response
    print "Entering command mode..."
    if not at.enter_command_mode():
        print "Could not enter command mode; try --force"
        sys.exit(-1)
    
    # If --list-* was requested, do that and exit (don't set any parameters)
    def _list_info():
        r_ver = at.get_radio_version()
        if not r_ver:
            print "** Could not access radio **"
        else:
            print "radio version: %g  board type: %d  board version: %d" % \
                  (r_ver,
                   at.get_board_type() or -1,
                   at.get_board_version() or -1)
            print "Parameters: \n%s" % at.get_params_text()
    if args.list_local:
        print "Querying local radio..."
        _list_info()
    if args.list_remote:
        at.set_remote_mode(True)
        print "Querying remote radio..."
        _list_info()
        at.set_remote_mode(False)
    if args.list_local or args.list_remote:
        print "Leaving command mode..."
        at.leave_command_mode()
        sys.exit(0)
    
    # If --set-* was requested, attempt to do all of them, then write and reboot
    # only the radio(s) that was/were changed
    def _set_params(myset):
        for prm in myset:
            if at.set_param(param_map[prm], myset[prm]):
                print "Set %s to %d" % (prm, myset[prm])
            else:
                print "Failed to set %s, aborting without saving changes." % prm
                return False
        if at.write_params():
            print "Wrote parameters to EEPROM."
        else:
            print "Failed to write parameters to EEPROM, aborting without saving changes."
            return False
        if at.reboot():
            print "Commanded reboot; changes should be in effect momentarily."
        else:
            print "Failed to command reboot; please manually reboot the radio."
        return True
    # Try remote radio first
    remote_failed = False
    if remote_set:
        at.set_remote_mode(True)
        if not at.get_radio_version:
            print "Could not contact remote radio, aborting without saving changes."
            remote_failed = True
        else:
            print "Changing settings on remote radio..."
            remote_failed = _set_params(remote_set)
        at.set_remote_mode(False)
    # Try local radio second (only if no remote failures)
    if local_set and not remote_failed:
        # Since we have to successfully be in command mode, don't need more checks
        print "Changing settings on local radio..."
        _set_params(local_set)
    
    # Always leave command mode when finished
    # (If we rebooted the local radio at the very end, this will be ignored)
    print "Leaving command mode..."
    at.leave_command_mode()
    sys.exit(0)

