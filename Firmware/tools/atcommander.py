#!/usr/bin/env python
#
# Defines the ATCommandSet class, which implements the AT command set for
#   radios running the SiK firmware, and provides a basic CLI for listing
#   and adjusting radio settings.
#
# Originally written by Mike Clement, 2014
#

# Modules used by the class
import serial, sys, time, fdpexpect

class ATCommandSet(object):
    ''' Interface to the AT command set '''

    ### AT Command Constants ###

    # Prefix determines if commanding attached or linked radio
    AT_LOCAL_PREFIX     = 'AT'
    AT_REMOTE_PREFIX    = 'RT'

    # AT commands that are implemented in this class
    AT_SHOW_BRD_TYPE    = 'I2'
    AT_SHOW_BRD_FREQ    = 'I3'
    AT_SHOW_BRD_VER     = 'I4'
    AT_SHOW_PARAM       = 'I5'
    AT_EXIT             = 'O'
    AT_PARAM            = 'S'
    AT_REBOOT           = 'Z'
    AT_PARAM_WRITE      = '&W'
    # AT commands yet to be implemented here
    AT_SHOW_VER_LONG    = 'I0'
    AT_SHOW_VER         = 'I1'
    AT_SHOW_TDM         = 'I6'
    AT_SHOW_RSSI        = 'I7'
    AT_PARAM_FACTORY    = '&F'
    AT_DEBUG_RSSI       = '&T=RSSI'
    AT_DEBUG_TDM        = '&T=TDM'
    AT_DEBUG_OFF        = '&T'

    # Parameters are gotten with AT_PARAM + PARAM_* + '?"
    # Parameters are set with AT_PARAM + PARAM_* + '=' + value
    PARAM_FORMAT        = 0
    PARAM_SERIAL_SPEED  = 1
    PARAM_AIR_SPEED     = 2
    PARAM_NETID         = 3
    PARAM_TXPOWER       = 4
    PARAM_ECC           = 5
    PARAM_MAVLINK       = 6
    PARAM_OPPRESEND     = 7
    PARAM_MIN_FREQ      = 8
    PARAM_MAX_FREQ      = 9
    PARAM_NUM_CHANNELS  = 10
    PARAM_DUTY_CYCLE    = 11
    PARAM_LBT_RSSI      = 12
    PARAM_MANCHESTER    = 13
    PARAM_RTSCTS        = 14
    PARAM_MAX_WINDOW    = 15

    ### Internals ###

    # Create object and immediate attempt to connect to radio
    def __init__(self, device, baudrate=57600, debug=False,
                 dsrdtr=False, rtscts=False, xonxoff=False,
                 timeout=5):
        # Initialize object data members
        self.__is_command = False       # Track if we've entered command mode
        self.__is_remote = False        # Track if operating on remote radio
        self.__read_timeout = timeout   # Max time to wait for data

        logfile=None
        if debug: logfile=sys.stdout

        # Initialize the serial connection
        # Note: we pass the buck on raised exceptions
        self.__port = serial.Serial(device, baudrate=baudrate, xonxoff=xonxoff,
                                   dsrdtr=dsrdtr, rtscts=rtscts, timeout=0)
        self.__ser = fdpexpect.fdspawn(self.__port.fileno(), logfile=logfile)

    # Send raw text to radio
    def __send(self, text):
        if (self.__port is None) or (self.__ser is None):
            return False

        try:
            res = self.__ser.send(text)
            time.sleep(0.2)  # Let the serial line catch up
            return res == len(text)
        except:
            return False

    # Form an AT command and send to radio
    def __send_at(self, command):
        # Don't send bytes if in "normal" mode, other radio is listening!
        if not self.__is_command:
            return False

        # Set local or remote prefix
        prefix = ATCommandSet.AT_LOCAL_PREFIX
        if self.__is_remote:
            # Not allowed to tell remote radio to exit command mode
            if command == ATCommandSet.AT_EXIT:
                return False
            prefix = ATCommandSet.AT_REMOTE_PREFIX

        text = '\r\n' + prefix + str(command) + '\r\n'
        return self.__send(text)

    # Look for 'pattern' (string RE) and return MatchObject, or None
    #   if no response seen before __read_timeout
    def __expect(self, pattern_list):
        if (self.__port is None) or (self.__ser is None):
            return None

        try:
            self.__ser.expect(pattern_list, timeout=self.__read_timeout)
            res = self.__ser.match
            time.sleep(0.2)  # Let the serial line catch up
            return res
        except:
            return None

    # Send AT command, then look for pattern
    def __query(self, command, pattern):
        if not self.__send_at(command):
            return None
        val = self.__expect(pattern)
        return val

    # Query for an int
    def __query_int(self, command):
        val = self.__query(command, ['(-?\d+)\r\n'])
        if val is None: return None
        return int(val.group(0))

    # Query for a float
    def __query_float(self, command):
        val = self.__query(command, ['(-?\d+\.\d+)\r\n'])
        if val is None: return None
        return float(val.group(0))

    # Query for literal text or re pattern (as string), return a string
    def __query_exact(self, command, text):
        val = self.__query(command, ['(%s)\r\n' % text])
        if val is None: return None
        s = val.group(0).decode('utf-8')
        return s.rstrip("\r\n")

    ### API ###

    # General notes:
    #  - Methods that change radio or object state return True or False
    #  - Methods that get radio data return that data or None
    #  - All exceptions, or than in __init__(), should be caught in the class

    ### Manage command mode ###

    def enter_command_mode(self):
        # Technically okay to resend this command, but won't see an 'OK' back
        if self.__is_command:
            return False

        # Must not send anything 1 second before/after +++
        # NOTE: Will time out if already in command mode
        time.sleep(1)
        if not self.__send('+++'):
            return False
        if self.__expect(['OK']) is None:
            return False

        self.__is_command = True
        return True

    def leave_command_mode(self):
        # Vacuously true if not in command mode
        if not self.__is_command: return True
        # Don't send bytes if in "normal" mode, other radio is listening!
        self.__is_remote = False
        res = self.__send_at(ATCommandSet.AT_EXIT)
        self.__is_command = False
        return res

    def leave_command_mode_force(self):
        # Overrides mode check, use only if radio is "stuck" in command mode
        self.__is_command = True
        return self.leave_command_mode()

    def is_command_mode(self):
        return self.__is_command

    ### Select local or remote operation ###

    def set_remote_mode(self, remote=False):
        # True = remote (linked) radio, False = local (attached) radio
        self.__is_remote = remote

    def is_remote_mode(self):
        return self.__is_remote

    ### Get general info ###

    def get_radio_version(self):
        # Version may be a positive float with an appended string
        return self.__query_exact(ATCommandSet.AT_SHOW_VER,
                                  '\d+\.\d+[^\r\n]*')

    def get_board_type(self):
        return self.__query_int(ATCommandSet.AT_SHOW_BRD_TYPE)

    def get_board_frequency(self):
        return self.__query_int(ATCommandSet.AT_SHOW_BRD_FREQ)

    def get_board_version(self):
        return self.__query_int(ATCommandSet.AT_SHOW_BRD_VER)

    # Return a multi-line string containing all parameters, for display
    def get_params_text(self):
        res = self.__query_exact(ATCommandSet.AT_SHOW_PARAM, 'S0:.*S14:.*')
        if res is None: return None
        return res

    ### Parameters (settings) access ###

    def get_param(self, p_id):
        # Assumes all params are ints
        return self.__query_int(ATCommandSet.AT_PARAM + str(p_id) + '?')

    def set_param(self, p_id, p_val):
        return self.__query_exact(ATCommandSet.AT_PARAM + str(p_id) + '=' + \
                                  str(p_val), 'OK')

    # Stores params to EEPROM (necessary after 1+ set_param() calls)
    def write_params(self):
        return self.__query_exact(ATCommandSet.AT_PARAM_WRITE, 'OK')

    ### Miscellaneous ###

    # Reboot the radio (necessary for settings to take effect)
    def reboot(self):
        if not self.__send_at(ATCommandSet.AT_REBOOT):
            return False
        # The local radio leaves command mode upon reboot
        if not self.__is_remote:
            self.__is_command = False
        return True

    # Unstick local radio from bootloader mode (must be out of command mode)
    # NOTE: based on a hack found in the ardupilot code
    # TODO: this breaks the underlying abstraction; consider revising
    def unstick(self):
        if self.__is_command:
            return False
        try:
            old_baudrate = self.__port.baudrate
            old_rtscts = self.__port.rtscts
            old_dsrdtr = self.__port.dsrdtr
            old_xonxoff = self.__port.xonxoff
            self.__port.baudrate = 115200
            self.__port.rtscts = False
            self.__port.dsrdtr = False
            self.__port.xonxoff = False
            for i in range(3):
                time.sleep(0.001)
                self.__port.write(str(0x30).encode())
                self.__port.write(str(0x20).encode())
            time.sleep(1)  # Let the radio reset
            self.__port.baudrate = old_baudrate
            self.__port.rtscts = old_rtscts
            self.__port.dsrdtr = old_dsrdtr
            self.__port.xonxoff = old_xonxoff
        except Exception as ex:
            print("Unsticking error: ", ex)
            return False
        return True

### User Interface ###

if __name__ == '__main__':
    # Modules used by the CLI
    import argparse, re

    # Exit codes
    EXIT_OK=0       # Succeeded
    EXIT_ARGS=1     # Argument error
    EXIT_OPEN=2     # Error opening device
    EXIT_CMDMODE=3  # Error entering command mode
    EXIT_CMDERR=4   # Error performing command

    # Mapping of parameter names to numeric indices (and vice versa)
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
                  'rtscts' : ATCommandSet.PARAM_RTSCTS,
                  'maxwindow' : ATCommandSet.PARAM_MAX_WINDOW }
    inv_param_map = { v:k for k,v in param_map.items() }

    # Grok arguments
    parser = argparse.ArgumentParser(description='Change settings on local and remote radio.',
                                     epilog="Settable parameters (can use multiple --set-*): %s" % \
                                            " ".join(sorted(param_map.keys())))
    parser.add_argument("--baudrate", type=int, default=57600, help='connect at baud rate')
    parser.add_argument("--rtscts", action='store_true', default=False, help='connect using rtscts')
    parser.add_argument("--dsrdtr", action='store_true', default=False, help='connect using dsrdtr')
    parser.add_argument("--xonxoff", action='store_true', default=False, help='connect using xonxoff')
    parser.add_argument("--debug", action='store_true', default=False, help='intermix raw AT traffic')
    parser.add_argument("-f", "--force", action='store_true', default=False, help='try to unstick radio first')
    parser.add_argument("-l", "--list-local", action='store_true', default=False,
                        help='list local parameters and exit')
    parser.add_argument("-r", "--list-remote", action='store_true', default=False,
                        help='list remote parameters and exit')
    parser.add_argument("-L", "--set-local", nargs=2, action='append', metavar=('PARAM', 'VALUE'),
                        help='set local parameter (will reboot radio at end)')
    parser.add_argument("-R", "--set-remote", nargs=2, action='append', metavar=('PARAM', 'VALUE'),
                        help='set remote parameter (will reboot radio at end)')
    parser.add_argument("-B", "--set-both", nargs=2, action='append', metavar=('PARAM', 'VALUE'),
                        help='set on BOTH radios (takes precedence)')
    parser.add_argument("device", help='locally attached radio device')
    args = parser.parse_args()

    # If no get/set was requested, then bail
    if not (args.list_local or args.list_remote or \
            args.set_local or args.set_remote or args.set_both):
        print("Please specify a --list-* or --set-* operation (try -h if unsure)")
        sys.exit(EXIT_ARGS)
    # Also bail if attempting to get and set (we could, but we don't)
    if (args.list_local or args.list_remote) and \
       (args.set_local or args.set_remote or args.set_both):
        print("We don't support listing and setting in the same command")
        sys.exit(EXIT_ARGS)

    # Parse any --set-* args and build dictionaries of parameters to set
    # Note: --set-both overrides --set-local and --set-remote. Beyond that,
    # we don't guard against the user specifying strange combinations.
    def _parse_set(params, myset):
        for pair in params:
            prm, val = pair
            if prm not in param_map:
                print("Parameter not valid: %s" % prm)
                sys.exit(EXIT_ARGS)
            try:
                myset[prm] = int(val)
            except:
                print("Param '%s' value must be an integer: %s" % (prm, val))
                sys.exit(EXIT_ARGS)
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

    # NOTE: If we get past here, we have valid params and will start to work
    #   with the actual radio

    # Initialize the serial connection
    try:
        at = ATCommandSet(args.device, baudrate=args.baudrate,
                          dsrdtr=args.dsrdtr, rtscts=args.rtscts,
                          xonxoff=args.xonxoff, debug=args.debug)
    except Exception as ex:
        print("Error opening serial connection to %s: %s" % \
              (args.device, str(ex)))
        sys.exit(EXIT_OPEN)

    # In case the radio was left in command mode, we can force it out
    # (Could just not "enter" command mode, but this seems safer somehow)
    # 08/06/2014 - Added "unstick" from bootloader mode
    if args.force:
        print("Forcing out of command mode first ...")
        if not at.leave_command_mode_force():
            sys.exit(EXIT_CMDMODE)
        print("Unsticking from bootloader mode ...")
        if not at.unstick():
            sys.exit(EXIT_CMDMODE)

    # Try a command a few times
    def _try(cmd, tries=3):
        res = None
        for i in range(tries):
            try:
                res = cmd()
                if res not in [False, None]: return res
                if i < tries-1:
                    print("... retrying ...")
            except Exception as ex:
                if i < tries-1:
                    print("... retrying (exception: %s) ..." % str(ex))
        return res

    # Enter command mode
    print("Entering command mode ...")
    if not _try(at.enter_command_mode):
        print("Could not enter command mode; try replugging radio or --force")
        at.leave_command_mode_force()
        sys.exit(EXIT_CMDMODE)

    # If --list-* was requested, do that and exit (don't set any parameters)
    def _list_info():
        r_ver = _try(at.get_radio_version)
        b_typ = _try(at.get_board_type)
        b_ver = _try(at.get_board_version)
        if r_ver is None or b_typ is None or b_ver is None:
            print("** Could not access radio version information **")
            return False
        print("radio version: %s  board type: %d  board version: %d" % \
              (r_ver, b_typ, b_ver))
        r_prm = _try(at.get_params_text)
        if r_prm is None:
            print("** Could not access radio parameters **")
            return False
        p_text = "Parameters:"
        for prm, val in re.findall('S(\d+):.*=(\d+)', r_prm):
            p_text += "\n  %-12s   %7u" % (inv_param_map[int(prm)], int(val))
        print(p_text)
        return True
    succeeded = True
    if args.list_local:
        print("Querying local radio ...")
        succeeded = _list_info()
    if succeeded and args.list_remote:
        at.set_remote_mode(True)
        print("Querying remote radio ...")
        succeeded = _list_info()
        at.set_remote_mode(False)
    if args.list_local or args.list_remote:
        print("Leaving command mode ...")
        at.leave_command_mode()
        if not succeeded: sys.exit(EXIT_CMDERR)
        sys.exit(EXIT_OK)

    # If --set-* was requested, attempt to do all of them, then write and reboot
    # only the radio(s) that was/were changed
    def _set_params(myset):
        for prm in myset:
            if not _try(lambda: at.set_param(param_map[prm], myset[prm])):
                print("Failed to set %s, aborting without saving changes." % prm)
                return False
            print("Set %s to %d" % (prm, myset[prm]))
        if not _try(at.write_params):
            print("Failed to write parameters to EEPROM, aborting without saving changes.")
            return False
        print("Wrote parameters to EEPROM.")
        if not _try(at.reboot):
            print("Failed to command reboot; please manually reboot the radio.")
            return False
        print("Commanded reboot; changes should be in effect momentarily.")
        at.reboot()  # Do one more just to be sure
        return True
    # Try remote radio first
    succeeded = True
    if remote_set:
        at.set_remote_mode(True)
        if not _try(at.get_radio_version):
            print("Could not contact remote radio, aborting without saving changes.")
            succeeded = False
        else:
            print("Changing settings on remote radio ...")
            succeeded = _set_params(remote_set)
        at.set_remote_mode(False)
    # Try local radio second (only if no remote failures)
    if local_set and succeeded:
        # Since we have to successfully be in command mode, don't need more checks
        print("Changing settings on local radio ...")
        succeeded = _set_params(local_set)
    # If we didn't reboot the local radio, leave command mode
    if at.is_command_mode():
        print("Leaving command mode ...")
        at.leave_command_mode()
    if not succeeded: sys.exit(EXIT_CMDERR)
    sys.exit(EXIT_OK)
