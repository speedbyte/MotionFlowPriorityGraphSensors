import telnetlib
import time
import re



# Command line interface (cli)
#
# The CLI logic module (named "cli") provides a command line interface
# available through telnet, a local socket, a pipe, or a serial
# line. The CLI is used for changing settings in runtime, which is of
# course only suitable when BruteFIR is used in realtime. It can be
# used interactively by hand, for example by connecting to it through
# telnet. It is also suitable for scripting BruteFIR, or using it as a
# means of interprocess communication if BruteFIR is used as the
# convolution engine for another program. 
#
# The context sensitive port field specifies which interface will be
# used as follows: 
#
#    * port: <INTEGER: TCP port number>; the CLI will listen on the
#            given port number for incoming telnet clients. 
#    * port: <STRING: "/dev/" ...>; when the string starts with
#            "/dev/" the CLI assumes a serial device (such as
#            "/dev/ttyS0" on Linux) is pointed out, and opens it as a
#            serial port, with the default line speed 9600 baud, if
#            not the line_speed field is used specifying another
#            speed. 
#    * port: <STRING: name of local socket>; any other string not
#            starting with "/dev/" is handled as the file name for a
#            local socket, and the CLI will create and listen for
#            incoming connections on the given path. If the path
#            exists, it will be replaced.
#    * port: <INTEGER: read end file descriptor>, <INTEGER: write end
#            file descriptor>; the CLI will assume that the given file
#            descriptors are already opened and ready for use, and
#            will attach the read end to CLI input, and the write end
#            to CLI output. This interface is suitable as interprocess
#            communication when BruteFIR is integrated into another
#            program, and is started through fork() and exec(). 
#
# The CLI does not have much terminal functionality to speak of, and
# is thus a bit cumbersome to use interactively. It reads a whole line
# at a time, and can interpret backspace, but that is about it. There
# is no echo functionality so the connecting client needs to handle
# that (telnet does, and terminal software for serial lines usually
# have a function to enable local echo).
#
# Instead of specifying a port, one can specify a string of commands,
# which will be run in a loop as a script. Example: "cli" { script:
# "cfc 0 0;; sleep 10;; cfc 0 1;; sleep 10"; }. The script may span
# several lines. Each line is carried out atomically (this is also
# true for command line mode), so if there are several commands on a
# single line, separated with semicolon, they will be performed
# atomically. The exception is when an empty statement is put in the
# line (just a semicolon), like in the script example, this will work
# as a line break, and thus separate atomic statements.
#
# A typical use for atomic statements is to change filter coefficents
# and volume at the same time.
#
# The sleep function in the CLI allows for sleeping in seconds,
# milliseconds or blocks. One block is exactly the filter length in
# samples, and if partitioned, it is the length of the
# partition. Block sleep can only be used in script mode.
#
# When in script mode, the first atomic statements will be executed
# just before the first block is processed, then the block is process
# (and sent to the output), and then the next set of atomic statements
# is run. It is not recommended to combine a sleep command with other
# statements in one line, since it makes the scripts harder to
# understand.
#
# If the field echo is set to true, the CLI commands will be echoed
# back to the user (the whole line at a time). This is off per
# default.
#
# When connected and you type "help" at the prompt, you will get the
# following output:
#
# Commands:
#
# lf -- list filters.
# lc -- list coefficient sets.
# li -- list inputs.
# lo -- list outputs.
# lm -- list modules.

# cfoa -- change filter output attenuation.
#         cfoa <filter> <output> <attenuation|Mmultiplier>
# cfia -- change filter input attenuation.
#         cfia <filter> <input> <attenuation|Mmultiplier>
# cffa -- change filter filter-input attenuation.
#         cffa <filter> <filter-input> <attenuation|Mmultiplier>
# cfc  -- change filter coefficients.
#         cfc <filter> <coeff>
# cod  -- change output delay.
#         cod <output> <delay>
# cid  -- change input delay.
#         cid <input> <delay>
# tmo  -- toggle mute output.
#         tmo <output>
# tmi  -- toggle mute input.
#         tmi <input>
# imc  -- issue input module command.
#         imc <index> <command>
# omc  -- issue output module command.
#         omc <index> <command>
# lmc  -- issue logic module command.
#         lmc <module> <command>
#
# sleep -- sleep for the given number of seconds [and ms], or blocks.
#          sleep 10 (sleep 10 seconds).
#          sleep b10 (sleep 10 blocks).
#          sleep 0 300 (sleep 300 milliseconds).
# abort -- terminate immediately.
# tp    -- toggle prompt.
# ppk   -- print peak info, channels/samples/max dB.
# rpk   -- reset peak meters.
# upk   -- toggle print peak info on changes.
# rti   -- print current realtime index.
# quit  -- close connection.
# help  -- print this text.
#
# Notes:
# 
# - When entering several commands on a single line,
#   separate them with semicolons (;).
# - Inputs/outputs/filters can be given as index
#   numbers or as strings between quotes ("").
#
# Most commands are simple and don't need to be further
# explained. Naturally, any changes will lag behind as long as the I/O
# delay is. The exception is the mute and change delay commands, they
# will lag behind as long as the period size of the sound card is,
# which most often is smaller than the program's total I/O
# delay. However, when there is a virtual channel mapping, the mute
# and delay will be lagged as well.
#
# The imc, omc and lmc commands are used to give commands to I/O
# modules and logic modules in run-time. To find out which modules
# that are loaded and which indexes they have, use the command lm. Not
# all modules support run-time commands though.
#
# Changing attenuations with cffa, cfia and cfoa can be done with dB
# numbers or simply by giving a multiplier, which then is prefixed
# with m, like this cfoa 0 0 m-0.5. Changing the attenuation with dB
# will not change the sign of the current multiplier. 



class bfTelnet(object):
  """

  """
  def __init__(self, ip_adress=None):
    """

    """
    self._tn_connection = telnetlib.Telnet()
    self._ip_adress = None
    self._cmd_terminator = '\n'
    if (ip_adress is not None):
      self.open(ip_adress)

  def open(self, ip_adress=None):
    """

    """
    if (ip_adress is not None):
      self._ip_adress = ip_adress

    if self._ip_adress is None:
      raise ValueError, "Valid IP adress is needed!"

    self._tn_connection.open(ip_adress,'3000')
    self._tn_connection.read_until("> ",1)

  def close(self):
    """

    """
    self._tn_connection.close()


  def getCommandsQueued(self):
    if self._cmd_terminator == '\n':
      result = False
    elif self._cmd_terminator == ';':
      result = True
    else:
      pass #TODO: raise an interal exception

    return result

  def setCommandsQueued(self, value):
    if not isinstance(value,type(True)):
      raise TypeError, 'Only bool values allowed'

    if value:
      self._cmd_terminator = ';'
    else:
      self._cmd_terminator = '\n'
      self.flushQueue()

  CommandsQueued = property(getCommandsQueued,setCommandsQueued, doc='')


  def list_filters(self):
    """

    """
    self._tn_connection.write('lf\n')
    return self._tn_connection.read_until("\n> ",1).rstrip('\n> ')

  def list_coefficient_sets(self):
    """

    """
    self._tn_connection.write('lc\n')
    return self._tn_connection.read_until("\n> ",1).rstrip('\n> ')

  def list_inputs(self):
    """

    """
    self._tn_connection.write('li\n')
    return self._tn_connection.read_until("\n> ",1).rstrip('\n> ')

  def list_outputs(self):
    """

    """
    self._tn_connection.write('lo\n')
    return self._tn_connection.read_until("\n> ",1).rstrip('\n> ')

  def parse_outputs(self, message=None):
    """

    """
    if message is None:
      message = self.list_outputs()

    lines = message.split('\n')

    if lines[0] != 'Output channels:':
      raise ValueError, "Wrong start string: %s" % (lines[0],)

    del lines[0]

    ms=r'^(\d+):\s*"(.*?)"\s*\(delay:\s*(\d+)\)\s*(\(muted\))?'
    r=re.compile(ms)

    for index in range(len(lines)):
      lines[index]=r.findall(lines[index].strip())

##['0: "ch1" (delay: 0) (muted)',
## '1: "ch2" (delay: 0) (muted)',
## '2: "ch3" (delay: 0)',
## '3: "ch4" (delay: 0)',
## '4: "ch5" (delay: 0)',
## '5: "ch6" (delay: 0)',
## '6: "ch7" (delay: 0)',
## '7: "ch8" (delay: 0)']

##'Output channels:\n0: "ch1" (delay: 0) (muted)\n1: "ch2" (delay: 0) (muted)\n2: "ch3" (delay: 0)\n3: "ch4" (delay: 0)\n4: "ch5" (delay: 0)\n5: "ch6" (delay: 0)\n6: "ch7" (delay: 0)\n7: "ch8" (delay: 0)'
    return lines

  def flushQueue(self):
    """

    """
    self._tn_connection.write('\n')
    self._tn_connection.read_until("> ",1)

  def change_filter_output_attenuation(self,filter_,output,value):
    """

    cfoa -- change filter output attenuation.
    cfoa <filter> <output> <attenuation|Mmultiplier>

    """
    cmd = "cfoa %d %d %s%s" % (filter_, output, value, self._cmd_terminator)
    self._tn_connection.write(cmd)
    if not self.CommandsQueued:
      self._tn_connection.read_until("> ",1)

  def change_output_delay(self, output, delay):
    """
    cod  -- change output delay.
    cod <output> <delay>

    """
    cmd = "cod %d %d%s" % (output, delay, self._cmd_terminator)
    self._tn_connection.write(cmd)
    if not self.CommandsQueued:
      self._tn_connection.read_until("> ",1)
  
  def toggle_mute_output(self, output):
    """

    # tmo  -- toggle mute output.
    #         tmo <output>

    """
    cmd = "tmo %d%s" % (output, self._cmd_terminator)
    self._tn_connection.write(cmd)
    if not self.CommandsQueued:
      self._tn_connection.read_until("> ",1)

  def toggle_mute_input(self, input):
    """
    
    # tmi  -- toggle mute input.
    #         tmi <input>

    """
    cmd = "tmi %d%s" % (output, self._cmd_terminator)
    self._tn_connection.write(cmd)
    if not self.CommandsQueued:
      self._tn_connection.read_until("> ",1)




# cfia -- change filter input attenuation.
#         cfia <filter> <input> <attenuation|Mmultiplier>

# cffa -- change filter filter-input attenuation.
#         cffa <filter> <filter-input> <attenuation|Mmultiplier>

# cfc  -- change filter coefficients.
#         cfc <filter> <coeff>

# cod  -- change output delay.
#         cod <output> <delay>

# cid  -- change input delay.
#         cid <input> <delay>

# tmo  -- toggle mute output.
#         tmo <output>

# tmi  -- toggle mute input.
#         tmi <input>

# imc  -- issue input module command.
#         imc <index> <command>

# omc  -- issue output module command.
#         omc <index> <command>

# lmc  -- issue logic module command.
#         lmc <module> <command>







##tn=telnetlib.Telnet()
##tn.open("192.168.2.2",'3000')
##print tn.read_until("> ",1)


##def get_help():
##  tn.write('help\n')
##  print tn.read_until("\n> ",1)

##def list_filters():
##  tn.write('lf\n')
##  print tn.read_until("\n> ",1)

##def list_coefficient_sets():
##  tn.write('lc\n')
##  print tn.read_until("\n> ",1)

##def list_inputs():
##  tn.write('li\n')
##  print tn.read_until("\n> ",1)

##def list_outputs():
##  tn.write('lo\n')
##  print tn.read_until("\n> ",1)

##def switch_to_a():
##  tn.write('cfoa "left speaker direct path" "left" M1; cfoa "right speaker direct path" "right" M1;cfoa "left speaker cross path" "left" M0; cfoa "right speaker cross path" "right" M0\n')
##  print tn.read_until("> ",1)

##def switch_to_b():
##  tn.write('cfoa "left speaker direct path" "left" M0; cfoa "right speaker direct path" "right" M0;cfoa "left speaker cross path" "left" M1; cfoa "right speaker cross path" "right" M1\n')
##  print tn.read_until("> ",1)

##def toggle_mute():
##  tn.write('tmi "left"; tmi "right"\n')
##  print tn.read_until("> ",1)

#tn.close()

