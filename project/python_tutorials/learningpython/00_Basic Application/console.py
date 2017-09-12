""" Utility functions to get input in scripts which run on the console.


    $Date$
    $Rev$
    $Author$
    $URL$

    Copyright (c) 2005 S1nn GmbH & Co. KG.
    All Rights Reserved.

"""
import msvcrt
import winsound

_svn_id = "$Id$"
_svn_rev = "$Rev$"
__version__ = _svn_rev[6:-2]



def get_choice(message, validchars=None):
    """Wait for a keypress and return the key.

    message is printed to the console.

    If validkeys is a tuple of characters only the corresponding keypresses
    are allowed.

    """
    done = False
    print message,
    while not done:
        key = msvcrt.getch()
        if key in ('\x00', '\xe0'):   # special function key pressed ?
            key = msvcrt.getch()
        if validchars is None:
            done = True
        elif key in validchars:
            done = True
        else:
            winsound.MessageBeep(-1)
            continue
    return key

def get_string_input(message, default):
    """Get string input from the console.

    """
    done = False
    print message
    prompt = '--> [%s] ' % (default,)
    while not done:
        s = raw_input(prompt).strip()
        if s=='':
            print 'using default: %s' % (default,)
            result = default
            done = True
        else:
            result = s
            done = True
    return result

def get_int_input(message, default, minval, maxval):
    """Get integer input from the console.

    """
    done = False
    print message
    prompt = '--> [%d] ' % (default,)
    while not done:
        s = raw_input(prompt).strip()
        if s=='':
            print 'using default: %d' % (default,)
            result = default
            done = True
        else:
            try:
                result=int(s)
            except ValueError:
                print "Don't know what do with: %s" % (s,)
                print "Please enter a valid integer:"
                continue
            if result < minval or result > maxval:
                print "Value needs to be between %d and %d" % (minval,maxval)
                continue
            done = True
    return result

def get_float_input(message, default, minval, maxval):
    """Get float input from the console.

    """
    done = False
    print message
    prompt = '--> [%.2f] ' % (default,)
    while not done:
        s = raw_input(prompt).strip()
        if s=='':
            print 'using default: %.2f' % (default,)
            result = default
            done = True
        else:
            try:
                result=float(s)
            except ValueError:
                print "Don't know what do with: %s" % (s,)
                print "Please enter a valid floating point number:"
                continue
            if result < minval or result > maxval:
                print "Value needs to be between %d and %d" % (minval,maxval)
                continue
            done = True
    return result

