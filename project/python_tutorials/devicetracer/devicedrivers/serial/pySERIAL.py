import _winreg as winreg
import itertools
import os

class SerialDriver(object):
    def __init__(self):
        self.serialports = []
    
    def _enumerate_serial_ports(self):
        """ Uses the Win32 registry to return an
            iterator of serial (COM) ports
            existing on this computer.
        """
        if os.name == "nt":
            path = 'HARDWARE\\DEVICEMAP\\SERIALCOMM'
            try:
                key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path)
            except WindowsError:
                raise
            for i in itertools.count():
                try:
                    val = winreg.EnumValue(key, i)
                    yield str(val[1])
                except EnvironmentError:
                    break
    
    def getSerialPortList(self):
        for portname in self._enumerate_serial_ports():
            self.serialports.append(portname)
        return self.serialports
        
    def __dir__(self):
        pass
       
    
