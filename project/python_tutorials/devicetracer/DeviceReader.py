import sys
import serial
from pynmea import nmea
from pynmea.streamer import NMEAStream
from serial.serialutil import SerialException
import logging

class GpsDriver(object):
    def __init__(self, serialport, baudratespeed):
        self.gpsdevice = serial.Serial(port=serialport, baudrate=baudratespeed, timeout=5)
        self.init()

    def init(self):
        if self.isOpen():
            return True
        return False

    def open(self):
        self.gpsdevice.open()
        
    def isOpen(self):
        return self.gpsdevice.isOpen()
    
    def readBuffer(self):
        try:
            data = self.gpsdevice.read(1)
            n = self.gpsdevice.inWaiting()
            if n:
                data = data + self.gpsdevice.read(n)
            return data
        except SerialException as e:
            logging.error("Big time read error, what happened: " + str(e))
            sys.exit(1)
        
    def close(self):
        self.gpsdevice.close()
            
if __name__ == '__main__':
     
    
    file_data_fd = open(r'f:\workspace_python\devicetracer\log\GPSlog.asc', 'r')
    streamer = NMEAStream()
    count = 0
#     next_data = p.get_objects()
#     nmea_objects = []
#     while next_data:
#         nmea_objects += next_data
#         next_data = p.get_objects()
#     print nmea_objects[0].sen_type
    
    # Read from the input buffer and append it to our line 
    # being constructed
    line = "$GPGGA,,,,,,0,00,99.99,,,,,,*48"
    nmea_ob = streamer._get_type(line)
    if ( isinstance(nmea_ob(),nmea.GPGGA) is True ):
        print nmea_ob
        p = nmea.GPGGA()
        p.parse(line)
        print p.sen_type
            
              
    count = count + 1
    print count
