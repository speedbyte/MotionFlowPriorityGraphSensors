#!/usr/bin/python
import threading
import Queue
import time
import csv
import serial
import re
from serial.serialutil import SerialException
import struct
import sys
import socket

def crccalc_ascii(command):
    print "recieved command %s\n" % command
    hex_byte = []
    data = 0
    total = 0
    # for 802.15.4 total = total + 0x01 + 0x01 + 0x12 + 0x34 + 0x00 
    total = total + 0x10 + 0x01 + 0x00 + 0x13 + 0xA2 + 0x00 + 0x40 + 0x99 + 0x78 + 0xB0 + 0xFF + 0xFE + 0x00 + 0x00
    # for 802.15.4 data_to_send = [0x7E, 0x00, len(command)+5, 0x01, 0x01, 0x12, 0x34, 0x00]
    data_to_send = [0x7E, 0x00, len(command)+14, 0x10, 0x01, 0x00, 0x13, 0xA2, 0x00, 0x40, 0x99, 0x78, 0xB0, 0xFF, 0xFE, 0x00, 0x00]
    chromo = ''
    UnpackAndConvertToHex = map(hex, struct.unpack(len(command)*'B',command))
    #print UnpackAndConvertToHex
    for x in range (len(UnpackAndConvertToHex)):
        data = int(UnpackAndConvertToHex[x],16)
        data_to_send.append(data)
        total = total + data
    crc_zigbee = 0xFF - ( total )
    for x in range (len(data_to_send)):
        chromo = chromo + struct.pack("B", data_to_send[x])
    chromo = chromo + struct.pack("B", (crc_zigbee & 0xFF ))
    return chromo

def sendtochromoflex (finalresult):
    UnpackAndConvertToHex = map(hex, struct.unpack(len(finalresult)*'B',finalresult))
    print UnpackAndConvertToHex
    magnetThread.deviceDriver.write(finalresult)
    return;

class SocketThread(threading.Thread):

    def __init__(self, threadNum, startsync=None):
        threading.Thread.__init__(self)
        self.timeToQuit = threading.Event()
        self.timeToQuit.clear()
        self.messageDelay = 0.001 # 1ms
        self.deviceDriver = None   
	self.val = ""
	self.newCommand = False

    def stop(self):
	print("socket end")
        self.timeToQuit.set()

    def getCommand(self):
	return self.val

    def run(self):
	socketfd = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
	socketfd.bind(('127.0.0.1', 50000)) 
	socketfd.listen(10)
        while(True):         
	    (s, m) = socketfd.accept()   
	    self.val = s.recv(1024)	            
            print self.val
	    self.newCommand = True
	    s.close()
	    # time to quit is set
            if self.timeToQuit.isSet():
                break              
            # read Device message from the Device bus

class MagnetThread(threading.Thread):

    def __init__(self, threadNum, startsync=None):
        threading.Thread.__init__(self)
        self.threadNum = threadNum
        self.filter = filter
        self.timeToQuit = threading.Event()
        self.timeToQuit.clear()
        self.messageDelay = 0.001 # 1ms
        self.deviceDriver = None   
        self.TxQueue = Queue.Queue()
        self.startsync=startsync
        self.ledstatusstored = None
        self.initMagneticValue = "0"
        self.currentMagneticValue = "0"

    def stop(self):
	print("magnet end")
        self.timeToQuit.set()

    def run(self):
        start = self.startsync
        sndctr = 0
        sleepCtr = 0          
        self.deviceDriver = serial.Serial('/dev/ttyUSB0', 38400)

        while(True):            
            # time to quit is set
            if self.timeToQuit.isSet():
                break              
            # read Device message from the Device bus
            self.readMagneticValues()
            if sleepCtr > 25:                
                time.sleep(self.messageDelay)                        
                sleepCtr = 0
            sleepCtr += 1   

    def readMagneticValues(self):
        result = self.deviceDriver.readline()
	print(result)
        splittedmsg = result.split('#')
        if ( isinstance(splittedmsg, list) ):
            if ( len ( splittedmsg ) > 4 ):
                if ( "MF" in splittedmsg[4]):
                    splitInitMagneticVal = splittedmsg[4].split(';')
                    self.initMagneticValue = splitInitMagneticVal[0][3:]
                    #self.initMagneticValue = splittedmsg[4][3:6] # 3:6
                    msg = "INIT : %s\t%s\n" % ( self.initMagneticValue, splittedmsg[4][7:10])
                elif ( "MF" in splittedmsg[5]) :
                    splitCurrentMagneticVal = splittedmsg[5].split(';')
                    self.currentMagneticValue = splitCurrentMagneticVal[0][3:]
                    #self.currentMagneticValue = splittedmsg[5][3:6] # 3:6
                    #msg = "CURRENT %s\t%s\t%s\n" % ( msg, self.currentMagneticValue, splittedmsg[5][7:10])
                
    def getInitMagneticValue(self):
        return self.initMagneticValue
    
    def getCurrentMagneticValue(self):
        return self.currentMagneticValue


if __name__ == '__main__' :
    count = 0
    filter = 0
    magnetThread = MagnetThread(count, 0)        
    magnetThread.start()
    socketThread = SocketThread(count, 0)        
    socketThread.start()
    
    storedMagneticValueInit = None
    storedMagneticValueCurrent = None
    currentstatusparkplatz = 'None'
    currentledstatus = socketThread.getCommand()
    try:
	while(1):
	        time.sleep(1)
		currentledstatus = socketThread.getCommand()
		if ( socketThread.newCommand == True):
			socketThread.newCommand = False
			serialdata = crccalc_ascii(currentledstatus)
			sendtochromoflex(serialdata)
	  	print("State: %s" % (currentstatusparkplatz,))
	        initMagneticValue = magnetThread.getInitMagneticValue()
	        print "init value" , initMagneticValue
		currentMagneticValue = magnetThread.getCurrentMagneticValue()
	        print "current value" , currentMagneticValue
		if ( initMagneticValue != storedMagneticValueInit ):
	       	    storedMagneticValueInit = initMagneticValue
	        if ( currentMagneticValue != storedMagneticValueCurrent ):
	            storedMagneticValueCurrent = currentMagneticValue
	            #print currentstatusparkplatz
	            difference =  abs(int(currentMagneticValue) - int(initMagneticValue))
	            if ( currentledstatus == "Reserve" and ( (currentstatusparkplatz != 'Comm') and (currentstatusparkplatz != 'Charge') )):
	                currentstatusparkplatz = 'Reserve'
	                currentledstatus = 'None'
	            if ( (currentstatusparkplatz != 'Comm') and  ((currentstatusparkplatz != 'Charge')) and ((currentstatusparkplatz != 'None'))):
	                difference =  abs(int(currentMagneticValue) - int(initMagneticValue))
	                #print int(currentMagneticValue)
	                #print int(initMagneticValue)
	                if ( difference > 50) :
	                    print initMagneticValue
	                    print currentMagneticValue
	                    serialdata = crccalc_ascii('Comm')
	                    sendtochromoflex(serialdata)
	                    currentstatusparkplatz = 'Comm'
	            elif ( currentstatusparkplatz == 'Comm' ):
	                serialdata = crccalc_ascii('Charge')
	                sendtochromoflex(serialdata)
	                currentstatusparkplatz = 'Charge' 
	            elif ( (abs(int(currentMagneticValue) - int(initMagneticValue))) < 40):
	                difference =  abs(int(currentMagneticValue) - int(initMagneticValue))
	                print difference
	                if ( currentstatusparkplatz == 'Reserve'):
	                    pass
	                elif ( currentstatusparkplatz != 'Free'):
	                    currentstatusparkplatz = 'Free'
	                    serialdata = crccalc_ascii('Free')
	                    sendtochromoflex(serialdata)
    except (KeyboardInterrupt):
	print "keyboard interrupt"	
	magnetThread.stop()
 	socketThread.stop()
	sys.exit(0)
    except:
	magnetThread.stop()
	socketThread.stop()
	raise
    

