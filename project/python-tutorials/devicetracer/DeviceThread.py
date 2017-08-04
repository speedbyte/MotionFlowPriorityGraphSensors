import wx
import threading
import Queue
# import devicedrivers.can.pyCAN as pyCAN
# import devicedrivers.serial.pySERIAL as pySERIAL
import pyCAN
import pySERIAL
import time
import serial
import re
from pynmea.streamer import NMEAStream
from pynmea import nmea
import logging
from serial.serialutil import SerialException


#dataq = Queue.Queue()
dataq_fahrtenbuch = Queue.Queue()

class DeviceThreadCan(threading.Thread):

    def __init__(self, threadNum, window, startsync, filter, dataq):
        threading.Thread.__init__(self)
        self.threadNum = threadNum
        self.window = window
        self.filter = filter
        self.timeToQuit = threading.Event()
        self.timeToQuit.clear()
        self.dataq = dataq
        self.messageDelay = 0.001 # 1ms   
        if ( self.window.cancheckbox.GetValue() == True ):
            try:
                self.deviceDriver = pyCAN.CanDriver()
                self.deviceDriver.open(self.window.baudrate)
            except WindowsError as e:
                dlg = wx.MessageDialog(self.window, "CAN Hardware " + str(e),
                                       'Attention',
                                       wx.OK | wx.ICON_INFORMATION
                                       #wx.YES_NO | wx.NO_DEFAULT | wx.CANCEL | wx.ICON_INFORMATION
                                   )
                dlg.ShowModal()
                dlg.Destroy()
                self.window.cancheckbox.SetValue(False)
                logging.error("can hardware not found")
            except pyCAN.REGTEST_Error as e:
                dlg = wx.MessageDialog(self.window, "CAN Hardware " + str(e),
                                       'Attention',
                                       wx.OK | wx.ICON_INFORMATION
                                       #wx.YES_NO | wx.NO_DEFAULT | wx.CANCEL | wx.ICON_INFORMATION
                                   )
                dlg.ShowModal()
                dlg.Destroy()
                self.window.cancheckbox.SetValue(False)
                logging.error("can hardware not found")
            else:
                logging.info("can hardware ok")
        self.TxQueue = Queue.Queue()
        self.startsync=startsync
        self.energy_total = 0
        self.kilometerStart = 0
        self.kilometerEnd = 0
        self.klima = 0
        self.licht = 0
        self.fanner = 0
        self.soc = 0
        self.spannung_total = 0        
        self.gpsSignalValidity = False
        self.geschwindigkeit = 0

    def stop(self):
        self.timeToQuit.set()
        if ( self.window.cancheckbox.GetValue() == True ) :
            self.join(1)
        msg = ""
        msg = "%s%d\t\t" % (msg, self.kilometerStart)
        msg = "%s%d\t" % (msg, self.kilometerEnd)
        msg = "%s%f\t" % (msg, self.energy_total)
        msg = "%s%s\t" % (msg, self.window.currentDriver)
        #print "thread %s" % msg
        if ( self.energy_total != 0):
            dataq_fahrtenbuch.put(msg)
        self.energy_total = 0
        self.kilometerStart = 0
        self.kilometerEnd = 0

    def run(self):
        msg = "Starting DeviceThread iterating with a delay of %.1f msec\n" \
              % (self.messageDelay*1000)
        wx.CallAfter(self.window.LogMessage, msg)
        # open connection to Peak Interface 
        start = self.startsync
        # worker loop for Device - Read and Write 
        sndctr = 0
        sleepCtr = 0          
        while(True):            
            # time to quit is set
            if self.timeToQuit.isSet():
                break              
            # read Device message from the Device bus
            self.ReceiveDeviceMsg(start)
            # send DeviceMsg over Peak Device
            if sndctr > 5:                   
                self.SendDeviceMsg()                        
                sndctr = 0
            sndctr += 1            
            # time this thread sleeps for other jobs
            if sleepCtr > 25:                
                time.sleep(self.messageDelay)                        
                sleepCtr = 0
            sleepCtr += 1   
        #  end of read write DeviceMsg while Loop
            
        # Close connection to Peak Interface 
        
        if ( self.window.cancheckbox.GetValue() == True ) :
            self.deviceDriver.close()
        
    def SendDeviceMsg(self):
        # send DeviceMsg over Peak Device
        try:
            msg_to_send = self.TxQueue.get(False)
            something_to_send = True
        except Queue.Empty:
            something_to_send = False
        # A message is in the queue now out to the bus  
        if something_to_send:
            wx.CallAfter(self.window.LogMessage, "Msg to send: "
                         + str(msg_to_send) + '\n')
            self.deviceDriver.write(msg_to_send)  
    
    def ReceiveDeviceMsg(self,start):
        # read device message from the Device bus
        time1 = (time.clock()-start)
        timeDelta = "%11s" % ("%4.6f" %(time1))
        try:
            if self.window.cancheckbox.GetValue() == True:
                result = self.deviceDriver.read()
        except pyCAN.QRCVEMPTY_Error:
            pass
            #logging.error("Receive empty error")
        except pyCAN.UNKNOWN_Error:
            logging.error("Unknown error")            
        else:
            # check Device Filter Info
            check = False
            if self.filter[0] != "":
                for elem in self.filter:
                    if self.window.cancheckbox.GetValue() == True:
                        if elem == "%.3X" % result.ID:
                            check = True
            else:
                check = True
            data_tuple = ("%.6f" %(time1),"1","0x%X" % result.ID,"Rx","d",str(result.LEN))
            #msg = timeDelta +" 1 " + "%4s" % "%.3X" % result.ID + "             Rx   d " + str(result.LEN) + " "
            if (result.ID == 0x448 ): #0x448 ):
                for index in range(result.LEN):
                    data_tuple = data_tuple + ("%.2X" % result.DATA[index],)
                    #msg = "%s %.2X" % (msg, result.DATA[index])
                    spannung_index = 52/8 # start 52, len 13
                    if index == spannung_index:
                        spannung_wert_teil1 = (0b11111 & result.DATA[index])<<8
                    if index == (spannung_index+1):
                        spannung_wert_teil2 = (0xFF & result.DATA[index])
                        self.spannung_total = ( spannung_wert_teil1 + spannung_wert_teil2 ) / 10.0 
                        if self.spannung_total > 400 :
                            logging.warning("invalid value for spannung = %d, changing to 0", self.spannung_total)
                            self.spannung_total = 0
#                         msg = "%s\t%f\tV" % (msg, self.spannung_total)
#                 msg = msg + "\n"
#                 try:
#                     dataq.put(msg)
#                 except:
#                     logging.error("can data queue failed")
                return
            elif (result.ID == 0x508 ): #508:
                keyword_can = "ENERGY"
                msg = "%s\t%s" % (keyword_can, timeDelta)
                for index in range(result.LEN):
                    data_tuple = data_tuple + ("%.2X" % result.DATA[index],)
                    #msg = "%s %.2X" % (msg, result.DATA[index])
                    strom_index = 21/8  # begin 21, len 14
                    if index == strom_index:
                        strom_wert_teil1 = (0b111111 & result.DATA[index])<<8
                    if index == (strom_index+1):
                        strom_wert_teil2 = (0xFF & result.DATA[index])
                        strom_total = (strom_wert_teil1 + strom_wert_teil2)
                        strom_total_factor = -(strom_total/10.0 - 819.2)
                        if strom_total_factor > 250 :
                            logging.warning("invalid value for strom = %f, changing to 0", strom_total_factor)
                            strom_total_factor = 0
                        energy_current = (strom_total_factor*self.spannung_total*0.1)/3600000
                        self.energy_total = energy_current + self.energy_total
                        msg = "%s\t%f\t%f\t%f\t%d\t%d\t%d\t%d\t%d" % (msg, strom_total_factor, self.spannung_total, energy_current, self.licht, self.klima, self.fanner, self.soc, self.geschwindigkeit)
                msg = msg + "\n"
                try:
                    #dataq.mutex.acquire()
                    self.dataq.put(msg)
                    #dataq.mutex.release()
                except:
                    logging.error("can data queue failed")
                return
            elif (result.ID == 0x100 ):
                for index in range(result.LEN):
                    data_tuple = data_tuple + ("%.2X" % result.DATA[index],)
                    #msg = "%s %.2X" % (msg, result.DATA[index])
                    strom_sensor_index = 2
                    if index == strom_sensor_index:
                        strom_sensor_value = result.DATA[index]<<8
                    if index == (strom_sensor_index+1):
                        strom_sensor_value = result.DATA[index] + strom_sensor_value 
                        msg = "%s\t%d\tV" % (msg, strom_sensor_value)
#                 msg = msg + "\n"
#                 try:
#                     dataq.put(msg)
#                 except:
#                     logging.error("can data queue failed")
            elif (result.ID == 0x412 ):
                msg = "KM"
                for index in range(result.LEN):
                    data_tuple = data_tuple + ("%.2X" % result.DATA[index],)
                    #msg = "%s %.2X" % (msg, result.DATA[index])
                    if index == 1:
                        self.geschwindigkeit = result.DATA[index]
                    if index == 3:
                        km_teil_1 = result.DATA[index]<<8
                    if index == 4:
                        km_teil_2 = result.DATA[index]
                        km_total = (km_teil_1 + km_teil_2)
                if self.kilometerStart == 0:
                    self.kilometerStart = km_total
                self.kilometerEnd = km_total
            elif (result.ID == 0x423 ):
                msg = "LICHT"
                for index in range(result.LEN):
                    data_tuple = data_tuple + ("%.2X" % result.DATA[index],)
                    #msg = "%s %.2X" % (msg, result.DATA[index])
                    if index == 3:
                        licht = result.DATA[index]
                if licht == 0:
                    self.licht = 0
                else:
                    self.licht = 1
#                 msg = msg + "\n"
#                 try:
#                     dataq.put(msg)
#                 except:
#                     logging.error("can data queue failed")
            elif (result.ID == 0x443 ):
                '''
                443 : first byte : blase higher nibble 80
                ac , lower nibble : 1
                
                blase stufe: 3d5 : 6th byte
                3,5,6,8,B
                '''
                msg = "KLIMA"
                for index in range(result.LEN):
                    data_tuple = data_tuple + ("%.2X" % result.DATA[index],)
                    #msg = "%s %.2X" % (msg, result.DATA[index])
                    if index == 0:
                        klima = result.DATA[index] & 0x1
                if klima == 0:
                    self.klima = 0
                else:
                    self.klima = 1
#                 msg = msg + "\n"
#                 try:
#                     dataq.put(msg)
#                 except:
#                     logging.error("can data queue failed")
            elif (result.ID == 0x3d5 ):
                msg = "FANNER"
                for index in range(result.LEN):
                    data_tuple = data_tuple + ("%.2X" % result.DATA[index],)
                    #msg = "%s %.2X" % (msg, result.DATA[index])
                    if index == 5:
                        fanner = result.DATA[index]
                if fanner < 6:
                    self.fanner = 0
                else:
                    self.fanner = 1
#                 msg = msg + "\n"
#                 try:
#                     dataq.put(msg)
#                 except:
#                     logging.error("can data queue failed")
            elif (result.ID == 0x518 ):
                msg = "SOC"
                for index in range(result.LEN):
                    data_tuple = data_tuple + ("%.2X" % result.DATA[index],)
                    #msg = "%s %.2X" % (msg, result.DATA[index])
                    if index == 7:
                        self.soc = result.DATA[index]
                        self.soc = self.soc/2
#                 msg = msg + "\n"
#                 try:
#                     dataq.put(msg)
#                 except:
#                     logging.error("can data queue failed")


class DeviceThreadGps(threading.Thread):

    def __init__(self, threadNum, window, startsync, filter, dataq):
        threading.Thread.__init__(self)
        self.threadNum = threadNum
        self.window = window
        self.filter = filter
        self.timeToQuit = threading.Event()
        self.timeToQuit.clear()
        self.dataq  = dataq
        self.messageDelay = 0.001 # 1ms   
        if ( self.window.gpscheckbox.GetValue() == True ):
            try:
                self.deviceDriver = pySERIAL.SerialDriver()
                logging.info(self.window.portnumber)
                self.portnamefull = self.full_port_name(self.window.portnumber)
                logging.info(self.portnamefull)
                self.deviceDriver = serial.Serial(self.portnamefull, 9600,  timeout=2)
            except SerialException as e:
                dlg = wx.MessageDialog(self.window, "Serial Hardware " + str(e),
                                       'Attention',
                                       wx.OK | wx.ICON_INFORMATION
                                       #wx.YES_NO | wx.NO_DEFAULT | wx.CANCEL | wx.ICON_INFORMATION
                                   )
                dlg.ShowModal()
                dlg.Destroy()
                self.window.gpscheckbox.SetValue(False)
            else:
                logging.info("serial port ok")
                    
        self.TxQueue = Queue.Queue()
        self.startsync=startsync
        self.gpsSignalValidity = False

    def stop(self):
        self.timeToQuit.set()
        if ( self.window.gpscheckbox.GetValue() == True ) :
            self.join(1)
        

    def run(self):
        msg = "Starting DeviceThread iterating with a delay of %.1f msec \n" \
              % (self.messageDelay*1000)
        wx.CallAfter(self.window.LogMessage, msg)
        # open connection to Peak Interface 
        start = self.startsync
        # worker loop for Device - Read and Write 
        sleepCtr = 0          
        while(True):            
            # time to quit is set
            if self.timeToQuit.isSet():
                break              
            # read Device message from the Device bus
            self.ReceiveDeviceMsg(start)
            time.sleep(0.1)
            # time this thread sleeps for other jobs
            if sleepCtr > 25:                
                time.sleep(self.messageDelay)                        
                sleepCtr = 0
            sleepCtr += 1   
        #  end of read write DeviceMsg while Loop
            
        # Close connection to Peak Interface 
        if ( self.window.gpscheckbox.GetValue() == True ) :
            self.deviceDriver.close()
        
        
    def ReceiveDeviceMsg(self,start):
        # read device message from the Device bus
        time1 = (time.clock()-start)
        timeDelta = "%11s" % ("%4.6f" %(time1))
        try:
            if self.window.gpscheckbox.GetValue() == True:
                result = self.deviceDriver.readline()
        except SerialException as e:
            logging.error(str(e))
        except:
            logging.error("problems with the windows serial port driver")
        else:
            # check Device Filter Info
            check = False
            if self.filter[0] != "":
                for elem in self.filter:
                    if self.window.gpscheckbox.GetValue() == True:
                        val = "%s" % result
                        #print val
                        if elem in "%s" % result:
                            check = True
            else:
                check = True
                streamer = NMEAStream()
                keyword_gps = "GPS"
                msg = "%s\t%s" % (keyword_gps, timeDelta)
                #line = "$GPGGA,,,,,,0,00,99.99,,,,,,*48"
                nmea_ob = streamer._get_type(result)
                if ( nmea_ob != None ):
                    if ( isinstance(nmea_ob(),nmea.GPGGA) is True ):
                        p = nmea.GPGGA()
                        p.parse(result)
                        #msg = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s" % (msg, p.timestamp, p.sen_type, p.latitude, p.lat_direction, p.longitude, p.lon_direction, p.antenna_altitude, p.altitude_units)
                        msg = msg.strip('\r\n')
                        msg  = "%s\t%s" % (msg, result)
                        #msg = msg + "\n"
                        try:
                            if ( self.gpsValiditySignal == True ):
                                #dataq.mutex.acquire()
                                self.dataq.put(msg)
                                #dataq.mutex.release()
                        except:
                            logging.error("gps data queue failed")
                    elif ( isinstance(nmea_ob(),nmea.GPRMC) is True ):
                        p = nmea.GPRMC()
                        p.parse(result)
                        if ( p.data_validity == 'A'):
                            self.gpsValiditySignal = True
                            msg = msg.strip('\r\n')
                            msg  = "%s\t%s" % (msg, result)
                        else:
                            self.gpsValiditySignal = False
                            msg  = "%s\tGPS Signale wird gesucht" %(msg)
                            msg = msg + "\n"
                        #msg = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s" % (msg, p.timestamp, p.sen_type, p.lat, p.lat_dir, p.lon, p.lon_dir, p.spd_over_grnd)
                        #msg = msg + "\n"
                        try:
                            #dataq.mutex.acquire()
                            self.dataq.put(msg)
                            #dataq.mutex.release()
                        except:
                            logging.error("gps data queue failed")
                    else:
                        msg = ""
                else:
                    logging.warning("nmea object is none")
                    
            
    def full_port_name(self,portname):
        """ Given a port-name (of the form COM7,
            COM12, CNCA0, etc.) returns a full
            name suitable for opening with the
            Serial class.
        """
        m = re.match('^COM(\d+)$', portname)
        if m and int(m.group(1)) < 10:
            return portname
        return '\\\\.\\' + portname
