import wx
import threading
import Queue
import DeviceThread
import time
import datetime
import logging

class DeviceLoggingThread(threading.Thread):
    def __init__(self, threadNum, window, startsync, filter, dataq):
        threading.Thread.__init__(self)
        self.threadNum = threadNum
        self.timeToQuit = threading.Event()
        self.timeToQuit.clear()
        self.filename_energy = 'None'
        self.filename_gps = 'None'
        self.filename_gesamt = 'None'
        self.fileasc_energy = None
        self.fileasc_gps = None
        self.fileasc_gesamt = None
        self.fahrtenbuch = None
        self.writer = None
        self.headerWritten = False
        self.fahrtenbuchWritten = False
        self.timestringBegin = 'Zeitmaschine'
        self.window = window
        self.dataq = dataq

    def stop(self):
        #DeviceThread.dataq.mutex.acquire()
        #DeviceThread.dataq.put(msg)
        #DeviceThread.dataq.mutex.release()
        self.timeToQuit.set()
        self.join(1)
        if self.fileasc_energy:
            self.fileasc_energy.close()
        if self.fileasc_gps:
            self.fileasc_gps.close()
        if self.fileasc_gesamt:
            self.fileasc_gesamt.close()
        if not self.fahrtenbuch:
            try:
                self.fahrtenbuch = open('log\\fahrtenbuch.txt' , "a");
            except:
                logging.error("unable to write the fahrtenbuch file")
            else:
                if self.fahrtenbuch:
                    try:
                        timestringEnd = datetime.datetime.now().strftime('%H:%M:%S') 
                        msg = DeviceThread.dataq_fahrtenbuch.get_nowait()
                        msg = "%s\t%s\t%s\n" % (self.timestringBegin, timestringEnd, msg)
                        self.fahrtenbuch.write(msg )
                    except Queue.Empty:
                        logging.info("no energy consumed")
                    except:
                        logging.info("possibly no fahrtenbuch has been created")
                    while(True):
                        try:
                            DeviceThread.dataq_fahrtenbuch.get_nowait() # flush everything
                        except Queue.Empty:
                            logging.info("successfully stopped all threads")
                            break
            finally:
                if self.fahrtenbuch:
                    self.fahrtenbuch.close()


    def run(self):
        self.timestringBegin = datetime.datetime.now().strftime('%a\t%d-%m-%Y\t%H:%M:%S') 
        while(True):            
            time.sleep(0.01)
            # time to quit is set
            if self.timeToQuit.isSet():
                break              
            if self.window.logstarted is False:
                break
            # read Device message from the Device bus
            try:
                
                msg = self.dataq.get()
                #msg = "test\n"
                if self.window.logstarted is False:
                    break
                wx.CallAfter(self.window.LogMessage, self.FormaterASC(msg))
                if self.window.getFileFormat() == "asc":
                    self.writeASC(msg)
            except Queue.Empty:
                print "Queue empty"
            #  end of read write DeviceMsg while Loop
            
        
    def FormaterASC(self, result):
        # Bsp.:
        #   8.100006 1  600             Rx   d 8 03 54 00 00 00 00 00 00
        #  11.080470 1  700             Rx   d 8 02 1A 90 00 00 00 00 00
        if ( self.headerWritten is False):
            header = "date "+ self.timestringBegin + "\n"
            #header = header + "base hex  timestamps absolute\n" 
            #header = header + "internal events logged\n"
            header = header + self.window.currentDriver + '\n'
            header_energy = header + "\tTimestamp\tampere_A\tvoltage_V\twattage_kWh\tlights\tac\tfanner\tSOC\tspeed"
            header_gps = header + "\tNMEA Format"
            
            self.writeHeader(header_energy, header_gps)
            self.headerWritten = True
        return result
        
    
    def writeHeader(self, header1, header2):
        """
                DATA_2014-07-17_09-56-30.txt
                ENERGY_2014-07-17_09-56-30.txt
                GPS_2014-07-17_09-56-30.txt
        """
        if self.window.getLogStatus():
            try:
                if not self.fileasc_energy:
                    self.filename_energy = 'ENERGY'+ datetime.datetime.now().strftime('_%Y-%m-%d_%H-%M-%S') + '.txt'
                    self.fileasc_energy = open('log\\' + self.filename_energy, "w");
                    logging.info("succesfully opened ENERGY log file")
                if not self.fileasc_gps:
                    self.filename_gps = 'GPS'+ datetime.datetime.now().strftime('_%Y-%m-%d_%H-%M-%S') + '.txt'
                    self.fileasc_gps = open('log\\' + self.filename_gps, "w");
                    logging.info("succesfully opened GPS log file")
                if not self.fileasc_gesamt:
                    self.filename_gesamt = 'DATA'+ datetime.datetime.now().strftime('_%Y-%m-%d_%H-%M-%S') + '.txt'
                    self.fileasc_gesamt = open('log\\' + self.filename_gesamt, "w");
                    logging.info("succesfully opened DATA log file")
                    
                self.fileasc_energy.write(header1 + '\n');
                self.fileasc_gps.write(header2 + '\n');
                self.fileasc_gesamt.write(header1 + '\n');
            except:
                logging.error("unable to open the log files, aborting")
        else:
            logging.warning("log check box not clicked")
        
    def writeASC(self, data_asc):
        """
        2014-07-17_09-56-30
        """
        if self.window.getLogStatus():
            self.fileasc_gesamt.write(data_asc);
            
            if ("GPS" in data_asc):
                self.fileasc_gps.write(data_asc);
            elif ("ENERGY" in data_asc):
                self.fileasc_energy.write(data_asc);
    
