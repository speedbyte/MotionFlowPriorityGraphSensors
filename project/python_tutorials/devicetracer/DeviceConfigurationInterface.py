import os
import ConfigParser
import logging

class ConfigurationInterface:
    def __init__(self,SteeringDataFile):
        self.save_cwd=os.getcwd()  
        # now handle config file      
        SteeringData=self.ReadConfigFile(SteeringDataFile)                
        if (len(SteeringData.sections())== 0):
            print "\nFile " + SteeringDataFile + " not found."
            print "Stop to start program.\n"            
            self.SteeringData = None
            return 
        
        #extern steering info     
        try:
            self.GoodDeviceIDs=eval(SteeringData.get("DEVICE_INFO","GOODDEVICEIDS"))
        except ConfigParser.NoOptionError:                 
            self.GoodDeviceIDs =[]
            pass # do nothing more
        
        try:
            self.Drivers=SteeringData.get("DRIVER_INFO","DRIVERS")
        except ConfigParser.NoOptionError:                 
            self.Drivers = ''
            pass # do nothing more

        try:
            self.BaudRate=SteeringData.get("DEVICE_INFO","BAUDRATE")
        except ConfigParser.NoOptionError:                 
            self.BaudRate = "500"
            pass # do nothing more
        
        try:
            self.CurrentDevice=SteeringData.get("DEVICE_INFO","CURRENTDEVICE")
        except ConfigParser.NoOptionError:                 
            self.CurrentDevice = "CAN"
            pass # do nothing more
        
        try:
            self.PortNumber=SteeringData.get("DEVICE_INFO","PORTNUMBER")
        except ConfigParser.NoOptionError:                 
            self.PortNumber = "COM1"
            pass # do nothing more
        
        try:
            self.LogFileFormat=SteeringData.get("TRACE_INFO","LOGFILEFORMAT")
        except ConfigParser.NoOptionError:                 
            self.LogFileFormat = "asc"
            pass # do nothing more                  
        
        try:
            self.DoLogTrace=SteeringData.get("TRACE_INFO","DO_LOGTRACE")
        except ConfigParser.NoOptionError:                 
            self.DoLogTrace = "false"
            pass # do nothing more
        
        try:
            self.DoCanTrace=SteeringData.get("TRACE_INFO","DO_CANTRACE")
        except ConfigParser.NoOptionError:                 
            self.DoCanTrace = "false"
            pass # do nothing more
        try:
            self.DoGpsTrace=SteeringData.get("TRACE_INFO","DO_GPSTRACE")
        except ConfigParser.NoOptionError:                 
            self.DoGpsTrace = "false"
            pass # do nothing more

        try:
            self.FrameSize=eval(SteeringData.get("WINDOW_INFO","FRAME_SIZE"))
        except ConfigParser.NoOptionError:                 
            self.FrameSize = [550,350]
            pass # do nothing more
        
        try:
            self.WindowIsMaximized=eval(SteeringData.get("WINDOW_INFO","ISMAXIMIZED"))
        except ConfigParser.NoOptionError:                 
            self.WindowIsMaximized = False 
            pass # do nothing more            
        
        # set reference      
        self.SteeringData = SteeringData
        return
    
    def ReadConfigFile(self,file):        
        self.ConfigFileName = file
        self.CF = ConfigParser.ConfigParser()
        self.CF.read(self.ConfigFileName)
        return self.CF
        
    def WriteConfigFileData(self):
        logging.info("Writing to config file")
        self.CF.write(open(self.ConfigFileName, 'wb')) 
        
    def Set(self, Section, Option, Value):
        try:
            self.SteeringData.has_section(Section)
            self.SteeringData.set(Section, Option, Value)
        except ConfigParser.NoSectionError:
            print "Error: Config file Set -> Section %s  do not exist" %(Section)