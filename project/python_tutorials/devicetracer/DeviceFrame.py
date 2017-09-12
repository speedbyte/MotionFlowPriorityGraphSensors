import wx
from DeviceThread import DeviceThreadCan, DeviceThreadGps
import devicedrivers.serial.pySERIAL as pySERIAL
import logging
from DeviceLoggingThread import DeviceLoggingThread
import os
import sys
import time
import Queue
import pyCAN

class MyFrame(wx.Frame):
    def __init__(self, ConfigData):
        
        self.ConfigData = ConfigData
        self.FrameSize = [550,350]
        title = "Datalogger for CAN and GPS signals Version_1.0 01.08.2014"
        if (self.ConfigData != None):            
            self.FrameSize[0] = self.ConfigData.FrameSize[0]
            self.FrameSize[1] = self.ConfigData.FrameSize[1]          
        if self.ConfigData.WindowIsMaximized == True:    
            wx.Frame.__init__(self, None, title=title,size = wx.Size(self.FrameSize[0],self.FrameSize[1])\
                              ,style = wx.DEFAULT_FRAME_STYLE | wx.MAXIMIZE)
        else:
            wx.Frame.__init__(self, None, title=title,size = wx.Size(self.FrameSize[0],self.FrameSize[1]))
                          
#         bmp = self.bitmap = wx.Bitmap('icon.bmp') 
#         icon = wx.IconFromBitmap(bmp)
#         self.SetIcon (icon) 
        
        self.threads = []
        self.DeviceThread = [None, None]
        self.DeviceLoggingThread = None
        self.logstarted = False
        self.count = 0        
        self.stringcount = 0
             
        panel = wx.Panel(self)
        self.startBtn = wx.Button(panel, -1, "Start DEVICE thread")
        self.stopBtn  = wx.Button(panel, -1, "Stop DEVICE thread")
        #self.quickAnalyseBtn = wx.Button(panel, -1, "Quick Analyse")
        
        self.tc = wx.StaticText(panel, -1, "DEVICE Threads: 00")
            
        self.log = wx.TextCtrl(panel, -1, "",
                                   style=wx.TE_RICH|wx.TE_MULTILINE)
            
        points = self.log.GetFont().GetPointSize()  # get the current size
        f = wx.Font(points+2, wx.MODERN, wx.NORMAL, wx.NORMAL)
        self.log.SetFont(f)
            
        formatlist = ['asc']        
        self.ch = wx.Choice(panel, -1, (100, 50), choices = formatlist)
        self.ch.SetStringSelection('asc')
        if (self.ConfigData != None):
            LogFileFormat = self.ConfigData.LogFileFormat                
            if (LogFileFormat not in formatlist):                
                LogFileFormat = 'asc'    
        else:          
            LogFileFormat = 'asc'
        self.ch.SetStringSelection(LogFileFormat) 
        self.format = LogFileFormat              
        
        baudlist = ['100','500']        
        self.wxbaudrate = wx.Choice(panel, -1, (100, 50), choices = baudlist)
        self.wxbaudrate.SetStringSelection('500')
        if (self.ConfigData != None):
            BaudRate = self.ConfigData.BaudRate
            if (BaudRate not in baudlist):                
                BaudRate = '500'    
        else:          
            BaudRate = '500'
        self.ch.SetStringSelection(BaudRate) 
        self.baudrate = BaudRate

        if (self.ConfigData != None):
            PortNumber = self.ConfigData.PortNumber
            portnumberlist = [PortNumber]
            self.wxportnumber = wx.Choice(panel, -1, (100,50), choices = portnumberlist)
        else:          
            PortNumber = 'COM1'
            portnumberlist = [PortNumber]
            self.wxportnumber = wx.Choice(panel, -1, (100,50), choices = portnumberlist)
        self.wxportnumber.SetStringSelection(PortNumber) 
        self.portnumber = PortNumber
        
        self.checkBoxLog = wx.CheckBox(panel, -1, "Log Trace")        
        if (self.ConfigData != None):
            DoLogTrace = eval(self.ConfigData.DoLogTrace)                               
        else:
            DoLogTrace = False               
        self.checkBoxLog.SetValue(DoLogTrace)
        # hack for HB - always set the logging box
        self.checkBoxLog.SetValue(True)
        self.checkBoxLog.Enable(False)
        if (DoLogTrace == True):
            self.logmsg = True           
        else: 
            self.logmsg = False
  
        self.cancheckbox = wx.CheckBox(panel, -1, "CAN" )
        if (self.ConfigData != None):
            DoCanTrace = eval(self.ConfigData.DoCanTrace)                               
        else:
            DoCanTrace = False               
        self.cancheckbox.SetValue(DoCanTrace)

        self.gpscheckbox = wx.CheckBox(panel, -1, "GPS" )
        if (self.ConfigData != None):
            DoGpsTrace = eval(self.ConfigData.DoGpsTrace)                               
        else:
            DoGpsTrace = False               
        self.gpscheckbox.SetValue(DoGpsTrace)
        
        inner = wx.BoxSizer(wx.HORIZONTAL)
        inner.Add(self.startBtn, 0, wx.RIGHT, 15)
        inner.Add(self.stopBtn, 0, wx.RIGHT, 15)
        inner.Add(self.checkBoxLog, 0, wx.RIGHT, 15)
        inner.Add(self.ch, 0 ,  wx.RIGHT, 15)
        inner.Add(self.tc, 0, wx.ALIGN_CENTER_VERTICAL, 15)
        inner.Add(self.cancheckbox, 0, wx.RIGHT, 15)
        inner.Add(self.wxbaudrate, 0, wx.RIGHT, 15)
        inner.Add(self.gpscheckbox, 0, wx.RIGHT, 15)
        inner.Add(self.wxportnumber, 0, wx.RIGHT, 15)
        #inner.Add(self.quickAnalyseBtn, 0, wx.RIGHT, 15)
        
        main = wx.BoxSizer(wx.VERTICAL)
        main.Add(inner, 0, wx.ALL, 5)
        
        driver = wx.BoxSizer(wx.HORIZONTAL)
        drivertitle = wx.StaticText(panel, -1, "Driver ")
        if (self.ConfigData != None):            
            Drivers = self.ConfigData.Drivers 
        else:
            Drivers = " "       
        self.textFilter = wx.TextCtrl(panel, -1, Drivers , size=(250, -1))
        self.textFilter.Enable()
        driver.Add(drivertitle, 0 , wx.ALIGN_CENTER_VERTICAL)
        driver.Add(self.textFilter, 0 , wx.RIGHT, 5)
        
        main.Add(driver, 0, wx.ALL, 8)
        
#         filter = wx.BoxSizer(wx.HORIZONTAL)
#         filtertitle = wx.StaticText(panel, -1, "Filter ")
#         if (self.ConfigData != None):            
#             GoodDeviceIds = "" 
#             for shelf in self.ConfigData.GoodDeviceIDs: 
#                 GoodDeviceIds = GoodDeviceIds + str(hex(shelf)) + ", "                                                                      
#         else:
#             GoodDeviceIds = " "       
#         self.textFilter = wx.TextCtrl(panel, -1, GoodDeviceIds , size=(250, -1))
#         self.textFilter.Disable()
#         filter.Add(filtertitle, 0 , wx.ALIGN_CENTER_VERTICAL)
#         filter.Add(self.textFilter, 0 , wx.RIGHT, 5)
#         
#         main.Add(filter, 0, wx.ALL, 8)

        main.Add(self.log, 1, wx.EXPAND|wx.ALL, 5)
        panel.SetSizer(main)

        self.Bind(wx.EVT_CHOICE, self.EvtChoice, self.ch)
        self.Bind(wx.EVT_CHOICE, self.EvtChoiceBaudRate, self.wxbaudrate)
        self.Bind(wx.EVT_CHOICE, self.EvtChoicePortNumber, self.wxportnumber)
        self.Bind(wx.EVT_CHECKBOX, self.EvtLog, self.checkBoxLog)
        self.Bind(wx.EVT_BUTTON, self.OnStartButton, self.startBtn)
        #self.Bind(wx.EVT_BUTTON, self.OnQuickAnalyseButton, self.quickAnalyseBtn)
        self.Bind(wx.EVT_BUTTON, self.OnStopButton, self.stopBtn)
        self.Bind(wx.EVT_CLOSE,  self.OnCloseWindow)
        self.Bind(wx.EVT_CHECKBOX, self.EvtChkBoxGps, self.gpscheckbox)
        self.Bind(wx.EVT_CHECKBOX, self.EvtChkBoxCan, self.cancheckbox)

        self.UpdateCount()
        FORMAT = '%(asctime)-15s %(levelname)s %(message)s'
        logging.basicConfig(filename='log/program.log',level=logging.DEBUG, format = FORMAT)
#         if ( os.getcwd() != "F:\workspace_python\DeviceTracer"):
#             dlg = wx.MessageDialog(self, "CAN Hardware " + str(pyCAN.ILLHW_Error()),
#                                    'Attention',
#                                    wx.OK | wx.ICON_INFORMATION
#                                    #wx.YES_NO | wx.NO_DEFAULT | wx.CANCEL | wx.ICON_INFORMATION
#                                )
#             dlg.ShowModal()
#             dlg.Destroy()
#             raise pyCAN.ILLHW_Error



    def OnStartButton(self, evt):
        logging.info('-'*10 + " START " + '-'*10)
        if self.DeviceThread[0] is not None:
            wx.MessageBox('You have first to stop running Thread', 'Info')
            return  
        filter = ['']
#         filter = self.textFilter.GetValue().split(',')        
#         filter = [elem.replace("0x","") for elem in filter]        
#         filter = [elem.strip() for elem in filter]
#         filter = [elem.upper() for elem in filter]        
        self.dataq = Queue.Queue()
        self.DeviceThread[0] = DeviceThreadCan(self.count, self, time.clock(), filter, self.dataq)        
        self.DeviceThread[1] = DeviceThreadGps(self.count, self, time.clock(), filter, self.dataq)        
        self.DeviceLoggingThread = DeviceLoggingThread(self.count, self, 0, filter, self.dataq)        
        if ( self.cancheckbox.GetValue() == True ):
            self.DeviceThread[0].daemon = True
            self.DeviceThread[0].start()
        if ( self.gpscheckbox.GetValue() == True ):
            self.DeviceThread[1].daemon = True
            self.DeviceThread[1].start()
        self.DeviceLoggingThread.daemon = True
        self.DeviceLoggingThread.start()
        self.textFilter.Disable()
        self.UpdateCount()
        self.log.Clear()
        self.currentDriver = self.textFilter.GetValue().strip()
        if self.currentDriver is '':
            self.currentDriver = 'Anonym'
        self.DisableAllButton()
        self.logstarted = True

    def DisableAllButton(self):
        self.cancheckbox.Enable(False)
        self.gpscheckbox.Enable(False)
        self.startBtn.Enable(False)
        self.stopBtn.Enable(True)
        self.ch.Enable(False)
        self.wxbaudrate.Enable(False)
        self.wxportnumber.Enable(False)
        self.checkBoxLog.Enable(False)
        #self.quickAnalyseBtn.Enable(False)
        self.textFilter.Enable(False)
            
    def EnableAllButton(self):
        self.cancheckbox.Enable(True)
        self.gpscheckbox.Enable(True)
        self.startBtn.Enable(True)
        self.stopBtn.Enable(False)
        self.ch.Enable(True)
        self.wxbaudrate.Enable(True)
        self.wxportnumber.Enable(True)
        #self.checkBoxLog.Enable(True)
        #self.quickAnalyseBtn.Enable(False)
        self.textFilter.Enable(True)

    def OnStopButton(self, evt):        
        self.logstarted = False
        logging.info('-'*10 + " STOP " + '-'*10)
        # the sequence of stopping is important !!
        if self.DeviceThread[0] is not None: 
            self.DeviceThread[0].stop()
        if self.DeviceThread[1] is not None: 
            self.DeviceThread[1].stop()
        if self.DeviceLoggingThread is not None:
            self.DeviceLoggingThread.stop()
        self.DeviceThread = [None, None]
        self.DeviceLoggingThread = None
        self.dataq = None
        self.UpdateCount()
        #self.logmsg = False
#         while(True):
#             try:
#                 msg = self.dataq.get_nowait() # flush everything
#                 logging.info("some thing left in the queue, flushing " + msg)
#             except Queue.Empty:
#                 break
        self.EnableAllButton()

    def OnQuickAnalyseButton(self, evt):
        print "analysing"
        filepath_open = None 
        Wildcard = "*.txt"
        txt = "Open File"
        prePath = os.getcwd()
        print prePath
        curdir = os.getcwd()
#        sys.path.append(os.path.abspath('../tools/eeprom_Script'))
        dlg = wx.FileDialog(self, message=txt, defaultDir=curdir,
            defaultFile="",wildcard=Wildcard,style=wx.OPEN|wx.CHANGE_DIR)
        if dlg.ShowModal() == wx.ID_OK:
            paths = dlg.GetPaths()
            paths2 = dlg.GetDirectory()
            dlg.Destroy()
            print "input from Dialog Box: ", paths[0]
            #import DeviceDataAnalyse
            #DeviceDataAnalyse.runAnalysis(paths[0])
            os.chdir(prePath)
#            UHVClassObjectConversionTextPrompts = parse_eeprom_excel_layout.UHVEepromDataConversion()
            #paths, paths2, curdir
#            UHVClassObjectConversionTextPrompts.runScript(paths[0], paths2, curdir)
        else:
            print "Aborted"

    
    def OnCloseWindow(self, evt):
        if (self.IsMaximized()== False):
            self.FrameSize[0], self.FrameSize[1] = self.GetSize()
            self.ConfigData.Set("WINDOW_INFO", "frame_size", self.FrameSize)             
        self.ConfigData.Set("WINDOW_INFO","ISMAXIMIZED",self.IsMaximized())  
        #ListFilterDeviceMsg = self.textFilter.GetValue().strip()
        #ListFilterDeviceMsg = "["+ str(ListFilterDeviceMsg.strip(",")) + "]" 
        
        DriverMsg = self.textFilter.GetValue().strip()
        
        self.ConfigData.Set("TRACE_INFO", "logfileformat", self.ch.GetStringSelection())
        self.ConfigData.Set("TRACE_INFO", "do_logtrace", self.checkBoxLog.GetValue())
        self.ConfigData.Set("TRACE_INFO", "do_cantrace", self.cancheckbox.GetValue())
        self.ConfigData.Set("TRACE_INFO", "do_gpstrace", self.gpscheckbox.GetValue())
#         if (len(ListFilterDeviceMsg) == 2): # empty array set default for example
#             pass
        #self.ConfigData.Set("DEVICE_INFO", "gooddeviceids", ListFilterDeviceMsg)
        self.ConfigData.Set("DEVICE_INFO", "baudrate", self.wxbaudrate.GetStringSelection())
        self.ConfigData.Set("DEVICE_INFO", "portnumber", self.wxportnumber.GetStringSelection())

        self.ConfigData.Set("DRIVER_INFO", "drivers", DriverMsg)
        
        self.ConfigData.WriteConfigFileData()
        self.StopThreads()        
        self.Destroy()

    def StopThreads(self):       
        if  not self.DeviceThread[0] is None: 
            self.DeviceThread[0].stop()
            self.DeviceThread[1].stop()
            self.DeviceLoggingThread.stop()
            self.DeviceThread = [None, None]
            self.DeviceLoggingThread = None
            #self.logmsg = False
            self.logstarted = False

    def UpdateCount(self):
        self.tc.SetLabel("DEVICE Threads: \n%s\n%s\n%s" % (self.DeviceThread[0], self.DeviceThread[1], self.DeviceLoggingThread))

    def LogMessage(self, msg):
        self.stringcount = self.stringcount + len(msg)
        if self.stringcount >= 20000:
            self.log.Remove(0,10000)
            self.stringcount = self.stringcount - 10000
            
        self.log.AppendText(msg)
            
    def getLogStatus(self):
        return self.logmsg
        
    def EvtLog(self, event):
        if not self.logmsg and not self.format:
            dlg = wx.MessageDialog(self, 'You must choose a format for the log file in the dropbox.',
                                   'Attention',
                                   wx.OK | wx.ICON_INFORMATION
                                   #wx.YES_NO | wx.NO_DEFAULT | wx.CANCEL | wx.ICON_INFORMATION
                               )
            dlg.ShowModal()
            dlg.Destroy()
            self.checkBoxLog.SetValue(False)
        else:
            pass
                       
            self.logmsg = not self.logmsg
        
    def EvtChoice(self, event):
        if (self.logstarted == True ):
            dlg = wx.MessageDialog(self, 'You must stop first the Device-Thread before you can change the format.',
                                   'Attention',
                                   wx.OK | wx.ICON_INFORMATION
                                   #wx.YES_NO | wx.NO_DEFAULT | wx.CANCEL | wx.ICON_INFORMATION
                               )
            dlg.ShowModal()
            dlg.Destroy()
        else:
            self.format = event.GetString()

    def EvtChkBoxGps(self, event):
        if (self.logstarted == True ):
            dlg = wx.MessageDialog(self, 'You must stop first the Device-Thread before you can change the format.',
                                   'Attention',
                                   wx.OK | wx.ICON_INFORMATION
                                   #wx.YES_NO | wx.NO_DEFAULT | wx.CANCEL | wx.ICON_INFORMATION
                               )
            dlg.ShowModal()
            dlg.Destroy()
        else:
            logging.info("gps check box has been clicked to " + str(self.gpscheckbox.GetValue()))
            if (self.gpscheckbox.GetValue() == True):
                self.wxportnumber.Clear()
                list_of_serial_ports = pySERIAL.SerialDriver().getSerialPortList()
                for portname in list_of_serial_ports:
                    self.wxportnumber.Append(portname)
                self.wxportnumber.SetStringSelection(self.ConfigData.PortNumber)

    def EvtChkBoxCan(self, event):
        if (self.logstarted == True ):
            dlg = wx.MessageDialog(self, 'You must stop first the Device-Thread before you can change the format.',
                                   'Attention',
                                   wx.OK | wx.ICON_INFORMATION
                                   #wx.YES_NO | wx.NO_DEFAULT | wx.CANCEL | wx.ICON_INFORMATION
                               )
            dlg.ShowModal()
            dlg.Destroy()
        else:
            logging.info("can check box has been clicked to " + str(self.cancheckbox.GetValue()))
            if (self.cancheckbox.GetValue() == True):
                self.wxbaudrate.SetStringSelection(self.ConfigData.BaudRate)
        
    def EvtChoiceBaudRate(self, event):
        if (self.logstarted == True ):
            dlg = wx.MessageDialog(self, 'You must stop first the Device-Thread before you can change the format.',
                                   'Attention',
                                   wx.OK | wx.ICON_INFORMATION
                                   #wx.YES_NO | wx.NO_DEFAULT | wx.CANCEL | wx.ICON_INFORMATION
                               )
            dlg.ShowModal()
            dlg.Destroy()
        else:
            self.baudrate = event.GetString()
    
    def EvtChoicePortNumber(self, event):
        if (self.logstarted == True ):
            dlg = wx.MessageDialog(self, 'You must stop first the Device-Thread before you can change the format.',
                                   'Attention',
                                   wx.OK | wx.ICON_INFORMATION
                                   #wx.YES_NO | wx.NO_DEFAULT | wx.CANCEL | wx.ICON_INFORMATION
                               )
            dlg.ShowModal()
            dlg.Destroy()
        else:
            self.portnumber = event.GetString()
    
    def getFileFormat(self):
        return self.format
    
    
def Start(ConfigData=None):
    app = wx.PySimpleApp()
    frm = MyFrame(ConfigData)
    frm.Show()
    app.MainLoop()

# ###########
# Main part
# ###########
if __name__ == "__main__": 
    Start()         
