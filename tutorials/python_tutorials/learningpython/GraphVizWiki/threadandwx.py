""" 

    $Date: 2008-02-12  $
    $Rev: 2588 $
    $Author: agrawal $
    $URL: svn://voice/SyncEol/EolApp.py $

    Copyright (c) 2008 S1nn GmbH & Co. KG.
    All Rights Reserved.

"""



import SendReceive
import CreatePayload
import AnalyseResponse
import pyCAN
import time
#from CanDispatcher import *
import Queue

import time
from threading import Thread

cd = pyCAN.CanDriver()

routineStatus_dict = {
'active' : 0x02,
'completed' : 0x0,
'aborted' : 0x1
}
responseStatus_dict = {
'negative_notSupported' : 0x7F,
'negative_responsePending' : 0x78,
'negative_ECUbusy' : 0x21,
'positive' : 0x71
}
routineType_dict = {
'quickrun': 0x10,
'timeRun' : 0x20,
'tillstopRun' : 0x30
}
messageseq = [
'StartEol',
'StartEolResult',
'SetEOLCCPUPin',
'SetEolCCPUPinResult',
'ColdRebootCCPUStart',
'ColdRebootCCPUResult',
#'UsbPenDriveTest',
#'UsbPenDriveResult',
#'AudioCodecReset',
#'AudioOutResult',
#'AudioOutStop'
#'AudioOutStart',
#'AudioOutResult',
#'TurnOnRamTestOnEolBootUpStart' ,
#'TurnOnRamTestOnEolBootUpResult',
#'GetRamIntegrityResultsStart',
#'GetRamIntegrityResultsResult'
'EmiRfTestStart',
#'EmiRfTestResult',
]

class EOLTester:
    def __init__(self):
      
      # Setup CAN send and receive thread
      self.FilterThread = None
      self.Dispatcher = CanDispatcher()
      self.Dispatcher.registerCallBackFct(self.rcvdMessage)
      self.Dispatcher.start()
      self.OutQueue = Queue.Queue()      
      self.InQueue = None
      
#      self.FilterThread = CanFilterThread()
#      filter = CanFilter()
#      filter.addCanID(0x7D8)
#      filter.addCanID(0x5C1)
#      #filter.addCanID(0x3EB)
#      self.FilterThread.addFilter(filter)      
#      self.Dispatcher.setRxQueue(self.FilterThread.getRxQueue())
#      self.OutQueue = self.FilterThread.getTxQueue()
#      self.FilterThread.start()
      
      # Create Thread for sending cyclic messages
      self.cyclicSender = CyclicSend()
      
      # Configure cyclic messages to send
      self.cyclicSender.setTxQueue(self.OutQueue)
    
      # Start sender
      self.cyclicSender.start()

    def setInQueue(self, queue):
        self.InQueue = queue
        
    def sendMsg(self, msg):
        self.OutQueue.put(msg)
        
    def rcvdMessage(self,msg):
        if self.InQueue is not None:
            print "received Msg ID: 0x%03x" % msg.ID
            self.InQueue.put(msg)
    
    def StopThread(self):
        self.cyclicSender.stopSending()

class CyclicSend(Thread):
    def __init__ (self):
      Thread.__init__(self)
      self.stopIt = False
   
    def setTxQueue(self, queue):
      self.txQueue = queue

    def stopSending(self):
        self.stopIt = True
    
    def sendCycMessage(self,msg):
        self.txQueue.put(msg)

    def run(self):
      while not (self.stopIt):
          time.sleep(1) 
          msg1 = pyCAN.makeCanMsg(0x7D0, [0x2,0x3E,0,0,0,0,0,0])
          msg2 = pyCAN.makeCanMsg(0x3E8, [0,0,0,0x4,0,0,0,0])
          cd.write(msg1)
          time.sleep(0.010)
          cd.write(msg2)          
            #Sending CAN messages now
#        while not (self.TxQueue.empty()):
#            try:
#                outMsg = self.TxQueue.get(False)
#            except Queue.Empty:
#                #Send queue is now empty, continue
#                print 'CAN Send queue empty'
#                pass                  
#            else:
#                try:
#                    #Put msg on CAN bus
#                    self.canDriver.write(outMsg)
#                except pyCAN.BUSOFF_Error:
#                    self.canDriver.resetClient()
def SendReceiveMain():
    #===============================================================================
    # #initialise objects
    
    cp = CreatePayload.CreatePayload()  # create Payload
    sr = SendReceive.SendReceive()      # send on Bus and get the response back
    ar = AnalyseResponse.AnalyseResponse()  # analyze the response
    #===============================================================================
    
    # create requests
    
    FlowControlMessage = cp.createFlowControlMessage()
    

    cd.open()
    while 1:
        time.sleep(1)
        for msgs in messageseq:
        # open can driver
            print "EolApp:" , msgs
            time.sleep(4)
            
            EolMessage = cp.createPayloadAndSegment(msgs)
            print "EolApp:" ,EolMessage
        
            #send request and receive response
            response = sr.sendReceiveProtocol(EolMessage, FlowControlMessage, cd)
            if response == [1]:
                #Close The peak driver
                return
            
            # read response
            responseZussamen = ar.readEolResponse(response)
            responseID = responseZussamen[0]
            testStatus = responseZussamen[1]
            
            # loops if the message sent is Result
            if 'Result' in msgs:
                while (testStatus & 0x7) == routineStatus_dict['active']:
                    EolMessage = cp.createPayloadAndSegment(msgs)
                    response = sr.sendReceiveProtocol(EolMessage, FlowControlMessage, cd)
                    if response == [1]:
                        return
                    # read response
                    responseZussamen = ar.readEolResponse(response)
                    responseID = responseZussamen[0]
                    testStatus = responseZussamen[1]
                    if ((testStatus & 0x7) == routineStatus_dict['active']):
                        time.sleep(30)
            if responseID == responseStatus_dict['positive']:
                print " positive response"
            else:
                print " negative response"
                return
        timer = 10000        
        while 1:
            try:                 
                time.sleep(0.010)
                result=cd.read() 
            except pyCAN.QRCVEMPTY_Error:
                timer = timer - 1
                if timer == 0:
                    print "Closing due to Timer"
                    return
                continue
            except pyCAN.UNKNOWN_Error:
                continue
            if result.ID == 0x5C1:
                print "there was a reset and closing"
                return
            timer = 10000


#!/usr/bin/env python
#----------------------------------------------------------------------------
# Name:         test7.py
# Purpose:      A minimal wxPython test program
#
# Author:       Robin Dunn
#
# Created:      A long time ago, in a galaxy far, far away...
# Copyright:    (c) 1998 by Total Control Software
# Licence:      wxWidgets license
#----------------------------------------------------------------------------

# NOTE: this sample requires wxPython 2.6 or newer

# import the wxPython GUI package
import wx


# Create a new frame class, derived from the wxPython Frame.
class MyFrame(wx.Frame):

    def __init__(self, parent, id, title):
        # First, call the base class' __init__ method to create the frame
        wx.Frame.__init__(self, parent, id, title)

        # Associate some events with methods of this class
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.Bind(wx.EVT_MOVE, self.OnMove)

        # Add a panel and some controls to display the size and position
        panel = wx.Panel(self, -1)
        # unten steht : parent, id, pos, size, style, validator, name
        label1 = wx.StaticText(panel, -1, "Size:")
        label2 = wx.StaticText(panel, -1, "Pos:")
        self.sizeCtrl = wx.TextCtrl(panel, -1, "", style=wx.TE_READONLY)
        self.posCtrl = wx.TextCtrl(panel, -1, "", style=wx.TE_READONLY)
        self.panel = panel

#       Use some sizers for layout of the widgets
#       self, rows, cols, vgap, hgap
#        
#                Col1            Col2
#        Row 1   size   vgap    text
#                   hgap
#                   
#        Row 2   pos    vgap    text
        sizer = wx.FlexGridSizer(2, 2, 5, 5)
        # in the proper order, please do not change the order of the code below.
        sizer.Add(label1)
        sizer.Add(self.sizeCtrl)
        sizer.Add(label2)
        sizer.Add(self.posCtrl)

        border = wx.BoxSizer()
        border.Add(sizer, 0, wx.ALL, 5)
        panel.SetSizerAndFit(border)
        self.Fit()


    # This method is called by the System when the window is resized,
    # because of the association above.
    def OnSize(self, event):
        size = event.GetSize()
        self.sizeCtrl.SetValue("%s, %s" % (size.width, size.height))

        # tell the event system to continue looking for an event handler,
        # so the default handler will get called.
        event.Skip()

    # This method is called by the System when the window is moved,
    # because of the association above.
    def OnMove(self, event):
        pos = event.GetPosition()
        self.posCtrl.SetValue("%s, %s" % (pos.x, pos.y))




# Every wxWidgets application must have a class derived from wx.App
class MyApp(wx.App):

    # wxWindows calls this method to initialize the application
    def OnInit(self):

        time.sleep(2)
        cyclicSender = CyclicSend()
        cyclicSender.start()
        SendReceiveMain()
        cyclicSender.stopSending()
        time.sleep(3)
        cd.close()
        cyclicSender = 1 # this would destroy the instance created for CyclicSender
        print " eolApp there was a problem in the CAN Driver.. restarting in 10 seconds"
        time.sleep(10)
        # Return a success flag
        #Create an instance of our customized Frame class
        frame = MyFrame(None, -1, "This is a test")
        frame.Show(True)

        # Tell wxWindows that this is our main window
#        self.SetTopWindow(frame)
        return True
    

 

if __name__ == '__main__':

    time.sleep(2)
    
    while 1:
        self.app = MyApp(0)     # Create an instance of the application class
        self.app.MainLoop()
        

