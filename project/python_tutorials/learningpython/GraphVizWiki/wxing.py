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
from threading import Thread
import time


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

        # Create an instance of our customized Frame class
        frame = MyFrame(None, -1, "This is a test")
        frame.Show(True)

        # Tell wxWindows that this is our main window
        self.SetTopWindow(frame)
        cyclicPrinting = CyclicPrint()
        cyclicPrinting.start()
        # Return a success flag
        return True


class CyclicPrint(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.count = 1
    
    def run(self):
        while(1):
            time.sleep(2)
            self.count = self.count + 1
            print self.count

if __name__=='__main__':
    
    app = MyApp(0)     # Create an instance of the application class
    app.MainLoop()     # Tell it to start processing events






