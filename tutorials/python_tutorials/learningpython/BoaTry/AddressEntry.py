#-----------------------------------------------------------------------------
# Name:        Frame1.py
# Purpose:     
#
# Author:      <your name>
#
# Created:     2008/05/08
# RCS-ID:      $Id: Frame1.py $
# Copyright:   (c) 2006
# Licence:     <your licence>
#-----------------------------------------------------------------------------
#Boa:Frame:AddressEntry

import wx

def create(parent):
    return AddressEntry(parent)

[wxID_ADDRESSENTRY, wxID_ADDRESSENTRYLISTCTRL1, wxID_ADDRESSENTRYPANEL1, 
 wxID_ADDRESSENTRYSTATICTEXT1, wxID_ADDRESSENTRYSTATICTEXT2, 
 wxID_ADDRESSENTRYTEXTCTRL1, 
] = [wx.NewId() for _init_ctrls in range(6)]

class AddressEntry(wx.Frame):
    def _init_coll_fgsFields_Items(self, parent):
        # generated method, don't edit

        parent.AddWindow(None, 0, border=0, flag=0)
        parent.AddWindow(None, 0, border=0, flag=0)
        parent.AddWindow(None, 0, border=0, flag=0)
        parent.AddWindow(None, 0, border=0, flag=0)
        parent.AddWindow(None, 0, border=0, flag=0)
        parent.AddWindow(None, 0, border=0, flag=0)
        parent.AddWindow(None, 0, border=0, flag=0)
        parent.AddWindow(None, 0, border=0, flag=0)
        parent.AddWindow(None, 0, border=0, flag=0)
        parent.AddWindow(None, 0, border=0, flag=0)

    def _init_coll_bsMain_Items(self, parent):
        # generated method, don't edit

        parent.AddWindow(self.listCtrl1, 1, border=2, flag=wx.EXPAND | wx.ALL)
        parent.AddSizer(self.fgsFields, 0, border=0, flag=0)
        parent.AddWindow(None, 0, border=0, flag=0)
        parent.AddWindow(self.textCtrl1, 0, border=0, flag=0)
        parent.AddWindow(self.staticText2, 0, border=0, flag=0)

    def _init_coll_listCtrl1_Columns(self, parent):
        # generated method, don't edit

        parent.InsertColumn(col=0, format=wx.LIST_FORMAT_LEFT,
              heading='First name', width=-1)
        parent.InsertColumn(col=1, format=wx.LIST_FORMAT_LEFT,
              heading='Last Name', width=-1)
        parent.InsertColumn(col=2, format=wx.LIST_FORMAT_LEFT, heading='City',
              width=-1)
        parent.InsertColumn(col=3, format=wx.LIST_FORMAT_LEFT,
              heading='Country', width=-1)

    def _init_sizers(self):
        # generated method, don't edit
        self.bsMain = wx.BoxSizer(orient=wx.VERTICAL)

        self.fgsFields = wx.FlexGridSizer(cols=2, hgap=0, rows=0, vgap=0)

        self._init_coll_bsMain_Items(self.bsMain)
        self._init_coll_fgsFields_Items(self.fgsFields)

        self.panel1.SetSizer(self.bsMain)

    def _init_ctrls(self, prnt):
        # generated method, don't edit
        wx.Frame.__init__(self, id=wxID_ADDRESSENTRY, name='AddressEntry',
              parent=prnt, pos=wx.Point(476, 388), size=wx.Size(400, 250),
              style=wx.DEFAULT_FRAME_STYLE, title='Address Entry Form')
        self.SetClientSize(wx.Size(392, 212))

        self.panel1 = wx.Panel(id=wxID_ADDRESSENTRYPANEL1, name='panel1',
              parent=self, pos=wx.Point(0, 0), size=wx.Size(392, 212),
              style=wx.TAB_TRAVERSAL)

        self.listCtrl1 = wx.ListCtrl(id=wxID_ADDRESSENTRYLISTCTRL1,
              name='listCtrl1', parent=self.panel1, pos=wx.Point(2, 2),
              size=wx.Size(388, 30), style=wx.LC_REPORT)
        self._init_coll_listCtrl1_Columns(self.listCtrl1)

        self.staticText1 = wx.StaticText(id=wxID_ADDRESSENTRYSTATICTEXT1,
              label='staticText1', name='staticText1', parent=self.panel1,
              pos=wx.Point(0, 141), size=wx.Size(55, 13), style=0)

        self.textCtrl1 = wx.TextCtrl(id=wxID_ADDRESSENTRYTEXTCTRL1,
              name='textCtrl1', parent=self.panel1, pos=wx.Point(0, 178),
              size=wx.Size(100, 21), style=0, value='textCtrl1')

        self.staticText2 = wx.StaticText(id=wxID_ADDRESSENTRYSTATICTEXT2,
              label='staticText2', name='staticText2', parent=self.panel1,
              pos=wx.Point(0, 199), size=wx.Size(55, 13), style=0)

        self._init_sizers()

    def __init__(self, parent):
        self._init_ctrls(parent)


if __name__ == '__main__':
    app = wx.PySimpleApp()
    frame = create(None)
    frame.Show()

    app.MainLoop()
