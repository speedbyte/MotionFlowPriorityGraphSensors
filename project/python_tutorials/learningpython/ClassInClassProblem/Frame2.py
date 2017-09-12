#Boa:Frame:Frame2

import wx

def create(parent):
    return Frame2(parent)

[wxID_FRAME2, wxID_FRAME2BUTTON1, wxID_FRAME2TEXTCTRL1, 
] = [wx.NewId() for _init_ctrls in range(3)]

class Frame2(wx.Frame):
    def _init_ctrls(self, prnt):
        # generated method, don't edit
        wx.Frame.__init__(self, id=wxID_FRAME2, name='', parent=prnt,
              pos=wx.Point(508, 297), size=wx.Size(400, 250),
              style=wx.DEFAULT_FRAME_STYLE, title='Frame2')
        self.SetClientSize(wx.Size(392, 216))

        self.button1 = wx.Button(id=wxID_FRAME2BUTTON1, label='button1',
              name='button1', parent=self, pos=wx.Point(144, 128),
              size=wx.Size(75, 23), style=0)
        self.button1.Bind(wx.EVT_BUTTON, self.OnButton1Button,
              id=wxID_FRAME2BUTTON1)

        self.textCtrl1 = wx.TextCtrl(id=wxID_FRAME2TEXTCTRL1, name='textCtrl1',
              parent=self, pos=wx.Point(104, 24), size=wx.Size(148, 32),
              style=0, value='textCtrl1')

    def __init__(self, parent):
        self._init_ctrls(parent)
        self.parentFrame1 = parent

    def OnButton1Button(self, event):
        self.parentFrame1.RefreshStaticText()
