#Boa:Frame:Frame1

import wx
import Frame2

def create(parent):
    return Frame1(parent)

[wxID_FRAME1, wxID_FRAME1BUTTON1, wxID_FRAME1STATICTEXT1, 
] = [wx.NewId() for _init_ctrls in range(3)]

class Frame1(wx.Frame):
    def _init_ctrls(self, prnt):
        # generated method, don't edit
        wx.Frame.__init__(self, id=wxID_FRAME1, name='', parent=prnt,
              pos=wx.Point(442, 305), size=wx.Size(400, 250),
              style=wx.DEFAULT_FRAME_STYLE, title='Frame1')
        self.SetClientSize(wx.Size(392, 216))

        self.button1 = wx.Button(id=wxID_FRAME1BUTTON1, label='button1',
              name='button1', parent=self, pos=wx.Point(152, 144),
              size=wx.Size(75, 23), style=0)
        self.button1.Bind(wx.EVT_BUTTON, self.OnButton1Button,
              id=wxID_FRAME1BUTTON1)

        self.staticText1 = wx.StaticText(id=wxID_FRAME1STATICTEXT1,
              label='staticText1', name='staticText1', parent=self,
              pos=wx.Point(160, 24), size=wx.Size(55, 48), style=0)

    def __init__(self, parent):
        self._init_ctrls(parent)
        self.parentInstance = parent
        print parent

    def OnButton1Button(self, event):
        self.staticText1.SetLabel("Setting label from Frame 1")
        Frame2Instance = Frame2.create(self)
        Frame2Instance.Show()
        
    def RefreshStaticText(self):
        self.staticText1.SetLabel('Refreshing')
