#Boa:Frame:SyncFrame

import wx

def create(parent):
    return SyncFrame(parent)

[wxID_SYNCFRAME, wxID_SYNCFRAMESTATICTEXT1, wxID_SYNCFRAMESYNCBUTTON1, 
 wxID_SYNCFRAMESYNCCOMMANDNAMEINPUT, wxID_SYNCFRAMESYNCSTATICTEXTCOMMAND, 
 wxID_SYNCFRAMESYNCSTATICTEXTRESPONSE, wxID_SYNCFRAMESYNCSTATICTEXTSTATUS, 
 wxID_SYNCFRAMESYNCSTATUSBAR1, 
] = [wx.NewId() for _init_ctrls in range(8)]

class SyncFrame(wx.Frame):
    def _init_coll_SyncStatusBar1_Fields(self, parent):
        # generated method, don't edit
        parent.SetFieldsCount(1)

        parent.SetStatusText(number=0, text='status')

        parent.SetStatusWidths([-1])

    def _init_ctrls(self, prnt):
        # generated method, don't edit
        wx.Frame.__init__(self, id=wxID_SYNCFRAME, name='SyncFrame',
              parent=prnt, pos=wx.Point(564, 301), size=wx.Size(590, 560),
              style=wx.DEFAULT_FRAME_STYLE, title='SyncFrame')
        self.SetClientSize(wx.Size(582, 522))
        self.SetStatusBarPane(1)

        self.SyncStatusBar1 = wx.StatusBar(id=wxID_SYNCFRAMESYNCSTATUSBAR1,
              name='SyncStatusBar1', parent=self, style=0)
        self._init_coll_SyncStatusBar1_Fields(self.SyncStatusBar1)
        self.SetStatusBar(self.SyncStatusBar1)

        self.SyncButton1 = wx.Button(id=wxID_SYNCFRAMESYNCBUTTON1,
              label='Emi Test', name='SyncButton1', parent=self,
              pos=wx.Point(24, 104), size=wx.Size(96, 32), style=0)
        self.SyncButton1.Bind(wx.EVT_BUTTON, self.OnSyncButton1Button,
              id=wxID_SYNCFRAMESYNCBUTTON1)

        self.SyncStaticTextCommand = wx.StaticText(id=wxID_SYNCFRAMESYNCSTATICTEXTCOMMAND,
              label='Command', name='SyncStaticTextCommand', parent=self,
              pos=wx.Point(136, 88), size=wx.Size(80, 40),
              style=wx.RAISED_BORDER | wx.ALIGN_CENTRE)

        self.SyncStaticTextResponse = wx.StaticText(id=wxID_SYNCFRAMESYNCSTATICTEXTRESPONSE,
              label='Response', name='SyncStaticTextResponse', parent=self,
              pos=wx.Point(136, 152), size=wx.Size(80, 40),
              style=wx.RAISED_BORDER | wx.ALIGN_CENTRE)

        self.SyncStaticTextStatus = wx.StaticText(id=wxID_SYNCFRAMESYNCSTATICTEXTSTATUS,
              label='Status', name='SyncStaticTextStatus', parent=self,
              pos=wx.Point(136, 216), size=wx.Size(80, 40),
              style=wx.RAISED_BORDER | wx.ALIGN_CENTRE)

        self.staticText1 = wx.StaticText(id=wxID_SYNCFRAMESTATICTEXT1,
              label='Command name', name='staticText1', parent=self,
              pos=wx.Point(136, 24), size=wx.Size(80, 40),
              style=wx.RAISED_BORDER | wx.ALIGN_CENTRE)

        self.SyncCommandNameInput = wx.StaticText(id=wxID_SYNCFRAMESYNCCOMMANDNAMEINPUT,
              label='Automatically generated', name='SyncCommandNameInput',
              parent=self, pos=wx.Point(272, 24), size=wx.Size(200, 40),
              style=wx.RAISED_BORDER | wx.ALIGN_CENTRE)
        self.SyncCommandNameInput.Bind(wx.EVT_CHAR,
              self.OnSyncCommandNameInputChar)

    def __init__(self, parent):
        self._init_ctrls(parent)

    def OnSyncButton1Button(self, event):
        print 'add Eol app code here'

    def OnSyncCommandNameInputChar(self, event):
        event.Skip()


if __name__ == '__main__':
    app = wx.PySimpleApp()
    frame = create(None)
    frame.Show()

    app.MainLoop()
