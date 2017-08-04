#Boa:Dialog:Dialog1

import wx

def create(parent):
    return Dialog1(parent)

[wxID_DIALOG1, wxID_DIALOG1BUTTON1, wxID_DIALOG1STATICBITMAP1, 
 wxID_DIALOG1STATICTEXT1, wxID_DIALOG1STATICTEXT2, 
] = [wx.NewId() for _init_ctrls in range(5)]

class Dialog1(wx.Dialog):
    def _init_ctrls(self, prnt):
        # generated method, don't edit
        wx.Dialog.__init__(self, id=wxID_DIALOG1, name='', parent=prnt,
              pos=wx.Point(468, 281), size=wx.Size(623, 639),
              style=wx.DEFAULT_DIALOG_STYLE, title='About Notebook')
        self.SetClientSize(wx.Size(615, 601))

        self.staticText1 = wx.StaticText(id=wxID_DIALOG1STATICTEXT1,
              label='Notebook - Simple Text Editor', name='staticText1',
              parent=self, pos=wx.Point(192, 0), size=wx.Size(215, 19),
              style=wx.ALIGN_CENTRE)
        self.staticText1.SetFont(wx.Font(12, wx.SWISS, wx.NORMAL, wx.NORMAL,
              False, 'Tahoma'))

        self.staticText2 = wx.StaticText(id=wxID_DIALOG1STATICTEXT2,
              label='This is my first Boa Constructor Application',
              name='staticText2', parent=self, pos=wx.Point(136, 32),
              size=wx.Size(356, 23), style=0)
        self.staticText2.SetBackgroundColour(wx.Colour(211, 205, 248))
        self.staticText2.SetFont(wx.Font(14, wx.SWISS, wx.NORMAL, wx.NORMAL,
              False, 'Tahoma'))

        self.staticBitmap1 = wx.StaticBitmap(bitmap=wx.Bitmap('F:/Work/S1nn_Tools/BoaTry/bd.bmp',
              wx.BITMAP_TYPE_BMP), id=wxID_DIALOG1STATICBITMAP1,
              name='staticBitmap1', parent=self, pos=wx.Point(152, 80),
              size=wx.Size(295, 457), style=0)

        self.button1 = wx.Button(id=wxID_DIALOG1BUTTON1, label='Close',
              name='button1', parent=self, pos=wx.Point(256, 560),
              size=wx.Size(75, 23), style=0)
        self.button1.Bind(wx.EVT_BUTTON, self.OnButton1Button,
              id=wxID_DIALOG1BUTTON1)

    def __init__(self, parent):
        self._init_ctrls(parent)

    def OnButton1Button(self, event):
        self.Close()
