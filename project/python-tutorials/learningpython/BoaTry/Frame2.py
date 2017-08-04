#Boa:Frame:Frame2

import wx
import Dialog1

def create(parent):
    print parent
    return Frame2(parent)

[wxID_FRAME2, wxID_FRAME2STATUSBAR1, wxID_FRAME2TEXTEDITOR, 
] = [wx.NewId() for _init_ctrls in range(3)]

[wxID_FRAME2MENUFILECLOSE, wxID_FRAME2MENUFILEEXIT, wxID_FRAME2MENUFILEOPEN, 
 wxID_FRAME2MENUFILESAVE, wxID_FRAME2MENUFILESAVEAS, 
] = [wx.NewId() for _init_coll_menuFile_Items in range(5)]

[wxID_FRAME2MENUHELPABOUT] = [wx.NewId() for _init_coll_menuHelp_Items in range(1)]

class Frame2(wx.Frame):
    def _init_coll_menuBar1_Menus(self, parent):
        # generated method, don't edit

        parent.Append(menu=self.menuFile, title='File')
        parent.Append(menu=self.menuHelp, title='Help')

    def _init_coll_menuHelp_Items(self, parent):
        # generated method, don't edit

        parent.Append(help='Display General information about 1st Edit',
              id=wxID_FRAME2MENUHELPABOUT, kind=wx.ITEM_NORMAL, text='About')
        self.Bind(wx.EVT_MENU, self.OnMenuHelpAboutMenu,
              id=wxID_FRAME2MENUHELPABOUT)

    def _init_coll_menuFile_Items(self, parent):
        # generated method, don't edit

        parent.Append(help='New File', id=wxID_FRAME2MENUFILEOPEN,
              kind=wx.ITEM_NORMAL, text='Open')
        parent.Append(help='', id=wxID_FRAME2MENUFILESAVE, kind=wx.ITEM_NORMAL,
              text='Save')
        parent.Append(help='', id=wxID_FRAME2MENUFILESAVEAS,
              kind=wx.ITEM_NORMAL, text='SaveAs')
        parent.Append(help='', id=wxID_FRAME2MENUFILECLOSE, kind=wx.ITEM_NORMAL,
              text='Close')
        parent.Append(help='', id=wxID_FRAME2MENUFILEEXIT, kind=wx.ITEM_NORMAL,
              text='Exit')
        self.Bind(wx.EVT_MENU, self.OnMenuFileCloseMenu,
              id=wxID_FRAME2MENUFILECLOSE)
        self.Bind(wx.EVT_MENU, self.OnMenuFileOpenMenu,
              id=wxID_FRAME2MENUFILEOPEN)
        self.Bind(wx.EVT_MENU, self.OnMenuFileSaveMenu,
              id=wxID_FRAME2MENUFILESAVE)
        self.Bind(wx.EVT_MENU, self.OnMenuFileSaveasMenu,
              id=wxID_FRAME2MENUFILESAVEAS)
        self.Bind(wx.EVT_MENU, self.OnMenuFileExitMenu,
              id=wxID_FRAME2MENUFILEEXIT)

    def _init_coll_statusBar1_Fields(self, parent):
        # generated method, don't edit
        parent.SetFieldsCount(1)

        parent.SetStatusText(number=0, text='status')

        parent.SetStatusWidths([-1])

    def _init_utils(self):
        # generated method, don't edit
        self.menuFile = wx.Menu(title='File')

        self.menuHelp = wx.Menu(title='Help')

        self.menuBar1 = wx.MenuBar()

        self._init_coll_menuFile_Items(self.menuFile)
        self._init_coll_menuHelp_Items(self.menuHelp)
        self._init_coll_menuBar1_Menus(self.menuBar1)

    def _init_ctrls(self, prnt):
        # generated method, don't edit
        wx.Frame.__init__(self, id=wxID_FRAME2, name='', parent=prnt,
              pos=wx.Point(789, 368), size=wx.Size(400, 250),
              style=wx.DEFAULT_FRAME_STYLE, title='1stEdit')
        self._init_utils()
        self.SetClientSize(wx.Size(392, 212))
        self.SetStatusBarPane(0)
        self.SetMenuBar(self.menuBar1)

        self.statusBar1 = wx.StatusBar(id=wxID_FRAME2STATUSBAR1,
              name='statusBar1', parent=self, style=0)
        self._init_coll_statusBar1_Fields(self.statusBar1)
        self.SetStatusBar(self.statusBar1)

        self.textEditor = wx.TextCtrl(id=wxID_FRAME2TEXTEDITOR,
              name='textEditor', parent=self, pos=wx.Point(0, 0),
              size=wx.Size(392, 169), style=wx.TE_MULTILINE, value='')

    def __init__(self, parent):
        self._init_ctrls(parent)
        self.FileName = None

    def OnMenuFileCloseMenu(self, event):
        self.FileName = None
        self.textEditor.Clear()
        self.setTitle('Notebook')

    def OnMenuFileOpenMenu(self, event):
        dlg = wx.FileDialog(self, 'Choose a file', '.', '', '*.*', wx.OPEN)
        try:
            if dlg.ShowModal() == wx.ID_OK:
                filename = dlg.GetPath()
                # Your code
                self.textEditor.LoadFile(filename) 
                self.FileName=filename
                self.SetTitle(('Notebook - %s') % filename)    
        finally:
            dlg.Destroy()

    def OnMenuFileSaveMenu(self, event):
        if self.FileName == None:
            return self.OnMenuFileSaveasMenu(event)
        else:
            self.textEditor.SaveFile(self.FileName)

    def OnMenuFileSaveasMenu(self, event):
        dlg = wx.FileDialog(self, 'Save File As', '.', '', '*.*', wx.SAVE)
        try:
            if dlg.ShowModal() == wx.ID_OK:
                filename = dlg.GetPath()
                # Your code
                self.textEditor.SaveFile(filename) 
                self.FileName=filename
                self.SetTitle(('Notebook - %s') % filename)                   
        finally:
            dlg.Destroy()

    def OnMenuFileExitMenu(self, event):
        self.Close()

    def OnMenuHelpAboutMenu(self, event):
        dlg = Dialog1.Dialog1(self)
        try:
            dlg.ShowModal()
        finally:
            dlg.Destroy()
