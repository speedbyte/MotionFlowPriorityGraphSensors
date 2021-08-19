clidefault = 'info'
#111111111111111111111111111111111111111111111111111111111111 Level1 initialize
#111111111111111111111111111111111111111111111111111111111111 Level1 initialize
#111111111111111111111111111111111111111111111111111111111111 Level1 initialize
import wx
import wx.grid as gridlib
import wx.lib.scrolledpanel as scrolled
from wxPython.wx import *
import os
import time
import datetime as dt
import Image
import BmpImagePlugin    
Image._initialized = 1
import matplotlib
matplotlib.use('WX')
from matplotlib.backends.backend_wxagg import Toolbar
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
from matplotlib.backends.backend_wx import NavigationToolbar2Wx
from matplotlib.axes import Subplot
from matplotlib.pylab import *
import cStringIO
import ConfigParser
from ConfigParser import SafeConfigParser
scp = SafeConfigParser()
import base64
import bimps

import thread
from ftplib import FTP 

#222222222222222222222222222222222222222                            Level2 ID's
#222222222222222222222222222222222222222                            Level2 ID's
ID_MAIN                 = 1000
ID_WINDOW_CLI           = 1001
ID_WINDOW_TOOLBAR       = 1002
ID_WINDOW_RIGHT         = 1003
ID_WINDOW_LEFT          = 1004
ID_MAINPANEL1           = 1005
ID_MAINPANEL1_B         = 1006
ID_MAINPANEL2           = 1007
ID_MAINPANEL3           = 1008
ID_MAINPANEL4           = 1009
#__________Menu_________
ID_NEW                  = 101
ID_OPEN                 = 102
ID_PRINT                = 103
ID_EXPORT               = 104
ID_IMPORT               = 105
ID_EXIT                 = 106
#____________
ID_SHOWDEV              = 107
ID_SHOWSET              = 108 
ID_SHOWSUM              = 109
ID_SHOWPLO              = 110
#____________
ID_BACKDROP             = 111
ID_BACKDRPR             = 112
#____________
ID_SHOWFIGR             = 113
ID_RESETPLOT            = 114
ID_LEGEND               = 115
ID_FIGSEL               = 116
ID_DEFAXIS              = 117
ID_DSP                  = 118
ID_ESP                  = 119
ID_ZALL                 = 120
ID_KALL                 = 121
#____________
ID_ABOUT                = 122
#________Toolbar________
ID_TOOLBAR              = 200
ID_TOOLBAR_FIRST        = 201
ID_TOOLBAR_SECOND       = 202
ID_TOOLBAR_THIRD        = 203
ID_TOOLBAR_FOURTH       = 204
#________Additional________
IDcli                   = 302
ID_STBMP1               = 303
ID_STBMP2               = 305


#222222222222222222222222222222222222222                       Level2 wildcards
#222222222222222222222222222222222222222                       Level2 wildcards
wildcardBMP =    "Backdrop Files (*.bmp)|*.bmp"
wildcardAll =    "All Files (*.*)|*.*"

#111111111111111111111111111111111111111111111111111111111111 Level1  MainFrame
#111111111111111111111111111111111111111111111111111111111111 Level1  MainFrame
#111111111111111111111111111111111111111111111111111111111111 Level1  MainFrame
class MyFrame(wx.Frame): 
    """Initialize all the controls, variables
       menus, statusbar,panels, plots...
    """
    def __init__(self, parent, id, title): 
        wx.Frame.__init__(self, parent, id, title, wx.DefaultPosition,
                          (500,400), wx.DEFAULT_DIALOG_STYLE |
                          wx.MAXIMIZE_BOX | wx.MINIMIZE_BOX | wx.THICK_FRAME |
                          wx.RESIZE_BORDER)


        self.Bind(wx.EVT_MAXIMIZE, self.EventMaxim)
        self.Bind(wx.EVT_CLOSE,self.OnCloseWindow)
        self.Bind(wx.EVT_CHAR_HOOK, self.OnMainKeyDownEvent)

        iconFile = "s1nn.ico"                              
        self.icon1 = wx.Icon(iconFile, wx.BITMAP_TYPE_ICO)
        self.SetIcon(self.icon1)
        
#222222222222222222222222222222222222222                       Level2 variables
#222222222222222222222222222222222222222                       Level2 variables
        self.filepathBackdrop        = os.getcwd()+'\\bd.bmp'
        self.filepath_application    = os.getcwd()
        self.filepath_working        = os.getcwd()
        self.filepath_user_defined   = ''
        self.filepath_config         = 'conf.ini'
        
        winids                       = []
        self.ScreenSze               = wx.GetDisplaySize()

        self.indi                    = 1
        self.mouseeventinit          = 1
        self._c                      = 1
        self._e                      = 1
        self.firstresize             = 2
        
        self.BackdropChanged         = False
        self.staticBitmap1           = None

        self.xstart                  = 20.0
        self.xend                    = 24000.0
        self.ystart                  = -20.0
        self.yend                    = 100.0
        self._firstplot              = 0
        self.SecondPlotActive        = False
        self.activePlot              = 'Plot1'
        self.legendtoggle            = 0
        self.axestoggle              = 0    
        self.DefaultAxis             = False

#222222222222222222222222222222222222222                       Level2 statusbar
#222222222222222222222222222222222222222                       Level2 statusbar
        self.CreateStatusBar()
        self.SetStatusText("Statusbar [empty]")

#222222222222222222222222222222222222222                            Level2 menu
#222222222222222222222222222222222222222                            Level2 menu
        menuBar = wxMenuBar()
        menu1 = wxMenu()
        menu1.Append(ID_NEW,   "&New Project",        "new project")
        menu1.Append(ID_OPEN,  "&Open Project",       "open project")
        menu1.AppendSeparator()
        menu1.Append(ID_IMPORT,"&Import",             "import")
        menu1.Append(ID_EXPORT,"&Export",             "export")
        menu1.AppendSeparator()
        menu1.Append(ID_PRINT, "&Print",              "print")
        menu1.AppendSeparator()
        menu1.Append(ID_EXIT,  "E&xit",               "exit program")
        menuBar.Append(menu1,  "&File");
        #+++++++++++++++++++++++++++++++++
        menu2 = wxMenu()
        menu2.Append(ID_SHOWDEV,"Show Device",           "Dev")
        menu2.Append(ID_SHOWSET,"Show Settings",         "Set")
        menu2.Append(ID_SHOWSUM,"Show Summary",          "Sum")
        menu2.Append(ID_SHOWPLO,"Show Plot",             "Plo")
        menuBar.Append(menu2,  "&Show");
        #+++++++++++++++++++++++++++++++++
        menu3 = wxMenu()
        menu3.Append(ID_BACKDROP,"Change backdrop",      "Change backdrop")
        menu3.Append(ID_BACKDRPR,"Resize backdrop",      "Resize Backdrop")
        menuBar.Append(menu3,    "&Settings");
        #+++++++++++++++++++++++++++++++++
        menu4 = wxMenu()
        menu4.Append(ID_SHOWFIGR,    "Show Plot",        "Show figure")
        self.submenu = wx.Menu()
        self.submenu.Append(ID_DSP,  "Disable Second Plot","", wx.ITEM_RADIO)
        self.submenu.Append(ID_ESP,  "Enable Second Plot","",wx.ITEM_RADIO)
        menu4.AppendMenu(ID_FIGSEL,  "Second Plot",      self.submenu)
        self.submenu2 = wx.Menu()
        self.submenu2.Append(ID_ZALL,"Zoom All (auto)",  "auto", wx.ITEM_RADIO)
        self.submenu2.Append(ID_KALL,"Keep Values",      "def", wx.ITEM_RADIO)
        menu4.AppendMenu(ID_DEFAXIS, "Plot Axis",        self.submenu2)
        menu4.Append(ID_RESETPLOT,   "Reset Active Plot","Reset PeTool Plot")
        menu4.Append(ID_LEGEND,      "Toggle Legend",    "toggle legend")
        menuBar.Append(menu4,        "&Figure");
        #+++++++++++++++++++++++++++++++++
        menu5 = wxMenu()
        menu5.Append(ID_ABOUT,      "About",        "about...")
        menuBar.Append(menu5,       "?");
        self.SetMenuBar(menuBar)
        #+++++++++++++++++++++++++++++++++
        #+++++++++++++++++++++++++++++++++
        EVT_MENU(self, ID_NEW,          self.NotImplemented)
        EVT_MENU(self, ID_OPEN,         self.NotImplemented)
        EVT_MENU(self, ID_IMPORT,       self.NotImplemented)
        EVT_MENU(self, ID_EXPORT,       self.NotImplemented)
        EVT_MENU(self, ID_PRINT,        self.NotImplemented)
        EVT_MENU(self, ID_EXIT,         self.TimeToQuit)        

        EVT_MENU(self, ID_SHOWDEV,self.toolbarBtn1)
        EVT_MENU(self, ID_SHOWSET,self.toolbarBtn2)
        EVT_MENU(self, ID_SHOWSUM,self.toolbarBtn3)
        EVT_MENU(self, ID_SHOWPLO,self.toolbarBtn4)

        EVT_MENU(self, ID_BACKDROP,     self.ChangeBackdrop)
        EVT_MENU(self, ID_BACKDRPR,     self.resizeBackdropE)

        EVT_MENU(self, ID_SHOWFIGR,     self.toolbarBtn4)
        EVT_MENU(self, ID_DSP,          self.SecondPlotDisabled)
        EVT_MENU(self, ID_ESP,          self.SecondPlotEnabled)
        EVT_MENU(self, ID_ZALL,         self.DefAxisN)
        EVT_MENU(self, ID_KALL,         self.DefAxisY)
        EVT_MENU(self, ID_LEGEND,       self.ToggleLegend)
        EVT_MENU(self, ID_RESETPLOT,    self.ResetPlot)

        EVT_MENU(self, ID_ABOUT,        self.OnAbout)

#222222222222222222222222222222222222222222               Level 2 Create Panels
#222222222222222222222222222222222222222222               Level 2 Create Panels
#3333333333333333333333333                                   Level 3 Main Panel
        win=wx.SashLayoutWindow(self,
                                ID_WINDOW_TOOLBAR,
                                wx.DefaultPosition,
                                (100, 30),
                                wx.NO_BORDER|wx.SW_3D)
        win.SetDefaultSize((self.ScreenSze[0]-30, 36))
        win.SetOrientation(wx.LAYOUT_HORIZONTAL)
        win.SetAlignment(wx.LAYOUT_TOP)
        win.SetSashVisible(wx.SASH_BOTTOM, True)
        self.WindowToolbar = win
        winids.append(win.GetId())

        win=wx.SashLayoutWindow(self,
                                ID_WINDOW_CLI,
                                wx.DefaultPosition,
                                (100, 30),
                                wx.NO_BORDER|wx.SW_3D)
        win.SetDefaultSize((self.ScreenSze[0]-30, 30))
        win.SetOrientation(wx.LAYOUT_HORIZONTAL)
        win.SetAlignment(wx.LAYOUT_BOTTOM)
        win.SetSashVisible(wx.SASH_TOP, True)
        self.WindowCLI = win
        winids.append(win.GetId())
        self.createCLI()

        win=wx.SashLayoutWindow(self,
                                ID_WINDOW_LEFT,
                                wx.DefaultPosition,
                                (100, 30),
                                wx.NO_BORDER|wx.SW_3D)
        win.SetDefaultSize((235, 1200))
        win.SetOrientation(wx.LAYOUT_VERTICAL)
        win.SetAlignment(wx.LAYOUT_LEFT)
        win.SetSashVisible(wx.SASH_RIGHT, True)
        self.WindowLeft = win
        winids.append(win.GetId())
        self.createLeftPanelControls()

        win=wx.SashLayoutWindow(self,
                                ID_WINDOW_RIGHT,
                                wx.DefaultPosition,
                                (100, 30),
                                wx.NO_BORDER|wx.SW_3D)
        win.SetDefaultSize((180, 1200))
        win.SetOrientation(wx.LAYOUT_VERTICAL)
        win.SetAlignment(wx.LAYOUT_RIGHT)
        win.SetSashVisible(wx.SASH_LEFT, True)
        self.WindowRight = win
        winids.append(win.GetId())
        self.createRightPanelControls()
        self.WindowRight.SetBackgroundColour((223,224,227))

        self.firstplot = []
        self.plot =   Figure(figsize=(5,4), dpi=100)
        self.firstplot.append(self.plot.add_subplot(111))
        self.plot_b = Figure(figsize=(5,4), dpi=100)
        self.firstplot.append(self.plot_b.add_subplot(111))

        self.WindowMain1 =   FigureCanvas(self, ID_MAINPANEL1, self.plot)
        self.WindowMain1_b = FigureCanvas(self, ID_MAINPANEL1_B, self.plot_b)
        self.WindowMain1.Bind(wx.EVT_LEFT_DCLICK, self.WinMain1DClick)
        self.WindowMain1_b.Bind(wx.EVT_LEFT_DCLICK, self.WinMain1_bDClick)

        self.WindowMain1.mpl_connect('motion_notify_event',self.mouse_move)
        self.WindowMain1.mpl_connect('button_press_event' ,self.on_click)
        self.WindowMain1.mpl_connect('key_press_event', self.on_key)
        self.lx, = self.firstplot[0].plot( (1,1), (1,1), 'k-' )  # the horiz line
        self.ly, = self.firstplot[0].plot( (1,1), (1,1), 'k-' )  # the vert line
        frstpl = self.firstplot[0]
        self.txt=self.firstplot[0].text(0.7, 0.9,'',transform=frstpl.transAxes)
        self._ResetPlot()
        self.addPlotToolbar()

        self.WindowMain2 = scrolled.ScrolledPanel(self,
                                       ID_MAINPANEL2,
                                       size=(1000,750),
                                       style=wx.TAB_TRAVERSAL|wx.SUNKEN_BORDER,
                                       name="panel2")

        self.WindowMain3 = scrolled.ScrolledPanel(self,
                                       ID_MAINPANEL3,
                                       size=(1000,750),
                                       style=wx.TAB_TRAVERSAL|wx.SUNKEN_BORDER,
                                       name="panel3")

        self.WindowMain4 = scrolled.ScrolledPanel(self,
                                        ID_MAINPANEL4,
                                        size=(1000,750),
                                        style=wx.TAB_TRAVERSAL|wx.SUNKEN_BORDER,
                                        name="panel4")


        self.Bind(wx.EVT_SIZE, self.OnSize) 
        self.Bind(wx.EVT_SASH_DRAGGED_RANGE,self.OnSashDrag,
                  id=min(winids), id2=max(winids))

        szex = self.WindowLeft.GetSize().width
        szey = self.WindowLeft.GetSize().height /10*7-10

        self.Paneldevice    = wx.Panel(self.WindowLeft, -1,
                                        pos=(0,0), size=(szex,szey),
                                        style=wx.SUNKEN_BORDER)
        self.PanelSettings  = wx.Panel(self.WindowLeft, -1,
                                        pos=(0,0), size=(szex,szey),
                                        style=wx.SUNKEN_BORDER)
        self.PanelSummary   = wx.Panel(self.WindowLeft, -1,
                                        pos=(0,0), size=(szex,szey),
                                        style=wx.SUNKEN_BORDER)
        self.PanelPlot      = wx.Panel(self.WindowLeft, -1,
                                        pos=(0,0), size=(szex,szey),
                                        style=wx.SUNKEN_BORDER)



        self.WindowLeft.Bind(wx.EVT_CHAR_HOOK, self.OnMainKeyDownEvent)
        self.WindowRight.Bind(wx.EVT_CHAR_HOOK, self.OnMainKeyDownEvent)

        self.ReadGlobalConfigFile(self.filepath_config)

        self.ControlsWindowMain4()
        self.ControlsPaneldevice()
        self.ControlsPanelSettings()
        self.ControlsPanelSummary()
        self.ControlsPanelPlot()

        self.WindowLeft.SetBackgroundColour((224,223,227))
        self.createToolBar()
        self.toolbar_plot.Hide()
        self.HideAll()
        self.WindowMain3.Show()

#111111111111111111111111111111111111111111111111111111111111 Level1 Procedures
#111111111111111111111111111111111111111111111111111111111111 Level1 Procedures
#111111111111111111111111111111111111111111111111111111111111 Level1 Procedures
#222222222222222222222222222222222222222222                   Level 2 framework
#222222222222222222222222222222222222222222                   Level 2 framework
#3333333333333333333333333                           Level 3 Window size change
    def OnSize(self, event):
        """Resize event. Calls ResizeAll.

        """
        self.ResizeAll()
        event.Skip()
        
#3333333333333333333333333                             Level 3 Event Window max
    def EventMaxim(self,event):
        """Resize all at frame maximize event.

        """
        time.sleep(.4)
        self.ResizeAll()
        event.Skip()
        
#3333333333333333333333333                              Level 3 Not Implemented
    def NotImplemented(self, event):
        """This happens at a function call which is not implemented yet.

        """
        dlg = wxMessageDialog(self, "you are on your own... deal with it...",
                              "nothing out there...", wxOK |wxICON_INFORMATION)
        dlg.ShowModal()
        dlg.Destroy()
        self.Refresh()
        event.Skip()
        
#3333333333333333333333333                               Level 3 Settings Error
    def ErrorMessage(self, msg):
        """This happens at a function call which is not implemented yet.

        """
        dlg = wxMessageDialog(self, msg,"Error Message",
                              wxOK |wxICON_INFORMATION)
        dlg.ShowModal()
        dlg.Destroy()
        self.Refresh()

#3333333333333333333333333                                         Level 3 Exit
    def TimeToQuit(self, event):
        """Destroys GUI application.

        """
        self.Close(true)

#3333333333333333333333333                                         Level 3 Exit
    def OnCloseWindow(self,event):
        """Destroy application

        """
        print 'bye'
        self.Destroy()
        event.Skip()
        
#3333333333333333333333333                              Level 3 Version & About
    def OnAbout(self, event):
        """Shows information about the version.

        """
        dlg=wxMessageDialog(self, "S1nn - QUEST\n"
                              "Qualification of Unattendet Electronical Systems Tool\n"\
                              "Version 1.0\n"\
                              "Copyright (c) 2007\n"\
                              "Schefenacker 1nnovation GmbH & Co. KG\n"\
                              "Patrick Jahnke\n"\
                              "All rights reserved",
                              "Schefenacker 1nnovation GmbH & Co. KG", wxOK |
                              wxICON_INFORMATION)
        dlg.ShowModal()
        dlg.Destroy()

#3333333333333333333333333                                   Level 3 Create CLI
    def createCLI(self):
        """Create a command line interface

        """
        self.exBtn = wx.Button(self.WindowCLI, -1, 'Execute command:',(5,5))
        sze = self.ScreenSze[0]-self.exBtn.GetSize().width-25
        self.cmd = wx.TextCtrl(self.WindowCLI, IDcli, clidefault,
                               (self.exBtn.GetSize().width+10,4),size=(sze,23))
        self.Bind(wx.EVT_BUTTON, self.OnCLIBtn, self.exBtn)
        self.cmd.Bind(wx.EVT_TEXT_ENTER, self.OnCLIBtn,id=IDcli)

#3333333333333333333333333                                   Level 3 CLI Button
    def OnCLIBtn(self,event):
        """Execute command in CLI line.

        """
        cmds = self.cmd.GetValue()
        print 'debug message: ', cmds
        if cmds == 'info' or cmds == 'Info':
            self.info()
        else:
            exec cmds
        event.Skip()

#3333333333333333333333333                             Level 3 sash window drag
    def OnSashDrag(self, event):
        """Resize the sash window at sash drag.

        """
        if event.GetDragStatus() == wx.SASH_STATUS_OUT_OF_RANGE:
            return
        eobj = event.GetEventObject()
        if eobj is self.WindowLeft:
            self.WindowLeft.SetDefaultSize((event.GetDragRect().width, 1000))
        if eobj is self.WindowRight:
            self.WindowRight.SetDefaultSize((event.GetDragRect().width, 1000))
        self.ResizeAll()

#3333333333333333333333333                              Level 3 resize Backdrop
    def resizeBackdropE(self,event):
        self.resizeBackdrop()
        self.ResizeAll()

    def resizeBackdrop(self):
        rs = self.WindowMain2
        if self.filepathBackdrop != None and self.filepathBackdrop != '':
            filepath = self.filepathBackdrop
            if self.staticBitmap1 != None:
                self.staticBitmap1.Destroy()
            im1=Image.open(filepath).resize((int(rs.GetSize().height/3*3.5),
                                            rs.GetSize().height),
                                            Image.ANTIALIAS)
            im1.save("bds.bmp")
            im1=wx.Image(('bds.bmp'),wx.BITMAP_TYPE_BMP).ConvertToBitmap()
            self.staticBitmap1 = wx.StaticBitmap(bitmap=im1, id=ID_STBMP1,
                                             parent=self.WindowMain2,
                                             pos=wx.Point(8, 8), style=0)

#3333333333333333333333333                              Level 3 change backdrop
    def ChangeBackdrop(self,event):
        dlg = wx.FileDialog(
            self, message="Choose a Backdrop file", defaultDir=os.getcwd(), 
            defaultFile="",wildcard=wildcardBMP,style=wx.OPEN|
            wx.CHANGE_DIR)
        dlg.SetDirectory(self.filepath_application)
        if dlg.ShowModal() == wx.ID_OK:
            paths = dlg.GetPaths()
            for path in paths:
               filepath = path
        dlg.Destroy()
        self.filepathBackdrop = filepath
        self.BackdropChanged = True
        self.resizeBackdrop()
        self.ResizeAll()
        
 #3333333333333333333333333                                      Level 3 Toolbar
    def createToolBar(self):
        """Create custom toolbar

        """
        pic = base64.decodestring(bimps.tb_1_pic)
        stream = cStringIO.StringIO(pic)
        im1 = wx.BitmapFromImage(wx.ImageFromStream(stream))
        tb_1= im1

        pic = base64.decodestring(bimps.tb_2_pic)
        stream = cStringIO.StringIO(pic)
        im1 = wx.BitmapFromImage(wx.ImageFromStream(stream))
        tb_2= im1

        pic = base64.decodestring(bimps.tb_4_pic)
        stream = cStringIO.StringIO(pic)
        im1 = wx.BitmapFromImage(wx.ImageFromStream(stream))
        tb_3= im1

        pic = base64.decodestring(bimps.tb_3_pic)
        stream = cStringIO.StringIO(pic)
        im1 = wx.BitmapFromImage(wx.ImageFromStream(stream))
        tb_4= im1

        tb = wx.ToolBar(self.WindowToolbar,ID_TOOLBAR)
        self.ToolBar = tb
        tb.SetToolBitmapSize((37,27))

        tb.AddTool(ID_TOOLBAR_FIRST, tb_1, isToggle=True)
        wx.EVT_TOOL(self, ID_TOOLBAR_FIRST, self.SetMode)
        tb.AddTool(ID_TOOLBAR_SECOND, tb_2, isToggle=True)
        wx.EVT_TOOL(self, ID_TOOLBAR_SECOND, self.SetMode)

        tb.AddSeparator()
        tb.AddSeparator()

        tb.AddTool(ID_TOOLBAR_THIRD, tb_3, isToggle=True)
        wx.EVT_TOOL(self, ID_TOOLBAR_THIRD, self.SetMode)
        tb.AddTool(ID_TOOLBAR_FOURTH, tb_4, isToggle=True)
        wx.EVT_TOOL(self, ID_TOOLBAR_FOURTH, self.SetMode)

        tb.Realize()

#3333333333333333333333333                              Level 3 Toolbar setmode
    def SetMode(self,event):
        """Toggles mode for the toolbar selection.

        """
        for id in [ID_TOOLBAR_FIRST,ID_TOOLBAR_SECOND,ID_TOOLBAR_THIRD,
                   ID_TOOLBAR_FOURTH]:
            self.ToolBar.ToggleTool(id,0)

        self.ToolBar.ToggleTool(event.GetId(),1)
        if event.GetId() == ID_TOOLBAR_FIRST:
            self.toolbarBtn1(event)

        elif event.GetId() == ID_TOOLBAR_SECOND:
            self.toolbarBtn2(event)

        elif event.GetId() == ID_TOOLBAR_THIRD:
            self.toolbarBtn3(event)

        elif event.GetId() == ID_TOOLBAR_FOURTH:
            self.toolbarBtn4(event)

#3333333333333333333333333                               Level 3 toolbar device
    def toolbarBtn1(self, event):
        """Show device panel. Hide all unneeded panels.

        """
        self.HideAll()
        self.updatedeviceValues()
        self.Paneldevice.Show()
        self.WindowMain2.Show()
        self.ResizeAll()

#3333333333333333333333333                            Level 3 toolbar settings
    def toolbarBtn2(self, event):
        """Show Settings panel. Hide all unneeded panels.

        """
        self.HideAll()
        self.updateSettingsValues()
        self.PanelSettings.Show()
        self.WindowMain2.Show()
        self.ResizeAll()

#3333333333333333333333333                                  Level 3 toolbar sum
    def toolbarBtn3(self, event):
        """Show Summary panel. Hide all unneeded panels.

        """
        self.HideAll()
        self.PanelSummary.Show()
        self.updateSummary()
        self.WindowMain4.Show()
        self.ResizeAll()
            
#3333333333333333333333333                                 Level 3 toolbar plot
    def toolbarBtn4(self, event):
        """Show Plot panel. Hide all unneeded panels.

        """
        self.HideAll()
        self.PanelPlot.Show()
        self.WindowMain1.Show()
        if self.SecondPlotActive == True:
            self.WindowMain1_b.Show()
        self.ResizeAll()

#3333333333333333333333333                                 Level 3 Plot Toolbar 
    def addPlotToolbar(self):
        """Add toolbar to control the Plot. Zoom, pan....

        """
        self.toolbar_plot = NavigationToolbar2Wx(self.WindowMain1)
        self.toolbar_plot.Realize()
        self.toolbar_plot.SetSizeWH(220,30)
        tw, th = self.toolbar_plot.GetSizeTuple()
        fw, fh = self.WindowMain1.GetSizeTuple()
        self.toolbar_plot.SetSize(wxSize(fw, th))
        self.toolbar_plot.update()

#3333333333333333333333333                                   Level 3 Mouse move
    def mouse_move(self, event):
        """Mouse move envent for freehand drawing, cross hair.

        """
        self.indi = self.indi+1

        if not event.inaxes: return 

        if self.mouseeventinit == 1:
            ax = event.inaxes
            self.mouseeventinit += 1
            minx, maxx = ax.get_xlim()
            miny, maxy = ax.get_ylim()
            x, y = event.xdata, event.ydata
            self.lx.set_data( (minx, maxx), (y, y) )
            self.ly.set_data( (x, x), (miny, maxy) )
            self.txt.set_text( 'x=%1.2f, y=%1.2f'%(x,y) )
            self.lx.set_alpha(0)
            self.ly.set_alpha(0)
            self.txt.set_alpha(0)
            self.WindowMain1.draw()

        if self._c % 2 == 0:
            self._crosshair(event)

#3333333333333333333333333                                   Level 3 cross hair
    def _crosshair(self,event):
        """Create cross hair in axes of plot.

        """
        ax = event.inaxes
        minx, maxx = ax.get_xlim()
        miny, maxy = ax.get_ylim()
        x, y = event.xdata, event.ydata
        self.lx.set_data( (minx, maxx), (y, y) )
        self.ly.set_data( (x, x), (miny, maxy) )
        self.txt.set_text( 'x=%1.2f, y=%1.2f'%(x,y) )
        self.WindowMain1.draw()

#3333333333333333333333333                                        Level 3 index
    def on_click(self,event):
        """Initialize plot manipulation if key 'f' is pressed

        """
        if event.inaxes==None: return

#3333333333333333333333333                                        Level 3 onKey
    def on_key(self,event):
        """

        """
        if event.key=='m':
            if self._e % 2==0:
                self.pldata.set_marker('')
            else:
                self.pldata.set_marker('o')
            self._e+=1
            self.WindowMain1.draw()

        if event.key=='c': 
            self._c += 1
            if self._c % 2 == 0:
                self.lx.set_alpha(1)
                self.ly.set_alpha(1)
                self.txt.set_alpha(1)

            else:
                self.lx.set_alpha(0)
                self.ly.set_alpha(0)
                self.txt.set_alpha(0)
            self.WindowMain1.draw()

#222222222222222222222222222222222222222         Level2  second plot management
#222222222222222222222222222222222222222         Level2  second plot management
#3333333333333333333333333                                   Level 3 reset plot
    def SecondPlotEnabled(self,event):
        self.SecondPlotActive = True
        self.activePlot = 'Plot2'
        self.firstplot[0].set_axis_bgcolor((.7,.7,.7))
        self.firstplot[1].set_axis_bgcolor((1,1,1))
        if self.WindowMain1.IsShown() == True:
            self.WindowMain1_b.Show()
        self.ResizeAll()
        event.Skip()
        
    def SecondPlotDisabled(self,event):
        self.SecondPlotActive = False
        self.activePlot = 'Plot1'
        self.firstplot[1].set_axis_bgcolor((.7,.7,.7))
        self.firstplot[0].set_axis_bgcolor((1,1,1))
        self.WindowMain1_b.Hide()
        self.ResizeAll()
        event.Skip()
        
    def WinMain1DClick(self,event):
        self.activePlot = 'Plot1'
        self.firstplot[1].set_axis_bgcolor((.7,.7,.7))
        self.firstplot[0].set_axis_bgcolor((1,1,1))

        self.xstart = self.firstplot[0].get_xlim()[0]
        self.xend   = self.firstplot[0].get_xlim()[1]
        self.ystart = self.firstplot[0].get_ylim()[0]
        self.yend   = self.firstplot[0].get_ylim()[1]
        self.WinR_TC1.SetValue(str(self.xstart))
        self.WinR_TC2.SetValue(str(self.xend))
        self.WinR_TC3.SetValue(str(self.ystart))
        self.WinR_TC4.SetValue(str(self.yend))

        self.WindowMain1_b.Refresh()
        self.WindowMain1.Refresh()
        event.Skip()

    def WinMain1_bDClick(self,event):
        self.activePlot = 'Plot2'
        self.firstplot[0].set_axis_bgcolor((.7,.7,.7))
        self.firstplot[1].set_axis_bgcolor((1,1,1))

        self.xstart = self.firstplot[1].get_xlim()[0]
        self.xend   = self.firstplot[1].get_xlim()[1]
        self.ystart = self.firstplot[1].get_ylim()[0]
        self.yend   = self.firstplot[1].get_ylim()[1]
        self.WinR_TC1.SetValue(str(self.xstart))
        self.WinR_TC2.SetValue(str(self.xend))
        self.WinR_TC3.SetValue(str(self.ystart))
        self.WinR_TC4.SetValue(str(self.yend))

        self.WindowMain1.Refresh()
        self.WindowMain1_b.Refresh()

#2222222222222222222222222222222222222222222             Level2 plot management
#2222222222222222222222222222222222222222222             Level2 plot management
#3333333333333333333333333                                    Level 3 resetplot
    def ResetPlot(self,event):
        self._ResetPlot()
        event.Skip()
        
    def _ResetPlot(self):
        """Reset plot.

        """
        if self.activePlot == 'Plot1':
            usedplot = self.firstplot[0]
            usedplot.set_title('Plot 1')
        if self.activePlot == 'Plot2':
            usedplot = self.firstplot[1]
            usedplot.set_title('Plot 2')
        if usedplot != None:
            p = usedplot
            p.cla()
            if self.DefaultAxis == True:
                p.set_autoscale_on(False)
            if self.DefaultAxis == False:
                p.set_autoscale_on(True)
            p.grid()
            if usedplot.get_xlim()[0] == 0:
                usedplot.set_xlim(0.1,usedplot.get_xlim()[1])
                print 'replaced: the 0 value at xstart at log scaled plot'
            if usedplot.get_xlim()[0] == usedplot.get_xlim()[1]:
                up = usedplot
                up.set_xlim(up.get_xlim()[0]-1,up.get_xlim()[1])
                print 'replaced: xstart now is != xend'
            p.semilogx()
            self.font = {'fontname'   : 'Courier',
                         'color'      : 'b',
                         'fontweight' : 'bold',
                         'fontsize'   : 11}
            p.set_xlabel('Frequency [Hz]', self.font)
            p.set_ylabel('Magnitude [dB]', self.font)
            self.WinR_TC1.SetValue(str(usedplot.get_xlim()[0])) 
            self.WinR_TC2.SetValue(str(usedplot.get_xlim()[1]))
            self.WinR_TC3.SetValue(str(usedplot.get_ylim()[0]))
            self.WinR_TC4.SetValue(str(usedplot.get_ylim()[1]))
            self._c = 1
            self._d = 1
            self._e = 1
            self._f = 1
            self.WindowMain1.Refresh()
            self.WindowMain1_b.Refresh()
            if self.activePlot == 'Plot1':
                usedplot.set_title('Plot 1')
            if self.activePlot == 'Plot2':
                usedplot.set_title('Plot 2')

#3333333333333333333333333                                Level 3 toggle legend
    def ToggleLegend(self,event):
        """Toggle legend visibility True/False.

        """        
        if self.legendtoggle % 2 == 0:
            if self.activePlot == 'Plot1':
                self.firstplot[0].get_legend().set_visible(False)
                self.WindowMain1.Refresh()
            if self.activePlot == 'Plot2':
                self.firstplot[1].get_legend().set_visible(False)
                self.WindowMain1_b.Refresh()
        else:
            if self.activePlot == 'Plot1':
                self.firstplot[0].get_legend().set_visible(True)
                self.WindowMain1.Refresh()
            if self.activePlot == 'Plot2':
                self.firstplot[1].get_legend().set_visible(True)
                self.WindowMain1_b.Refresh()
        self.legendtoggle +=1
        event.Skip()
        
#3333333333333333333333333                                 Level 3 default axis
    def DefAxisY(self,event):
        """Set the axes of the plots to default values.

        """
        if self.activePlot == 'Plot1':
            usedplot = self.firstplot[0]
        if self.activePlot == 'Plot2':
            usedplot = self.firstplot[1]
        self.DefaultAxis = True
        usedplot.set_autoscale_on(False)
        self.WinR_TC1.SetBackgroundColour((200,200,200)) 
        self.WinR_TC2.SetBackgroundColour((200,200,200))
        self.WinR_TC3.SetBackgroundColour((200,200,200))
        self.WinR_TC4.SetBackgroundColour((200,200,200))
        self.WinR_ST2.SetLabel('PeTool Plot axis  *hold*')        
        self.PeToolAxis(event)
        event.Skip()
        
    def DefAxisN(self,event):
        """Auto set the axes of the plot.

        """
        if self.activePlot == 'Plot1':
            usedplot = self.firstplot[0]
        if self.activePlot == 'Plot2':
            usedplot = self.firstplot[1]
        self.DefaultAxis = False
        self.WinR_TC1.SetBackgroundColour((255,255,255))
        self.WinR_TC2.SetBackgroundColour((255,255,255))
        self.WinR_TC3.SetBackgroundColour((255,255,255))
        self.WinR_TC4.SetBackgroundColour((255,255,255))
        self.WinR_ST2.SetLabel('PeTool Plot axis')        
        usedplot.set_autoscale_on(True)
        event.Skip()
        
    def EventAxesValueToggle(self,event):
        if self.axestoggle % 2 == 0:
            self.DefAxisY(event)
            self.submenu2.Check(ID_KALL,True)
        if self.axestoggle % 2 == 1:
            self.DefAxisN(event)
            self.submenu2.Check(ID_ZALL,True)
        self.axestoggle +=1
        self.WindowRight.Refresh()
        event.Skip()
        
#3333333333333333333333333                                 Level 3 set controls
    def PeToolAxis(self,event):
        """Set default axes values to text controls.

        """
        self.xstart = self.WinR_TC1.GetValue()
        self.xend = self.WinR_TC2.GetValue()
        self.ystart = self.WinR_TC3.GetValue()
        self.yend = self.WinR_TC4.GetValue()
        self.WinR_TC1.SetValue(str(self.xstart))
        self.WinR_TC2.SetValue(str(self.xend))
        self.WinR_TC3.SetValue(str(self.ystart))
        self.WinR_TC4.SetValue(str(self.yend))
        event.Skip()
        
    def SetAxisControlOnPlotEvent(self):
        """Set text controls to plot axes limits.

        """
        if self.activePlot == 'Plot1':
            usedplot = self.firstplot[0]
        if self.activePlot == 'Plot2':
            usedplot = self.firstplot[1]
        x = list(usedplot.get_xlim())
        y = list(usedplot.get_ylim())
        self.WinR_TC1.SetValue(str(x[0]))
        self.WinR_TC2.SetValue(str(x[1]))
        self.WinR_TC3.SetValue(str(y[0]))
        self.WinR_TC4.SetValue(str(y[1]))

#3333333333333333333333333                                  Level 3 update axis
    def ChangePlotAxis(self,event):
        """Update all plot axes.

        """
        self.ChangePlotAxisXS(event)
        self.ChangePlotAxisXE(event)
        self.ChangePlotAxisYS(event)
        self.ChangePlotAxisYE(event)
        event.Skip()
        
#3333333333333333333333333                               Level 3 change x start
    def ChangePlotAxisXS(self,event):
        """Update the plot x-start axes.

        """
        if self.activePlot == 'Plot1':
            usedplot = self.firstplot[0]
        if self.activePlot == 'Plot2':
            usedplot = self.firstplot[1]
        if usedplot != None:
            self.xstart = float(self.WinR_TC1.GetValue())
            try:
                usedplot.set_xlim(self.xstart,self.xend)
            except:
                pass
            self.WindowMain1.Refresh()
            self.WindowMain1_b.Refresh()
        event.Skip()
        
#3333333333333333333333333                               Level 3 change x start
    def ChangePlotAxisXE(self,event):
        """Update the plot x-end axes.

        """
        if self.activePlot == 'Plot1':
            usedplot = self.firstplot[0]
        if self.activePlot == 'Plot2':
            usedplot = self.firstplot[1]
        if usedplot != None:
            self.xend = self.WinR_TC2.GetValue()
            try:
                usedplot.set_xlim(self.xstart,self.xend)
            except:
                pass
            self.WindowMain1.Refresh()
            self.WindowMain1_b.Refresh()
        event.Skip()
        
#3333333333333333333333333                               Level 3 change x start
    def ChangePlotAxisYS(self,event):
        """Update the plot y-start axes.

        """
        if self.activePlot == 'Plot1':
            usedplot = self.firstplot[0]
        if self.activePlot == 'Plot2':
            usedplot = self.firstplot[1]
        if usedplot != None:
            self.ystart = self.WinR_TC3.GetValue()
            try:
                usedplot.set_ylim(self.ystart,self.yend)
            except:
                pass
            self.WindowMain1.Refresh()
            self.WindowMain1_b.Refresh()
        event.Skip()
        
#3333333333333333333333333                               Level 3 change x start
    def ChangePlotAxisYE(self,event):
        """Update the plot y-end axes.

        """
        if self.activePlot == 'Plot1':
            usedplot = self.firstplot[0]
        if self.activePlot == 'Plot2':
            usedplot = self.firstplot[1]
        if usedplot != None:
            self.yend = self.WinR_TC4.GetValue()
            try:
                usedplot.set_ylim(self.ystart,self.yend)
            except:
                pass
            self.WindowMain1.Refresh()
            self.WindowMain1_b.Refresh()
        event.Skip()
        

#111111111111111111111111111111111111111111111111111111111111111111111111111111
#111111111111111111111111111111111111111111111111111111111111111111111111111111
#111111111111111111111111111111111111111111111111111111111111111111111111111111
#2222222222222222222222222222222222                 Level 2 Left Panel Controls
#2222222222222222222222222222222222                 Level 2 Left Panel Controls
    def createLeftPanelControls(self):
        """Create the controls for the left sashpanel.

        """
        self.WinL_SL1 = wx.StaticLine(self.WindowLeft, -1, style=0)
        self.WinL_ST1 = wx.StaticText(self.WindowLeft, -1, "Measurement:")
        self.WinL_Btn1 = wx.Button(self.WindowLeft, -1, 'Measure')
        self.Bind(wx.EVT_BUTTON, self.BtnMeasure, self.WinL_Btn1)

        self.WinL_SL2 = wx.StaticLine(self.WindowLeft, -1, style=0)
        self.WinL_Btn2 = wx.Button(self.WindowLeft, -1, 'Repeat last')
        self.Bind(wx.EVT_BUTTON, self.BtnMeasureRepeat, self.WinL_Btn2)
        self.WinL_Btn3 = wx.Button(self.WindowLeft, -1, 'Cancel')
        self.Bind(wx.EVT_BUTTON, self.BtnMeasureCancel, self.WinL_Btn3)
        self.WinL_ST2 = wx.StaticText(self.WindowLeft, -1, "Mic. Position: 0")
        self.WinL_ST2.SetFont(wx.Font(12, wx.SWISS, wx.NORMAL, wx.BOLD, False,
                                      'MS Shell Dlg 2'))
        
        self.WinL_ST2.Hide()

#2222222222222222222222222222222222                Level 2 Right Panel Controls
#2222222222222222222222222222222222                Level 2 Right Panel Controls
    def createRightPanelControls(self):
        """Create controls for the right sashpanel.

        """
        self.WinR_SL1  = wx.StaticLine(self.WindowRight, -1, style=0)
        self.WinR_ST1  = wx.StaticText(self.WindowRight, -1,
                                           "System Information")
        self.ProInfo_1 = wx.StaticText(self.WindowRight, -1,
                                           "Device: ")
        self.ProInfo_2 = wx.StaticText(self.WindowRight, -1,
                                           "Test running: ")
        self.ProInfo_3 = wx.StaticText(self.WindowRight, -1,
                                           "Test duration: ")
        self.ProInfo_4 = wx.StaticText(self.WindowRight, -1,
                                           "Test Sequence ID: ")
        self.ProInfo_5 = wx.StaticText(self.WindowRight, -1,
                                           "External Sync:.: ")
        
        self.ProInfo_1a = wx.StaticText(self.WindowRight, -1,
                                                 "not defined ")
        self.ProInfo_2a = wx.StaticText(self.WindowRight, -1,
                                                 "not defined ")
        self.ProInfo_3a  = wx.StaticText(self.WindowRight, -1,
                                                 "not defined")
        self.ProInfo_4a = wx.StaticText(self.WindowRight, -1,
                                                 "not defined")
        self.ProInfo_5a = wx.StaticText(self.WindowRight, -1,
                                                 "not defined")

        self.WinR_SL2 = wx.StaticLine(self.WindowRight, -1, style=0)
        self.WinR_ST2 =wx.StaticText(self.WindowRight,-1,
                                         'Plot axis')
        self.WinR_ST3 = wx.StaticText(self.WindowRight, -1,'x-axis')
        self.WinR_ST4 = wx.StaticText(self.WindowRight, -1,'y-axis')
        self.WinR_TC1   = wx.TextCtrl(self.WindowRight, -1,
                                         style=0, value='20')
        self.WinR_TC2   = wx.TextCtrl(self.WindowRight, -1,
                                         style=0, value='20000')
        self.WinR_TC3   = wx.TextCtrl(self.WindowRight, -1,
                                         style=0, value='0')
        self.WinR_TC4   = wx.TextCtrl(self.WindowRight, -1,
                                         style=0, value='100')

        self.WinR_TC1.Bind(wx.EVT_TEXT, self.ChangePlotAxisXS,id=-1)
        self.WinR_TC2.Bind(wx.EVT_TEXT, self.ChangePlotAxisXE,id=-1)
        self.WinR_TC3.Bind(wx.EVT_TEXT, self.ChangePlotAxisYS,id=-1)
        self.WinR_TC4.Bind(wx.EVT_TEXT, self.ChangePlotAxisYE,id=-1)
        self.WinR_TC1.Bind(wx.EVT_TEXT_ENTER, self.ChangePlotAxisXS,id=-1)
        self.WinR_TC2.Bind(wx.EVT_TEXT_ENTER, self.ChangePlotAxisXE,id=-1)
        self.WinR_TC3.Bind(wx.EVT_TEXT_ENTER, self.ChangePlotAxisYS,id=-1)
        self.WinR_TC4.Bind(wx.EVT_TEXT_ENTER, self.ChangePlotAxisYE,id=-1)
        self.WinR_TC1.Bind(wx.EVT_LEFT_DCLICK, self.EventAxesValueToggle)
        self.WinR_TC2.Bind(wx.EVT_LEFT_DCLICK, self.EventAxesValueToggle)
        self.WinR_TC3.Bind(wx.EVT_LEFT_DCLICK, self.EventAxesValueToggle)
        self.WinR_TC4.Bind(wx.EVT_LEFT_DCLICK, self.EventAxesValueToggle)

        im1=wx.Image(('questlogo.bmp'),wx.BITMAP_TYPE_BMP).ConvertToBitmap() 
        self.staticBitmap2 = wx.StaticBitmap(bitmap=im1, id=ID_STBMP2,
                                        parent=self.WindowRight,
                                        pos=wx.Point(8, 8), style=0)
        
#2222222222222222222222222222222222               Level 2 Panel Device Controls
#2222222222222222222222222222222222               Level 2 Panel Device Controls
    def ControlsPaneldevice(self):
        """Create the controls for the device Panel.

        """
        self.Pdevice_SL1 = wx.StaticLine(self.Paneldevice, -1, style=0)
        self.Pdevice_ST1 = wx.StaticText(self.Paneldevice, -1,
                                           "device Settings")


#2222222222222222222222222222222222             Level 2 Panel Settings Controls
#2222222222222222222222222222222222             Level 2 Panel Settings Controls
    def ControlsPanelSettings(self):
        """Create controls for Settings Panel.

        """
        self.PSettings_SL4 = wx.StaticLine(self.PanelSettings, -1, style=0)
        self.PSettings_ST4 = wx.StaticText(self.PanelSettings, -1,
                                           "Audio Settings")

        self.PSettings_ST4a = wx.StaticText(self.PanelSettings,-1,
                                            'Sample Rate [Hz]:')
        self.PSettings_TC4a = wx.TextCtrl(self.PanelSettings,
                                         -1,
                                         'leer')
        self.PSettings_ST4b = wx.StaticText(self.PanelSettings,-1,'Points:')
        self.PSettings_TC4b = wx.TextCtrl(self.PanelSettings,
                                         -1,
                                         'leer')
        self.PSettings_ST4c = wx.StaticText(self.PanelSettings,-1,'Averages:')
        self.PSettings_TC4c = wx.TextCtrl(self.PanelSettings,
                                         -1,
                                         'leer')
        self.PSettings_ST4d=wx.StaticText(self.PanelSettings,-1,'Output Gain [dB]:')
        self.PSettings_TC4d = wx.TextCtrl(self.PanelSettings,
                                         -1,
                                         'leer')
        self.PSettings_ST4e=wx.StaticText(self.PanelSettings,-1,'Input Gain [dB]:')
        self.PSettings_TC4e = wx.TextCtrl(self.PanelSettings,
                                         -1,
                                         'leer')

        self.PSettings_TC4a.Bind(wx.EVT_KEY_UP, self.OnTextCtrlSettingsKeyUp)
        self.PSettings_TC4b.Bind(wx.EVT_KEY_UP, self.OnTextCtrlSettingsKeyUp)
        self.PSettings_TC4c.Bind(wx.EVT_KEY_UP, self.OnTextCtrlSettingsKeyUp)
        self.PSettings_TC4d.Bind(wx.EVT_KEY_UP, self.OnTextCtrlSettingsKeyUp)
        self.PSettings_TC4e.Bind(wx.EVT_KEY_UP, self.OnTextCtrlSettingsKeyUp)


    def OnTextCtrlSettingsKeyUp(self, event):
        event.Skip()
        self.updateSettingsSettings()

#2222222222222222222222222222222222              Level 2 Panel Summary Controls
#2222222222222222222222222222222222              Level 2 Panel Summary Controls
    def ControlsPanelSummary(self):
        """Create controls for Summary Panel.

        """
        self.PSummary_SL1 = wx.StaticLine(self.PanelSummary, -1, style=0)
        self.PSummary_ST1 = wx.StaticText(self.PanelSummary, -1,
                                           "Show Summary")

#2222222222222222222222222222222222              Level 2 Panel Summary Controls
#2222222222222222222222222222222222              Level 2 Panel Summary Controls
    def ControlsWindowMain4(self):
        self.WindowMain4.SetBackgroundColour((255,255,255))

        self.Summarygrid=wx.grid.Grid(self.WindowMain4,
                                      -1, name='SummaryGrid',style =0)
        self.Summarygrid.CreateGrid(100, 3)
        self.Summarygrid.SetRowLabelSize(20)
        self.Summarygrid.SetColLabelSize(20)
        self.Summarygrid.SetColLabelValue(0, "")
        self.Summarygrid.SetColLabelValue(1, "")
        self.Summarygrid.SetColLabelValue(2, "")
        self.Summarygrid.SetColLabelAlignment(wx.ALIGN_CENTER, wx.ALIGN_BOTTOM)
        row = 100
        while row >= 0:
            col = 3
            while col >= 0:
                self.Summarygrid.SetReadOnly(row, col, True)
                col = col - 1
            row = row - 1

        self.boxSizer1 = wx.BoxSizer(orient=wx.VERTICAL)
        self.SumText0=wx.StaticText(self.WindowMain4,-1,"QUEST - Report:")
        self.SumText1= wx.StaticText(self.WindowMain4,-1,"Active Path: ")
        self.SumText2= wx.StaticText(self.WindowMain4,-1,"Date: ")
        self.SumText3= wx.StaticText(self.WindowMain4,-1,"Author: ")
        self.SumLine1=wx.StaticLine(self.WindowMain4,-1,style=0)

        self.SumText0.SetFont(wx.Font(16, wx.SWISS, wx.NORMAL, wx.BOLD, True))
        self.SumText1.SetFont(wx.Font(12, wx.SWISS, wx.NORMAL, wx.NORMAL,False))
        self.SumText2.SetFont(wx.Font(12, wx.SWISS, wx.NORMAL, wx.NORMAL,False))
        self.SumText3.SetFont(wx.Font(12, wx.SWISS, wx.NORMAL, wx.NORMAL,False))
        
        self.boxSizer1.Add(self.SumText0, 0, wx.ALIGN_LEFT|wx.ALL, 5) 
        self.boxSizer1.Add(self.SumText1, 0, wx.ALIGN_LEFT|wx.ALL, 5) 
        self.boxSizer1.Add(self.SumText2, 0, wx.ALIGN_LEFT|wx.ALL, 5) 
        self.boxSizer1.Add(self.SumText3, 0, wx.ALIGN_LEFT|wx.ALL, 5) 
        self.boxSizer1.Add(self.SumLine1, 0, wx.GROW | wx.ALIGN_LEFT|wx.ALL, 0)
        self.boxSizer1.SetDimension(10,10,400,250)

        pic = base64.decodestring(bimps.logo_pic)
        stream = cStringIO.StringIO(pic)
        im1 = wx.BitmapFromImage(wx.ImageFromStream(stream))

        self.SummaryBitmap1 = wx.StaticBitmap(bitmap=im1, id=-1,
                                             name='SummaryBitmap1',
                                             parent=self.WindowMain4,
                                             pos=wx.Point(8, 8),
                                             size=wx.Size(200,155), style=0)

#3333333333333333333333333                               Level 3 update summary
    def updateSummary(self):
        """Update Summary Panel data and layout.
        
        """
        self.SumText1.SetLabel('Active Path: %s '%(self.filepath_working))
        self.SumText2.SetLabel(dt.datetime.now().strftime("Date: %A %m/%d/20%y"))
        self.SumText3.SetLabel('Author: %s '%(win32api.GetUserName()))
        sg = self.Summarygrid
        sg.ClearGrid()
        sg.DeleteRows(0,sg.GetNumberRows())
        sg.AppendRows(100)

        sg.SetCellValue(0, 0, 'tbd')
        sg.SetCellSize(3, 0, 1, 3)
        sg.SetCellBackgroundColour(3,0,(200,200,200))


        for row in range(0,100):
            for col in range(0,3):
                sg.SetCellFont(row, col, wx.Font(10, wx.SWISS,
                                            wx.NORMAL,wx.NORMAL, False))
        sg.SetCellFont(0, 0, wx.Font(10, wx.SWISS, wx.NORMAL,wx.BOLD, False))
        sg.SetCellFont(4, 0, wx.Font(10, wx.SWISS, wx.NORMAL,wx.BOLD, False))
        row = 15
        sg.DeleteRows(row-1,sg.GetNumberRows()-row+1)

        self.info()

#2222222222222222222222222222222222                 Level 2 Panel Plot Controls
#2222222222222222222222222222222222                 Level 2 Panel Plot Controls
    def ControlsPanelPlot(self):
        """Create controls for Plot Panel.

        """
        self.PPlot_SL1 = wx.StaticLine(self.PanelPlot, -1, style=0)
        self.PPlot_ST1 = wx.StaticText(self.PanelPlot, -1,
                                               "Show Plot")

#111111111111111111111111111111111111111111111111111111111111111111111111111111
#111111111111111111111111111111111111111111111111111111111111111111111111111111
#111111111111111111111111111111111111111111111111111111111111111111111111111111
#222222222222222222222222222222222               Level 3 On Main Panel Key Down
#222222222222222222222222222222222               Level 3 On Main Panel Key Down
    def OnMainKeyDownEvent(self,evt):
        """Global Key events for the function keys (start, stop, mute...)

        """
        if evt.GetKeyCode() == 342: #F1
            print 'key code F1'

        if evt.GetKeyCode() == 343: #F2
            print 'key code F2'

        if evt.GetKeyCode() == 344: #F3
            print 'key code F3'

        if evt.GetKeyCode() == 345: #F4
            print 'key code F4'

        if evt.GetKeyCode() == 346: #F5
            print 'key code F5'

        if evt.GetKeyCode() == 347: #F6
            print 'key code F6'

        if evt.GetKeyCode() == 348: #F7
            print 'key code F7'

        if evt.GetKeyCode() == 349: #F8
            print 'key code F8'

        if evt.GetKeyCode() == 350: #F9
            print 'key code F9'

        if evt.GetKeyCode() == 351: #F10
            print 'key code F10'

        if evt.GetKeyCode() == 352: #F11
            print 'key code F11'

        if evt.GetKeyCode() == 353: #F12
            print 'key code F12'

        evt.Skip()
       
#3333333333333333333333333                               Level 3 Button Measure
    def BtnMeasure(self, event):
        """Set the MeasurementActive to True and disable all controls for
           safety.
        """          
        print 'apply'

#3333333333333333333333333                                Level 3 Button Cancel
    def BtnMeasureCancel(self, event):
        """Cancel measurement. Reset variables and enable all controls.

        """
        print 'measure cancel'
 
#3333333333333333333333333                                Level 3 Button Repeat
    def BtnMeasureRepeat(self,event):
        """Repeat the last measurement step...(?)

        """
        print 'measure repeat'

#3333333333333333333333333                        Level 3 Button device Connect
    def BtndeviceConnect(self,event):
        print 'btndeviceconnect'

    def EvtText(self, event):
        print 'get password'
        self.pw = event.GetString()

    def EvtTextSU(self, event):
        print 'get su password'
        self.pwsu = event.GetString()

#222222222222222222222222222222222222222222          Level 2 Read Global Config
#222222222222222222222222222222222222222222          Level 2 Read Blobal Config
    def ReadGlobalConfigFile(self,path):
        """Read out the file given by importConfigFile and evaluate values.
           (samplerate, points, seats, channels, averages, deviceuse, gains...)

        """
        scp.read(path)
        for section in scp.sections():
            for item_name, item_value in scp.items(section):
                print "Startup INI %s - %s: %s" %(section,item_name,item_value)
                if item_name == 'm_devices':
                    self.m_Devices = eval(item_value)

        scp.read("defdir.ini")
        for section in scp.sections():
            for item_name, item_value in scp.items(section):
                if item_name == 'defdir':
                    self.filepath_user_defined = item_value
                    print 'Startup INI Settings -',self.filepath_user_defined
                    
#222222222222222222222222222222222222222222               Level 2 Set User Path
#222222222222222222222222222222222222222222               Level 2 Set User Path
    def SetDefUserPath(self,event):
        """Open FileDialog to navigate to a default directory.

        """        
        txt  = "Set Default User Path"
        txt2 = "Set Default Path To This Directory"
        dlg = wx.FileDialog(
            self, message= txt ,defaultDir=os.getcwd(), 
            defaultFile= txt2,wildcard=wildcardAll,style=wx.SAVE|wx.CHANGE_DIR
            )
        if dlg.ShowModal() == wx.ID_OK:
            paths = dlg.GetDirectory()
            self.filepath_user_defined = paths
        dlg.Destroy()
        if self.filepath_user_defined != None and self.filepath_user_defined != '':
            print self.filepath_user_defined
            f = file(self.filepath_application+'\\defdir.ini',"wU")
            config = ConfigParser.ConfigParser()
            config.add_section("DefDir")
            config.set("DefDir","defdir",self.filepath_user_defined)
            config.write(f)
            f.close() 
        else:
            return
 
#222222222222222222222222222222222222222222         Level 2 Update panel values
#222222222222222222222222222222222222222222         Level 2 Update panel values
#3333333333333333333333333                      Level 3 Measurement Info Update
    def updateMeasurementInfo(self):
        self.ProInfo_1a.SetLabel('1leer')
        self.ProInfo_2a.SetLabel('2leer')
        self.ProInfo_3a.SetLabel('3leer')
        self.ProInfo_4a.SetLabel('4leer')
        self.ProInfo_5a.SetLabel('5leer')
        
#3333333333333333333333333                          Level 3 device panel update
    def updatedeviceValues(self):
        """For safety update the control values on the panel, so that even
           changes made at the command line are known and shown in GUI.
        """
        print 'update device values (set control values)'


    def updatedeviceSettings(self):
        """For safety update the control values on the panel, so that even
           changes made at the command line are known and shown in GUI.
        """
        print 'update device settings (get control values)'

#3333333333333333333333333                        Level 3 Settings panel update
    def updateSettingsValues(self):
        """For safety update the control values on the panel, so that even
           changes made at the command line are known and shown in GUI.
        """
        print 'update setting values (set control values)'
        self.PSettings_TC4a.SetValue('1leer')
        self.PSettings_TC4b.SetValue('2leer')
        self.PSettings_TC4c.SetValue('3leer')
        self.PSettings_TC4d.SetValue('4leer')
        self.PSettings_TC4e.SetValue('5leer')

    def updateSettingsSettings(self):
        """For safety update the control values on the panel, so that even
           changes made at the command line are known and shown in GUI.
        """
        print 'update setting settings (get control values)'
        self.m_SampleRate = self.PSettings_TC4a.GetValue()
        self.m_Points     = self.PSettings_TC4b.GetValue()
        self.m_Averages   = self.PSettings_TC4c.GetValue()
        self.m_OutputGain = N.power(10.0,float(self.PSettings_TC4d.GetValue())/20)
        self.m_InputGain  = N.power(10.0,float(self.PSettings_TC4e.GetValue())/20)
        

    def updateAllSettings(self):
        """Make a complete update that all internal values are correctly
           shown at GUI controls.
        """
        print 'update all settings'
        self.updatedeviceSettings()
        self.updateSettingsSettings()

    def updateAllValues(self):
        """A complete update of control values.

        """
        print 'update all values'
        self.updatedeviceValues()
        self.updateSettingsValues()
        self.updateSummary()
        self.ResizeAll()

#222222222222222222222222222222222222222222                  Level 2 Resize all
#222222222222222222222222222222222222222222                  Level 2 Resize all
#3333333333333333333333333                             Level 3 Hide and Disable
    def disableAllControls(self):
        """Disables all controls.

        """        
        self.PSettings_TC4a.Disable()
        self.PSettings_TC4b.Disable()
        self.PSettings_TC4c.Disable()
        self.PSettings_TC4d.Disable()
        self.PSettings_TC4e.Disable()
        
    def enableAllControls(self):
        """Enables all controls.

        """        
        self.PSettings_TC4a.Enable()
        self.PSettings_TC4b.Enable()
        self.PSettings_TC4c.Enable()
        self.PSettings_TC4d.Enable()
        self.PSettings_TC4e.Enable()

    def HideAll(self):
        """Hide all Panels

        """
        self.WindowMain1.Hide()
        self.WindowMain1_b.Hide()
        self.WindowMain2.Hide()
        self.WindowMain3.Hide()
        self.WindowMain4.Hide()
        self.Paneldevice.Hide()
        self.PanelSettings.Hide()
        self.PanelSummary.Hide()
        self.PanelPlot.Hide()
        self.WinL_Btn1.Disable()        

#3333333333333333333333333                                   Level 3 resize all
    def ResizeAll(self):
        """Resize every dynamic control, text, button etc..

        """
        wx.LayoutAlgorithm().LayoutFrame(self, self.WindowMain2)
        wx.LayoutAlgorithm().LayoutFrame(self, self.WindowMain3)
        wx.LayoutAlgorithm().LayoutFrame(self, self.WindowMain4)
        self.cmd.SetSizeWH(self.GetSize().width-132,23)

        lmx = self.WindowLeft.GetSize().width
        lmy = self.WindowLeft.GetPosition().y
        rmx = self.WindowRight.GetSize().width
        rmy = self.WindowRight.GetPosition().y
        pw = self.GetSize().width - lmx - rmx -8
        ph = self.WindowRight.GetSize().height
        
        if self.SecondPlotActive == False:
            self.WindowMain1.Move((lmx,lmy))
            self.WindowMain1.SetSizeWH(pw,ph)
            self.WindowMain1_b.Move((lmx,lmy))
            self.WindowMain1_b.SetSizeWH(pw,ph)
        if self.SecondPlotActive == True:
            self.WindowMain1.SetSizeWH(pw,ph/2)
            self.WindowMain1.Move((lmx,lmy))
            lmy2 = lmy + self.WindowMain1.GetSize().height
            self.WindowMain1_b.SetSizeWH(pw,ph/2)
            self.WindowMain1_b.Move((lmx,lmy2))

    # backdrop resize +++ backdrop resize +++ backdrop resize +++ backdrop resi
    # backdrop resize +++ backdrop resize +++ backdrop resize +++ backdrop resi
        if self.firstresize >=0:
            self.resizeBackdrop()
        self.firstresize = self.firstresize -1

        self.staticBitmap1.SetSizeWH(self.WindowMain2.GetSize().width,
                                     self.WindowMain2.GetSize().height)
        self.staticBitmap1.CenterOnParent(wx.BOTH)
        self.staticBitmap1.Refresh()

    # WindowLeft resize +++ WindowLeft resize +++ WindowLeft resize +++ WindowL
    # WindowLeft resize +++ WindowLeft resize +++ WindowLeft resize +++ WindowL
        self.WinL_SL1.SetSizeWH(self.WindowLeft.GetSize().width-10,2)
        self.WinL_SL1.Move((5,self.WindowLeft.GetSize().height/10*7))
        self.WinL_ST1.Move((self.WinL_SL1.GetPosition().x+13,
                                 self.WinL_SL1.GetPosition().y-6))
        self.WinL_SL2.SetSizeWH(self.WindowLeft.GetSize().width-10,2)
        self.WinL_SL2.Move((5,self.WindowLeft.GetSize().height/10*8))

        s=self.WinL_SL2.GetPosition().y-self.WinL_SL1.GetPosition().y+3
        self.WinL_Btn1.SetSizeWH(self.WindowLeft.GetSize().width/2,s/2)
        x=self.WindowLeft.GetSize().width/2-self.WinL_Btn1.GetSize().width/2
        y = self.WinL_SL1.GetPosition().y+s/2-self.WinL_Btn1.GetSize().height/2
        self.WinL_Btn1.Move((x,y))

        self.WinL_Btn2.SetSizeWH(self.WindowLeft.GetSize().width/3,30)
        self.WinL_Btn3.SetSizeWH(self.WindowLeft.GetSize().width/3,30)
        x=self.WindowLeft.GetSize().width/3*2-self.WindowLeft.GetSize().width/8+5
        xx=self.WindowLeft.GetSize().width/3-self.WindowLeft.GetSize().width/4
        y = self.WinL_SL2.GetPosition().y + 17
        self.WinL_Btn2.Move((x,y))
        self.WinL_Btn3.Move((xx,y))

        x = self.WinL_Btn3.GetPosition().x + self.WinL_Btn3.GetSize().width/2
        y = self.WinL_Btn3.GetPosition().y + self.WinL_Btn3.GetSize().height*2
        self.WinL_ST2.Move((x,y))

    # WindowRight resize +++ WindowRight resize +++ WindowRight resize +++ Wind
    # WindowRight resize +++ WindowRight resize +++ WindowRight resize +++ Wind
        rwsl      = self.WinR_SL1
        rwsl1     = self.WinR_SL2
        rwst1     = self.WinR_ST2
        rwxaxisST = self.WinR_ST3
        rwyaxisST = self.WinR_ST4
        rwXStart  = self.WinR_TC1
        rwXEnd    = self.WinR_TC2
        rwYStart  = self.WinR_TC3
        rwYEnd    = self.WinR_TC4

        rwsl.SetSizeWH(self.WindowRight.GetSize().width-10,2)
        rwsl.Move((5,self.WindowRight.GetSize().height/5*4))
        self.WinR_ST1.Move((rwsl.GetPosition().x+13,
                                 rwsl.GetPosition().y-6))

        x = self.WindowRight.GetSize().width/10*4+5
        rwYStart.SetSizeWH(x,21)
        rwYEnd.SetSizeWH(x,21)
        rwXStart.SetSizeWH(x,21)
        rwXEnd.SetSizeWH(x,21)
        rwYStart.Move((10,rwsl.GetPosition().y-42))
        x = rwYStart.GetSize().width + rwYStart.GetPosition().x
        rwYEnd.Move((x+5,rwsl.GetPosition().y-42))
        rwyaxisST.Move((x-13,rwYStart.GetPosition().y-16))
        rwXStart.Move((10,rwyaxisST.GetPosition().y-22))
        rwXEnd.Move((x+5, rwXStart.GetPosition().y))
        rwxaxisST.Move((x-13, rwXStart.GetPosition().y-16))
        rwsl1.SetSizeWH(self.WindowRight.GetSize().width-10,2)
        rwsl1.Move((5,rwxaxisST.GetPosition().y-10))
        rwst1.Move((rwsl1.GetPosition().x+13,
                    rwsl1.GetPosition().y-6))

        self.ProInfo_1.Move((rwsl.GetPosition().x+5,
                                 rwsl.GetPosition().y+20))
        self.ProInfo_2.Move((rwsl.GetPosition().x+5,
                                 self.ProInfo_1.GetPosition().y+15))
        self.ProInfo_3.Move((rwsl.GetPosition().x+5,
                                self.ProInfo_2.GetPosition().y+15))
        self.ProInfo_4.Move((rwsl.GetPosition().x+5,
                                  self.ProInfo_3.GetPosition().y+15))
        self.ProInfo_5.Move((rwsl.GetPosition().x+5,
                                 self.ProInfo_4.GetPosition().y+15))
        self.ProInfo_1a.Move((self.ProInfo_3.GetPosition().x+100,
                                       rwsl.GetPosition().y+20))
        self.ProInfo_2a.Move((self.ProInfo_3.GetPosition().x+100,
                                       self.ProInfo_1.GetPosition().y+15))
        self.ProInfo_3a.Move((self.ProInfo_3.GetPosition().x+100,
                                      self.ProInfo_2.GetPosition().y+15))
        self.ProInfo_4a.Move((self.ProInfo_3.GetPosition().x+100,
                                        self.ProInfo_3.GetPosition().y+15))
        self.ProInfo_5a.Move((self.ProInfo_3.GetPosition().x+100,
                                       self.ProInfo_4.GetPosition().y+15))
        sze=self.WindowRight.GetSize().width-self.ProInfo_3.GetSize().width

        szex = self.WindowLeft.GetSize().width-2
        szey = self.WindowLeft.GetSize().height/10*7-10


    # general control size +++ general control size +++ general control size ++
    # general control size +++ general control size +++ general control size ++
        szeX = 120
        szeY = 22
        self.PSettings_TC4a.SetSizeWH(szeX,szeY)
        self.PSettings_TC4b.SetSizeWH(szeX,szeY)
        self.PSettings_TC4c.SetSizeWH(szeX,szeY)
        self.PSettings_TC4d.SetSizeWH(szeX,szeY)
        self.PSettings_TC4e.SetSizeWH(szeX,szeY)
        x = 11
        oy = 27

    # device resize +++ device resize +++ device resize +++ device resize +++ s
    # device resize +++ device resize +++ device resize +++ device resize +++ s
        self.Paneldevice.SetSizeWH(szex,szey)
        slw = self.Paneldevice.GetSize().width-10
        self.Pdevice_SL1.SetSizeWH(slw,2)
        self.Pdevice_SL1.Move((5,8))
        self.Pdevice_ST1.Move((self.Pdevice_SL1.GetPosition().x+13,
                               self.Pdevice_SL1.GetPosition().y-6))
        
    # settings resize +++ settings resize +++ settings resize +++ settings resi
    # settings resize +++ settings resize +++ settings resize +++ settings resi
        self.PanelSettings.SetSizeWH(szex,szey)
        slw = self.PanelSettings.GetSize().width-10
        self.PSettings_SL4.SetSizeWH(slw,2)
        self.PSettings_SL4.Move((5,8))
        self.PSettings_ST4.Move((self.PSettings_SL4.GetPosition().x+13,
                                 self.PSettings_SL4.GetPosition().y-6))
        self.PSettings_ST4a.Move((x,self.PSettings_SL4.GetPosition().y+15))
        self.PSettings_TC4a.Move((105,self.PSettings_SL4.GetPosition().y+15))
        self.PSettings_ST4b.Move((x,self.PSettings_ST4a.GetPosition().y+oy))
        self.PSettings_TC4b.Move((105,self.PSettings_ST4a.GetPosition().y+oy))
        self.PSettings_ST4c.Move((x,self.PSettings_ST4b.GetPosition().y+oy))
        self.PSettings_TC4c.Move((105,self.PSettings_ST4b.GetPosition().y+oy))
        self.PSettings_ST4d.Move((x,self.PSettings_ST4c.GetPosition().y+oy))
        self.PSettings_TC4d.Move((105,self.PSettings_ST4c.GetPosition().y+oy))
        self.PSettings_ST4e.Move((x,self.PSettings_ST4d.GetPosition().y+oy))
        self.PSettings_TC4e.Move((105,self.PSettings_ST4d.GetPosition().y+oy))

    # summary resize +++ summary resize +++ summary resize +++ summary resize +
    # summary resize +++ summary resize +++ summary resize +++ summary resize +
        self.PanelSummary.SetSizeWH(szex,szey)

        self.PSummary_SL1.SetSizeWH(self.PanelSummary.GetSize().width-10,2)
        self.PSummary_SL1.Move((5,8))
        self.PSummary_ST1.Move((self.PSummary_SL1.GetPosition().x+13,
                               self.PSummary_SL1.GetPosition().y-6))

        s = self.WindowMain4
        bmp = self.SummaryBitmap1
        sg = self.Summarygrid
        y = s.GetSize().height-216
        sg.SetSizeWH(s.GetSize().width-5,y)
        sg.Move((0,
            self.boxSizer1.GetPosition().y+self.boxSizer1.GetSize().height-50))
        sg.SetColSize(0, 155)
        sg.SetColSize(1, 125)
        sg.SetColSize(2, sg.GetSize().width-325)

        self.Summarygrid.AutoSizeRow(0)             
        self.Summarygrid.AutoSizeRow(1)
        self.Summarygrid.AutoSizeRows()

        bmp.Move((s.GetSize().width-233,22))
        
    # plot resize +++ plot resize +++ plot resize +++ plot resize +++ plot resi
    # plot resize +++ plot resize +++ plot resize +++ plot resize +++ plot resi
        self.PanelPlot.SetSizeWH(szex,szey)

        self.PPlot_SL1.SetSizeWH(self.PanelPlot.GetSize().width-10,2)
        self.PPlot_SL1.Move((5,8))
        self.PPlot_ST1.Move((self.PPlot_SL1.GetPosition().x+13,
                               self.PPlot_SL1.GetPosition().y-6))

    # toolbar resize +++ toolbar resize +++ toolbar resize +++ toolbar resize + 
    # toolbar resize +++ toolbar resize +++ toolbar resize +++ toolbar resize + 
        self.toolbar_plot.Move((self.WindowToolbar.GetPosition().x+520,
                                self.WindowToolbar.GetPosition().y))

    # refresh +++ refresh +++ refresh +++ refresh +++ refresh +++ refresh +++ r
    # refresh +++ refresh +++ refresh +++ refresh +++ refresh +++ refresh +++ r
        self.updateMeasurementInfo()
        self.toolbar_plot.Raise()
        self.WindowLeft.Refresh()
        self.WindowRight.Refresh()
        self.toolbar_plot.Refresh()
        self.WindowMain1.Refresh()
        self.WindowMain1_b.Refresh()
        self.WindowMain2.Refresh()
        self.WindowMain3.Refresh()
        self.WindowMain4.Refresh()


#1111111111111111111111111111111111111111111111111111          Level1 Show Info
#1111111111111111111111111111111111111111111111111111          Level1 Show Info
#1111111111111111111111111111111111111111111111111111          Level1 Show Info
    def info(self):
        print '***************************************************************'
        print '***************************************************************'
        print '***************************************************************'
        print 'self.filepath_working             -> ', self.filepath_working
        print '... tbd ...'
        print '***************************************************************'
        print '***************************************************************'
        print '***************************************************************'
      
#111111111111111111111111111111111111111111111111111111111111 Level1 Splash Scr
#111111111111111111111111111111111111111111111111111111111111 Level1 Splash Scr
#111111111111111111111111111111111111111111111111111111111111 Level1 Splash Scr
class SplashScreen(wx.SplashScreen):
    """Call and show SplashScreen.

    """
    def __init__(self):
        """Show the S1nn splash screen.

        """
        im1=wx.Image(('splash.bmp'),wx.BITMAP_TYPE_BMP).ConvertToBitmap()
        self.splashscreen = wx.SplashScreen.__init__(self, im1,
                                wx.SPLASH_CENTRE_ON_SCREEN | wx.SPLASH_TIMEOUT
                                | wx.FRAME_SHAPED, 1000, None, -1)
        self.Bind(wx.EVT_CLOSE, self.OnCloseSplash)
        self.fc = wx.FutureCall(1000, self.ShowMain)

    def OnCloseSplash(self, evt):
        evt.Skip()
        self.Hide()
        
        if self.fc.IsRunning():
            self.fc.Stop()
            self.ShowMain()

    def ShowMain(self):
        frame = MyFrame(None, ID_MAIN, "S1nn - QUEST")
        frame.Maximize(1)
        frame.Show(True) 
        if self.fc.IsRunning():
            self.Raise()

    def SetWindowShape(self, *evt):
        r = wx.RegionFromBitmap(self.SplashBMP)
        self.hasShape = self.SetShape(r)

#111111111111111111111111111111111111111111111111111111111111 Level1 Init Appl.
#111111111111111111111111111111111111111111111111111111111111 Level1 Init Appl.
#111111111111111111111111111111111111111111111111111111111111 Level1 Init Appl.
class GUIApp(wx.App):
    """Initialize wx.Application

    """
    def OnInit(self):
        splash = SplashScreen()
        splash.Show()
        return True

#222222222222222222222222222222222222222                                 Level2
#222222222222222222222222222222222222222                                 Level2
if __name__ == '__main__': 
    app = GUIApp(0)
    app.MainLoop()