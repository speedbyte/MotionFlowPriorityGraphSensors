#!/usr/bin/env python
# *_* encoding=utf-8 _*_

import binascii

binascii.unhexlify('41')    #-> 'A'
binascii.hexlify('A')       # -> '41'
int('41', 16)               # -> 65

float('223.2') # -> 223.2

s = "Vikäs"
str_coded_utf8 = s.decode('utf-8')               # -> typ 'str' to typ 'unicode'
str_coded_base64 = s.decode('base64')  # -> typ 'str' to typ 'unicode'
str_coded_utf8.encode('utf-8')       # -> typ 'unicode' to  typ 'str'

str = "VIKAS"
x = str.encode('utf-8')
UnpackAndConvertToHex = map(hex, struct.unpack(len(str)*'B',str))


# map second parameter requires a list
map(str, [12,34,56]) # converts [12,34,56] into ['12','34','56']
#or
[str(x) for x in [12, 34, 56]]

# to convert '0x22' into 0x22
int('0x22', 16)  # convert into integer


# to convert 255 into '0xFF'
hex(255)            # converts into str

#You can test the type of the object as follows:
a='abc'
isinstance(a, str)   # -> True
isinstance(a, (list, tuple))  # ->False





# to get the date_ 2009-05-13 for example.
import datetime
self.date = datetime.date(1, 1, 1 )
print self.date.today()
print self.date.fromtimestamp(time.time())


import os
import sys
import struct
#from xml.etree import ElementTree as ET
import elementtree.ElementTree as ET

et = ET.ElementTree(file = "PRG_77_UHV_EEPROM_15_VW_VAS5163.xml")
datasets = et.getiterator("DATA")
i = 0
for dataset in datasets:
    i += 1
    filename = "binFromXML_%i.bin" % i
    file = open(filename, "wb")    

    for data in dataset.getiterator("PARAMETER_DATA"):
        values = data.text.replace("\n","").strip().split(",")
        ints = []
        for val in values:
            file.write(struct.pack("B", int(val.strip(),16)))
    file.close()
ioutfile = "intelhexFromXML_%i.hex" % i
soutfile = "s19FromXML_%i.s19" % i
os.execl("./srec_cat.exe", "srec_cat.exe", "%s -Binary" % filename, "-output %s" % ioutfile, "-Intel")
os.execl("./srec_cat.exe", "srec_cat.exe", "%s -Binary" % filename, "-output %s" % soutfile, "-Motorola")

            filepath_open = paths[0]          # select paths from list of selected files
            filepath_open_des = open(filepath_open, "r")
            dataSetFileContents = filepath_open_des.read()
            filepath_open_des.close()

data = struct.pack("B", byteContentsHex[x])

val = struct.unpack('B',strval)
tuple zur�ck.

        def openfilebutton(self, event):
            print 'set file path'
            self.filepath_save = None
            txt  = 'Save data To...'
            Wildcard = "Save file type bla bla bla, example .txt (*.txt)|*.txt"
            dlg = wx.FileDialog(
                self, message= txt ,defaultDir=os.getcwd(), 
                wildcard=Wildcard,style=wx.SAVE|wx.CHANGE_DIR
                )
            if dlg.ShowModal() == wx.ID_OK:
                paths = dlg.GetPaths()
                paths2 = dlg.GetDirectory()
            dlg.Destroy()
            self.filepath_save = paths[0]
            
os.system("del *.*")
            
from where import did I receive the event. did it come from comboBox1 or comboBox2 etc ?

self.et = ET.ElementTree(file = xmlFileName)
blah = self.et.getiterator("edd")
blah_tiefer = blah.getiterator("dsfdsf")
print blah_tiefer.text

In Python you have the print statement, which also silently appends a linefeed (unless you end the statement with a comma). 

>>> import sys
>>> sys.stdout = open('c:/log3.txt','w')  
>>> print str(odbc.print_resultset(c))  
>>> sys.stdout=sys.__stdout__   

#!/usr/bin/python
import sys
 
# a simple class with a write method
class WritableObject:
    def __init__(self):
        self.content = []
    def write(self, string):
        self.content.append(string)
 
# example with redirection of sys.stdout
foo = WritableObject()                   # a writable object
sys.stdout = foo                         # redirection
print "one, two, three, four"            # some writing
print "little hat"
print "little hat"
sys.stdout = sys.__stdout__              # remember to reset sys.stdout!
print "foo's content:", foo.content                # show the result of the writing
 
# example with redirection of the print statement
bar = WritableObject()                   # another writable object
print >>bar, "one, two, three, four"     # some (redirected) writing
print >>bar, "little hat made of paper"
print "bar's content:", bar.content                # the result



classobject.__del__()
classobject = classname()

if the value is string or not

( isinstance(ByteValue, basestring)
if ( type(LastRow) == StringType ):
	print "String"

	
current_dir=os.getcwd()
print "Current directory:",current_dir
current_dir[:-3] # last three charachters will be cancelled.

	