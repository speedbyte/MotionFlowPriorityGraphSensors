""" 


    $Date: 2008-12-17 19:40:08 +0100 (Mi, 17 Dez 2008) $
    $Rev: 3743 $
    $Author: agrawal $
    $URL: svn://voice/work/VA/Python_work/FunctionCollection.py $

    Copyright (c) 2007 S1nn GmbH & Co. KG.
    All Rights Reserved.

"""

_svn_id = "$Id$"
_svn_rev = "$Rev: 3743 $"
__version__ = _svn_rev[6:-2]





# Ascii 10 = Carriage Return ( \r )
# Ascii 13 = Line Feed ( \n )

import sys, os, time
import glob
import easyexcel
import csv
import re
import struct

def convertTemplate_to_xml_eclipse():
    """
    #create a directory - eclipse_templates_for_import
    #create a file / overwrite with the "nameofthefile"_eclipsetemplate.xml
    #add the following lines in the beginning of the code.
    #<?xml version="1.0" encoding="UTF-8" standalone="no"?><templates><template autoinsert="true" context="org.eclipse.cdt.ui.text.templates.c" deleted="false" description="this is a s1nn template for CE.h file" enabled="true" name="s1nn_template>
    #add the following line at the end of the code:
    #</template></templates>
    #add in front of the entire file wherever there is a carriage return with &#13;
    """
    try:
        os.chdir("eclipse_templates")
        os.system("del *.*")
        os.chdir("..")
    except:
        pass
    try:
        os.mkdir("eclipse_templates")
    except OSError:
        pass
    Allfiles = os.listdir(os.curdir)
    for name_of_file in Allfiles:
        if os.path.isfile(name_of_file):
                if ".c" in name_of_file:
                    operation_on_file(name_of_file, '.c')
               
                elif ".h" in name_of_file:
                    operation_on_file(name_of_file, '.h')


        
def operation_on_file(f, extension):

    fd_original = open(f,'r')
    path = os.chdir("eclipse_templates")
    newfile = f.strip(extension) + '_eclipse_' + extension.strip('.') + '.xml'
    fd_copy = open(newfile, 'w')
    contents = fd_original.read()
    replacedcontents = contents.replace("&", "&amp;")
    towrite = "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?><templates><template autoinsert=\"true\" context=\"org.eclipse.cdt.ui.text.templates.c\" deleted=\"false\" description=\"this is a s1nn template for CE.h file\" enabled=\"true\" name=\"" + f.strip(extension) + "\">"
    fd_copy.write(towrite)
    fd_copy.write(replacedcontents)
    fd_copy.close()
    fd_original.close()
    
    fd_copy = open(newfile, 'r')
    contents = fd_copy.read()
    replacedcontents = contents.replace("\n", "&#13;\n")
    fd_copy.close()
    fd_copy = open(newfile, 'w')
    fd_copy.write(replacedcontents)
    fd_copy.close()

    fd_copy = open(newfile, 'r')
    contents = fd_copy.read()
    replacedcontents = contents.replace("$", "$$")
    fd_copy.close()
    fd_copy = open(newfile, 'w')
    fd_copy.write(replacedcontents)
    fd_copy.write("</template></templates>")
    fd_copy.close()  
    path = os.chdir("..") 
        
    
          
    
def creategraphsauto():
    print "verdammt leben"
    os.chdir('E:/GraphViz/graphs/directed/')
    #os.curdir = 'E:\GraphViz\graphs\directed\'
    filenames = os.listdir(os.curdir)
    for x in filenames:
        length = len(x)
        picname = x[:length-4] + '.jpeg'
        cmd = 'dot.exe -Tjpeg -o %s %s' % (picname,x)
        try:
            os.system(cmd)
        except:
            pass
        
def createlistname(path):
    os.chdir(path)
    Allfiles = os.listdir(os.curdir)
    Globfiles = glob.glob(os.curdir+'*.dot')
    print Globfiles
    for f in Allfiles:
        print f
    cmd = 'dir *.s19 /OD'
    cmd = 'dir *.* /OD'
    os.system(cmd)
    #print os.sep()

def trimFile(filename):
    """Trims the newlines and carriage returns out of a file."""
    try:
        os.remove(filename + "-backup." + time.strftime("%m-%d-%Y"))
    except:
        print " no backup file"    
    contents = readContents(filename)
    contents = replaceWeirdCharacters(contents)
    writeContents(filename, contents)

def readContents(filename):
    """Returns the contents of a file."""
    return open(filename, "rb").read()  #opens the file in binary content


def writeContents(filename, contents):
    """Writes out the contents to filename, and backs things up just
    in case bad things happen."""
    
    os.rename(filename,
              filename + "-backup." + time.strftime("%m-%d-%Y"))
    out = open(filename, "wb")
    out.write(contents)
    out.flush()
    out.close()

def changetoUnix(filename):
    path1 = 'V:/Projektordner/Ford/S000149FO_MGM/25_SoftwareDev/VMCU/'
    path2 = 'V:\Austausch\Jahnke\Tools_KW12-08\Python Modules\\'
    path = path3
    print path
    outfile = open('E:\DirExec\eclipse\links\rvds.link','a')
    filenamenew = filename.replace('\\', '/')
    print filenamenew
    outfile.write(filenamenew)

def replaceWeirdCharacters(contents):
    """Removes newlines and carriage returns from a string, and returns
    this cleaned-up string to the user.

    Note: this uses Python 2.1 string methods: it might be better
    to use the string.replace() function instead for compatibility."""
    return contents.replace("\r", "")

def stripping():
    a = [0x35, 0x2e, 0x36, 0x2e, 0x38, 0x31, 0x30, 0x0, 0x0, 0x0]
    s = '           65$65$85$86$80$67$87$94$90$68$88$70$57$80$107$'
    print ''.join([chr(int(i)) for i in s.split('$') if i != ''])
    print ''.join(["%c" % i for i in [96,114,41,42]])
    print chr(53)
    print ord('5')
    result =  ''.join(["%c" % i for i in a if i != 0x0])
    print result
    print ''.rstrip(result)
    x = 5
    y = 10
    print "%x %x" %(x,y)    


def work_with_csv():
    #excel_instance = easyexcel.easyExcel("20081208_Kombi.xls", True, 1)
    excel_instance = csv.reader(open("20081208_Kombi.csv", "rb"))
    excel_instance_2 = csv.DictReader(open("20081208_Kombi.csv", "rb"), 'Comment')
    x = 0
    #while x < 20:
     #   print excel_instance_2.next()
      #  x = x +1 
    #for row in excel_instance:
    #    print row
    for row in excel_instance_2:
        print row
    
    for row in csv.reader(['one,two,three']):
        print row
    for row in csv.reader(['one:two:three'], delimiter=':'):
        print row

def convertToUTF8():
    result=RecreateTable[counter][i].encode('utf-8')
    tuple: (u'(p\\u0159esm\\u011brov\\xe1n)',)
    
if __name__ == '__main__':
    #convertTemplate_to_xml_eclipse()
    #work_with_csv()   
    work_with_easy_excel() 
    #excel_instance.close()
    #print os.curdir  # this will print .
    #creategraphsauto()
    #createlistname(path)
    #changetoUnix(filename)
    #trimFile(filename)
    #convert_unicode_to_byte()
    
    
# End Of File FunctionCollection.py