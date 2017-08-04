## Script for processing long CAN trace files
## It writes MDI and UHV BAP communication and MDI 0x55B message to a new
## and hopefully smaller file. It also sets CAN channel to 1 per default.
## Otherwise BAP viewer will not show data.
import tracefilter
import time
import sys
import tempfile
import re

def main():
    t = tracefilter.tracefilter(infile ,outfile)

    channelNumber = '[0-9]'  # output from all channels and map to add.replacement ( see below ) 
    #channelNumber = '2'     # only output from channel 2 and map to add.replacement ( see below ) 
    
    patterns = [
                '(\s*[0-9]{1,}.[0-9]{1,}\s{1,}'+channelNumber+'\s{1,}55B.{1,})',
                '(\s*[0-9]{1,}.[0-9]{1,}\s{1,}'+channelNumber+'\s{1,}575.{1,})',
                '(\s*[0-9]{1,}.[0-9]{1,}\s{1,}'+channelNumber+'\s{1,}62C.{1,})',
                '(\s*[0-9]{1,}.[0-9]{1,}\s{1,}'+channelNumber+'\s{1,}63B.{1,})',
                '(\s*[0-9]{1,}.[0-9]{1,}\s{1,}'+channelNumber+'\s{1,}63C.{1,})',
                '(\s*[0-9]{1,}.[0-9]{1,}\s{1,}'+channelNumber+'\s{1,}64C.{1,})',
                '(\s*[0-9]{1,}.[0-9]{1,}\s{1,}'+channelNumber+'\s{1,}66F.{1,})',
                '(\s*[0-9]{1,}.[0-9]{1,}\s{1,}'+channelNumber+'\s{1,}67C.{1,})',
                '(\s*[0-9]{1,}.[0-9]{1,}\s{1,}'+channelNumber+'\s{1,}6C5.{1,})',
                '(\s*[0-9]{1,}.[0-9]{1,}\s{1,}'+channelNumber+'\s{1,}6C6.{1,})',
                '(\s*[0-9]{1,}.[0-9]{1,}\s{1,}'+channelNumber+'\s{1,}6c7.{1,})',
                '(\s*[0-9]{1,}.[0-9]{1,}\s{1,}'+channelNumber+'\s{1,}6d2.{1,})',
                '(\s*[0-9]{1,}.[0-9]{1,}\s{1,}'+channelNumber+'\s{1,}6d5.{1,})',
                '(\s*[0-9]{1,}.[0-9]{1,}\s{1,}'+channelNumber+'\s{1,}6d6.{1,})',
                '(\s*[0-9]{1,}.[0-9]{1,}\s{1,}'+channelNumber+'\s{1,}6d7.{1,})'
                ]
    replacements = [['(\s[0-9]{1}\s)',' 1 ',1]]

    t.addMatches(patterns)
    t.addReplacements(replacements)
    t.parseInfile()

def isDecimalFormat(filename):
    """
    Check for decimal format of CAN message log
    """
    trcfile = open(filename, 'r')
    flag = False
    i = 0
    for line in trcfile:
        if line.find('base dec') != -1:
            flag = True
            break;
        elif line.find('base hex') != -1:
            flag = False
            break;
        else:
            flag = None
    trcfile.close()
    return flag

def convertToHexFormat(filename):
    """
    Converts a decimal CAN log file to a hexadecimal one
    """
    newfilename = "hex_" + filename
    inf = open(filename, 'r')
    outf = open(newfilename, 'w')
    hd = re.compile("\s*base\s{1,}dec.{1,}")    
    pt = re.compile("\s*[0-9]{1,}.[0-9]{1,}\s{1,}[0-9]\s{1,}")
    hx = lambda tt: "%02X" % int(tt)
    for line in inf:
        if hd.match(line):
            print line
            outf.write("%sBegin Triggerblock\n" % (line.replace("dec", "hex")))
        if pt.match(line, 1):
            line = line.strip()
            seq = re.split("\s+", line)
            elements = seq[:2] + ["%02X" % int(seq[2])] + seq[3:6] + map(hx, seq[6:])
            newline = "%s\n" % (" ".join(elements))            
            outf.write(newline)
                       
    inf.close()
    outf.close()
    return newfilename

if len(sys.argv) == 3:
    print 'Got command line arguments...parsing'
    infile = sys.argv[1]
    outfile = sys.argv[2]
    decFlag = isDecimalFormat(infile)
    if decFlag == True:
        print "Log data is in decimal format. Converting to hexadecimal..."
        infile = convertToHexFormat(infile)
        main()
    elif decFlag == False:
        print "Log data is in hexdecimal format. No Conversion needed."
        main()
    else:
        print "Not a CANoe log file. Exiting!"
        # Exiting
##elif len(sys.argv) == 1:
##    ttuple = time.strptime(time.ctime())
##    tstamp = str(ttuple[0]) + '_' + str(ttuple[1]) + '_' + str(ttuple[2])
##    outfile = 'C:\\Work\\PYTHON_DEV\\canTraceFilter\\' + + 'mditrace' + tstamp + '.asc'
##    infile = 'C:\\Work\\PYTHON_DEV\\canTraceFilter\\canlog.asc'
##    main()
else:
    print 'Wrong command line parameters'
    print 'use: <scriptname>.py inputfile outputfile'
