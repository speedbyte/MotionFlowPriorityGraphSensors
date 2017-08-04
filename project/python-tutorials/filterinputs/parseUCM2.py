## Script for processing long CAN trace files
## It writes BAP communication and MDI 0x55B message to a new
## and hopefully smaller file. It also sets CAN channel to 1 per default.
## Otherwise BAP viewer will not show data.
import tracefilter
import time
import sys

def main():
    t = tracefilter.tracefilter(infile ,outfile)
    patterns = [
                '(\s*2008-07-0.{1,})',
                '(\s*[0-9]{1,}.[0-9]{1,}\s{1,}[0-9]\s{1,}63C.{1,})',
                '(\s*[0-9]{1,}.[0-9]{1,}\s{1,}[0-9]\s{1,}63c.{1,})',
                '(\s*[0-9]{1,}.[0-9]{1,}\s{1,}[0-9]\s{1,}575.{1,})',
                '(\s*[0-9]{1,}.[0-9]{1,}\s{1,}[0-9]\s{1,}6C6.{1,})',
                '(\s*[0-9]{1,}.[0-9]{1,}\s{1,}[0-9]\s{1,}6c6.{1,})']
    replacements = [['(\s[0-9]{1}\s)',' 1 ',1]]

    t.addMatches(patterns)
    t.addReplacements(replacements)
    t.parseInfile()

if len(sys.argv) == 3:
    print 'Got command line arguments...parsing'
    infile = sys.argv[1]
    outfile = sys.argv[2]
    main()
elif len(sys.argv) == 1:
    ttuple = time.strptime(time.ctime())
    tstamp = str(ttuple[0]) + '_' + str(ttuple[1]) + '_' + str(ttuple[2])
    outfile = 'C:\\Work\\PYTHON_DEV\\canTraceFilter\\' + + 'mditrace' + tstamp + '.asc'
    infile = 'C:\\Work\\PYTHON_DEV\\canTraceFilter\\canlog.asc'
    main()
else:
    print 'Wrong command line parameters'
    print 'use: <scriptname>.py inputfile outputfile'