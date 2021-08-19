## Script for processing long CAN trace files
## It writes BAP communication and MDI 0x55B message to a new
## and hopefully smaller file. It also sets CAN channel to 1 per default.
## Otherwise BAP viewer will not show data.
import tracefilter
import time
import sys
import os

def main():
    t = tracefilter.tracefilter(infile ,outfile)
    patterns = ['([0-9]{4}\-[0-9]{2}\-[0-9]{2}\s{1,}[0-9]{2}\:[0-9]{2}\:[0-9]{2}\,[0-9]{1,}>\s{1,}LOG:\suCmedia\sMDI)',
                '([0-9]{4}\-[0-9]{2}\-[0-9]{2}\s{1,}[0-9]{2}\:[0-9]{2}\:[0-9]{2}\,[0-9]{1,}>\s{1,}AL:\sCAN\son\srequest.{1,})',
                '([0-9]{4}\-[0-9]{2}\-[0-9]{2}\s{1,}[0-9]{2}\:[0-9]{2}\:[0-9]{2}\,[0-9]{1,}>\s{1,}AL:\sFirst\sUSB\son.{1,})',
                '([0-9]{4}\-[0-9]{2}\-[0-9]{2}\s{1,}[0-9]{2}\:[0-9]{2}\:[0-9]{2}\,[0-9]{1,}>\s{1,}AL:\sMalloc.{1,})',
                '([0-9]{4}\-[0-9]{2}\-[0-9]{2}\s{1,}[0-9]{2}\:[0-9]{2}\:[0-9]{2}\.[0-9]{1,}>\s{1,}AG:\s{1,}<DLE><STX>)',
                '([0-9]{4}\-[0-9]{2}\-[0-9]{2}\s{1,}[0-9]{2}\:[0-9]{2}\:[0-9]{2}\.[0-9]{1,}>\s{1,}AG:\s{1,}<DLE><ACK>)',
                '([0-9]{4}\-[0-9]{2}\-[0-9]{2}\s{1,}[0-9]{2}\:[0-9]{2}\:[0-9]{2}\.[0-9]{1,}>\s{1,}AG:\s{1,}<DLE><NAK>)',
                '([0-9]{4}\-[0-9]{2}\-[0-9]{2}\s{1,}[0-9]{2}\:[0-9]{2}\:[0-9]{2}\.[0-9]{1,}>\s{1,}ZAB:\s{1,}<DLE><STX>)',
                '([0-9]{4}\-[0-9]{2}\-[0-9]{2}\s{1,}[0-9]{2}\:[0-9]{2}\:[0-9]{2}\.[0-9]{1,}>\s{1,}ZAB:\s{1,}<DLE><ACK>)',
                '([0-9]{4}\-[0-9]{2}\-[0-9]{2}\s{1,}[0-9]{2}\:[0-9]{2}\:[0-9]{2}\.[0-9]{1,}>\s{1,}ZAB:\s{1,}<DLE><NAK>)']

    desc = [' ucMedia Resets found',
            ' CAN On requests found',
            ' USB On requests found',
            ' Malloc errors found',
            ' ucMedia Send messages',
            ' ucMedia ACKs',
            ' ucMedia NAKs',
            ' ucCan Send messages',
            ' ucCan ACKs',
            ' ucCan NAKs'
            ]

    replacements = [['(\s[0-9]{1}\s)',' 1 ',1]]

    input = file(infile, 'r')
    output = file(outfile, 'a')
    t.addMatches(patterns)
    for line in input:
        t.matchFilter(line)

    matches = t.getStatistics()
    i = 0
    for p in patterns:
        result = str(matches[i]) + desc[i] + '\n'
        print result
        output.write(result)
        i += 1

    input.close()
    output.close()

if len(sys.argv) == 3:
    print 'Got command line arguments...parsing'
    infile = sys.argv[1]
    outfile = sys.argv[2]
    main()
elif len(sys.argv) == 1:
    ttuple = time.strptime(time.ctime())
    tstamp = str(ttuple[0]) + '_' + str(ttuple[1]) + '_' + str(ttuple[2])
    outfile = 'C:\\Work\\PYTHON_DEV\\canTraceFilter\\' + 'ucM_result' + tstamp + '.asc'
    infile = 'C:\\Work\\PYTHON_DEV\\canTraceFilter\\MDI_uCm_Trace.log'
    main()
else:
    print 'Wrong command line parameters'
    print 'use: <scriptname>.py inputfile outputfile'

