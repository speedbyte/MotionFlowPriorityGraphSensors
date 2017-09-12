## Script for processing ckcmBlues trace files where a timestamp was
## accidentally inserted into raw trace.
import ckcmfilter
import time
import sys

def main():
    t = ckcmfilter.ckcmfilter(infile ,outfile)
    t.replace()

if len(sys.argv) == 3:
    print 'Got command line arguments...parsing CKCM Trace...'
    infile = sys.argv[1]
    outfile = sys.argv[2]
    main()
    print 'Done.'
else:
    print 'Wrong command line parameters'
    print 'use: <scriptname>.py <inputfile> <outputfile>'
