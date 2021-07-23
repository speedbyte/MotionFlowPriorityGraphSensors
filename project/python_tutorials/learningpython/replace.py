#!/usr/bin/env python
# replace a string in multiple files

import fileinput
import glob
import sys
import os


if len(sys.argv) < 2:
    print 'usage: %s search_text replace_text directory' \
        % os.path.basename(sys.argv[0])
    sys.exit(0)


stext = sys.argv[1]
rtext = sys.argv[2]
if len(sys.argv) == 4:
    path = os.path.join(sys.argv[3], '*')
else:
    path = '*'


print 'finding: %s and replacing with: %s' % (stext, rtext)


files = glob.glob(path)
for line in fileinput.input(files, inplace=1):
    if stext in line:
        line = line.replace(stext, rtext)
    sys.stdout.write(line)
