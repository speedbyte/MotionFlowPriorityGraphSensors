import os
import re

class tracefilter:

    def __init__(self, infile = None, outfile = None):

        self.infile = infile
        self.outfile = file(outfile,'w')
        self.patterns = []
        self.replacements = []
        self.statistics = {}

    ## Configure match patterns
    ## @param pattern List of patterns that define the regular expressions 
    ##                to match.
    def addMatches(self, pattern):
        # Check if we got a pattern list
        if type(pattern) is type([]):
            i = len(self.patterns)
            for pat in pattern:
                self.patterns.append(  re.compile(( str( pat ) ), re.IGNORECASE))
                self.statistics[i] = 0
                i += 1
        # Check if we got a string element        
        elif type(pattern) == type(''):
            self.patterns.append((re.compile(str(pattern))))
        # Neither string nor list
        else:
            # Do nothing
            pass

    ## Configures replacement patterns
    ## @param replacement List with replacement info:
    ##                    [0] = regex pattern for search
    ##                    [1] = replacement text string
    ##                    [2] = number of occurences to replace per line
    ##                    example: [['\s[0-9]{1}','new text', 1],]
    def addReplacements(self, replacement):
        # Check if we got a replacement list
        if type(replacement) is type([]) and len(replacement) > 0:
            for pat in replacement:
                self.replacements.append([re.compile(str(pat[0]), re.IGNORECASE),pat[1],pat[2]])
        # Check if we got a string element        
        else:
            # Do nothing
            pass

    ## Parses inout file for matching lines.
    ## If a line matches a replacement search is started (optional).
    ## Therefore corresponding patterns have to be defined.
    def parseInfile(self):
        # Check if filename path is valid
        if os.path.isfile(self.infile):  
            trace = file(self.infile, 'r')
            for line in trace:
                if self.matchFilter(line):
                    if self.replacements is not []:
                        line = self.replaceFilter(line)
                    self.outfile.write(line)
            self.outfile.close()
        else:
            raise Exception('Input file path invalid')
            print 'No input file found.'

    ## Compare line of text with regex pattern
    ## @param line Line from infile
    ## @return True|False depending of successful match
    def matchFilter(self, line):
        i = 0
        for p in self.patterns:
            if p.match(str(line)) is not None:
                self.statistics[i] += 1
                return True
            i += 1
        return False

    ## Replace matched entries in lines
    ## @param line Line of text to search for replacement pattern
    def replaceFilter(self, line):
        if len(self.replacements) > 0:
            for p in self.replacements:
                newline =  p[0].sub(p[1], line, p[2])
            return newline
        else:
            return line

    def getStatistics(self):
        return self.statistics


