INCDIR = -I.
#DBG    = -g
OPT    = -O3
CPP    = g++
#CPP = icc
#OPT = -O3 -xN

CFLAGS = $(OPT) $(INCDIR)
LINK   = -lm 

.cpp.o:
	$(CPP) $(CFLAGS) -c $< -o $@

all: stereo

stereo: stereo.cpp 
	$(CPP) $(CFLAGS) -o stereo stereo.cpp $(LINK)

clean:
	/bin/rm -f stereo *.o

clean-all: clean
	/bin/rm -f *~ 



