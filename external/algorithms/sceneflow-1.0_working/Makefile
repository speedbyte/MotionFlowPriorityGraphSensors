## Makefile for sceneflow
# Authors: Frederic Huguet and Frederic Devernay
# (c)INRIA

CXX = g++
LD = g++
CXXFLAGS = -DNDEBUG -Wall -O3 -ffast-math 
#CXXFLAGS = -Wall -g
LDFLAGS = -g


OPENCVCXXFLAGS := `pkg-config --cflags opencv`
OPENCVLLDFLAGS := `pkg-config --libs opencv`
SOURCES = dictionary.cpp  groundtruth.cpp  ImagesTest.cpp  iniconfig.cpp  iniparser.cpp  Resolution.cpp  strlib.cpp resize.cpp
HEADERS = dictionary.h  groundtruth.h  ImagesTest.h  iniconfig.h  inifile.h  iniparser.h  Resolution.h  strlib.h resize.h

EXTRA_DIST = Makefile sceneflow.sln sceneflow.vcproj README.txt LICENSE-iniparser.txt \
Ball.ini Teddy.ini Venus.ini Cones.ini Diana.ini \
Ball_SF.ini Teddy_SF.ini Venus_SF.ini Cones_SF.ini Diana_SF.ini \
Images/Ball/BallL.0.pgm Images/Ball/BallL.1.pgm Images/Ball/BallR.0.pgm Images/Ball/BallR.1.pgm \
Images/Cones/ConesL.0.pgm Images/Cones/ConesL.1.pgm Images/Cones/ConesR.0.pgm Images/Cones/ConesR.1.pgm Images/Cones/GroundTruthStereoInit.png  Images/Cones/GroundTruthStereo.png Images/Cones/disc.pgm \
Images/Teddy/TeddyL.0.pgm Images/Teddy/TeddyL.1.pgm Images/Teddy/TeddyR.0.pgm Images/Teddy/TeddyR.1.pgm Images/Teddy/GroundTruthStereoInit.png  Images/Teddy/GroundTruthStereo.png Images/Teddy/disc.pgm \
Images/Venus/VenusL.0.pgm Images/Venus/VenusL.1.pgm Images/Venus/VenusR.0.pgm Images/Venus/VenusR.1.pgm Images/Venus/GroundTruthStereoInit.png  Images/Venus/GroundTruthStereo.png Images/Venus/disc.pgm \
Images/Diana/Il10.png Images/Diana/Il26.png Images/Diana/Ir10.png Images/Diana/Ir26.png \
Results/Ball/README.txt Results/Ball/Debug/README.txt \
Results/Cones/README.txt Results/Cones/Debug/README.txt \
Results/Teddy/README.txt Results/Teddy/Debug/README.txt \
Results/Venus/README.txt Results/Venus/Debug/README.txt \
Results/Diana/README.txt Results/Diana/Debug/README.txt \
bp-vision/convolve.h \
bp-vision/filter.h \
bp-vision/image.h \
bp-vision/imconv.h \
bp-vision/imutil.h \
bp-vision/Makefile \
bp-vision/misc.h \
bp-vision/pnmfile.h \
bp-vision/stereo.h \
bp-vision/readme.pdf \
bp-vision/stereo.cpp \
bp-vision/stereo.vcproj

PROJECTOBJ = $(SOURCES:.cpp=.o)

PACKAGE=sceneflow
VERSION=1.0

all: bp-vision/stereo StereoFlow Results TAGS

bp-vision/stereo:
	(cd bp-vision; $(MAKE) stereo)

.cpp.o:
	$(CXX) $(CXXFLAGS) $(OPENCVCXXFLAGS) -o $@ -c $<

StereoFlow: $(PROJECTOBJ)
	$(LD) -o $@ $^ $(LDFLAGS) $(OPENCVLLDFLAGS) $(BLINKYLLDFLAGS)

Results:
	for i in Ball Teddy Cones Venus Diana; do \
	  mkdir -p Results/$$i Results/$$i/Debug; \
	  echo "This directory will contain the final results and log file for $$i" > Results/$$i/README.txt; \
	  echo "This directory will contain the partial results and log file for $$i" > Results/$$i/Debug/README.txt; \
	done

clean:
	-rm -f $(PROJECTOBJ) StereoFlow
	-mv Results Results.old
	-rm -rf Results.old

iniconfig.o: iniconfig.cpp iniconfig.h inifile.h

ImagesTest.o: ImagesTest.cpp ImagesTest.h Resolution.h groundtruth.h iniconfig.h inifile.h

Resolution.o: Resolution.cpp Resolution.h iniconfig.h inifile.h

TAGS: $(SOURCES) $(EXTRA_DIST)
	-etags *.cpp *.h bp-vision/*.cpp bp-vision/*.h
	-etags -a -l none --regex='/^\([A-Za-z_]+\)[ \t]+=[ \t]+[^ ;]*/\1/' *.ini

dist: Results
	rm -rf $(PACKAGE)-$(VERSION)
	mkdir $(PACKAGE)-$(VERSION)
	tar cf - $(SOURCES) $(HEADERS) $(EXTRA_DIST) | (cd $(PACKAGE)-$(VERSION); tar xvf -)
	tar --owner 0 --group 0 -zcf $(PACKAGE)-$(VERSION).tar.gz $(PACKAGE)-$(VERSION) && rm -rf $(PACKAGE)-$(VERSION)

.PHONY: zip
zip: Results
	rm -rf $(PACKAGE)-$(VERSION)
	mkdir $(PACKAGE)-$(VERSION)
	tar cf - $(SOURCES) $(HEADERS) $(EXTRA_DIST) | (cd $(PACKAGE)-$(VERSION); tar xvf -)
	zip -r $(PACKAGE)-$(VERSION).zip $(PACKAGE)-$(VERSION) && rm -rf $(PACKAGE)-$(VERSION)

# for i in Ball Teddy Cones Venus Diana; do echo "This directory will contain the final results and log file for $i" > Results/$i/README.txt; echo "This directory will contain the partial results and log file for $i" > Results/$i/Debug/README.txt; done
