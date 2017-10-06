

/**
 http://www.qtforum.org/article/18473/tutorial-for-using-qt-with-vtk.html
 wget http://download.qt.io/official_releases/qt/5.7/5.7.0/qt-opensource-linux-x64-5.7.0.run
 run the file
 Install  qt5-default. Check if the  /usr/lib/x86_64-linux-gnu/qtchooser/default.conf has qt5 entries. We can  also
 get to qmake by typing qmake -qt=qt5
 libav ( avconv ) and ffmpeg ( ffmpeg ) are competitors but finally ffmpeg wins and is included in the ubuntu versions.
 wget https://www.ffmpeg.org/releases/ffmpeg-2.8.5.tar.bz2 then ./configure then make all and then make install

 Hi, just wanted to share my experience on a jump start with using Qt+VTK.
 As I don't have experience with using Qt I decided to go the easiest possible way - downloaded Qt4 and the QtCreator.
 The steps I took next:
 1. Built Qt4 from source and installed it.
 2. Added /usr/local/Trolltech to my PATH environment variable
 3. Downloaded the source for VTK 5.4
 4. Launched ccmake in the root of the source tree.
 5. Switched ON VTK_USE_GUISUPPORT VTK_USE_QVTK.
 6. Pressed "c" to configure in ccmake, and specified the version of Qt 4.0.
 7. After configuring in ccmake, launched cmake, and then make and make install.
 8. The needed QVTKWidget.h will be in the /usr/bin/local/include/vtk-5.4.
 9. Now we launch QtCreator and create the simple application.
 10. In the mainwindow.h I add these headers:
 11. In the mainwindow.cpp I added this code in the constructor:
 12. In the .pro file I added these:

 LIBS    += -L/usr/local/lib/vtk-5.4 -lvtkCommon -lvtksys -lQVTK -lvtkQtChart -lvtkViews -lvtkWidgets -lvtkInfovis
 -lvtkRendering -lvtkGraphics -lvtkImaging -lvtkIO -lvtkFiltering -lvtklibxml2 -lvtkDICOMParser -lvtkpng -lvtkpng
 -lvtktiff -lvtkzlib -lvtkjpeg -lvtkalglib -lvtkexpat -lvtkverdict -lvtkmetaio -lvtkNetCDF -lvtksqlite -lvtkexoIIc
 -lvtkftgl -lvtkfreetype -lvtkHybrid
 INCLUDEPATH += /usr/local/include/vtk-5.4

 13. In the Project->Build settings->Build environment I added a variable LD_LIBRARY_PATH = /usr/local/lib/vtk-5.4 - so
 that when you launch the application the dynamic linker finds the libraries.
 14. After building and running (Press [F5] in Qt creator) it should show you a red QVTKWidget. :)

 */