find libs/ffmpeg-install/lib/pkgconfig -name *.pc -exec sed libs/ffmpeg-install/lib/pkgconfig/libavcodec.pc
cp -Rv libs/boost-install/* /usr/local/
cp -Rv libs/opencv-install/* /usr/local/
cp -Rv libs/ffmpeg-install/* /usr/local/
cp -Rv utils/gnuplot-iostream/gnuplot-iostream.h /usr/local/include/gnuplot
