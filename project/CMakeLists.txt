cmake_minimum_required(VERSION 2.8)
project(DisplayImage)

set(CMAKE_VERBOSE_MAKEFILE on)

# Prints the environment variables
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(CMakePrintNothingConfig)  # Looks for CMakePrint.cmake under CMAKE_MODULES_PATH


#Find the modules ( the modules are coded inside the OpenCVConfig.cmake and OpenCVModules.make )
#-----------
set(OpenCV_DIR "/local/usr/local/share/OpenCV")
find_package( OpenCV REQUIRED ) # see the list of the modules below that would be loaded.
#-----------

# IN FFPMEG
# To use pkg-config instead of cmake modules, shell needs to be invoked.
# CXXFLAGS="-D__STDC_CONSTANT_MACROS -Wdeprecated-declarations" ./configure --prefix="$(pwd)/build" --bindir="./bin" --enable-shared 

# IN OPENCV
#export PKG_CONFIG_PATH=$FFMPEG_PWD/build/lib/pkgconfig:$BOOTSTRAP_PWD/stage/lib/pkgconfig     
#PKG_CONFIG_PATH_FFMPEG=$(pkg-config --cflags libavcodec) 
#if [[ $PKG_CONFIG_PATH_FFMPEG == "-I$FFMPEG_PWD/build/include"* ]]; then echo "Correct PKG_CONFIG_PATH_FFMPEG $PKG_CONFIG_PATH_FFMPEG"; else echo "Incorrect PKG_CONFIG_PATH_FFMPEG $PKG_CONFIG_PATH_FFMPEG"; exit_function; fi

if ( OpenCV_FOUND )
message (STATUS "found OpenCV package")
endif (OpenCV_FOUND)

add_executable(DisplayImage DisplayImage.cpp GreyImage.cpp)
target_link_libraries (DisplayImage ${OpenCV_LIBS})

add_executable(OpticalFlow main.cpp)
target_link_libraries (OpticalFlow ${OpenCV_LIBS})
#opencv_calib3d
#opencv_core
#opencv_features2d
#opencv_flann
#opencv_highgui
#opencv_imgcodecs
#opencv_imgproc
#opencv_ml
#opencv_objdetect
#opencv_photo
#opencv_shape
#opencv_stitching
#opencv_superres
#opencv_video
#opencv_videoio
#opencv_videostab

# Another way to print the environment variables
#set(CMakePrint_DIR  "${CMAKE_SOURCE_DIR}/cmake")
#find_package(CMakePrint)


# Some tries
# CMAKE_MODULE_PATH -> where to find the packages
# CMAKE_PREFIX_PATH -> where to find the packages
#find_library(OpenCV_CORE NAMES libopencv_core.so HINTS "/local/usr/local/lib/")
#find_library(OpenCV_HIGHGUI NAMES libopencv_highgui.so HINTS "/local/usr/local/lib/")
#find_library(OpenCV_IMGPROC NAMES libopencv_imgproc.so HINTS "/local/usr/local/lib/")
#find_library(OpenCV_IMGCODECS NAMES libopencv_imgcodecs.so HINTS "/local/usr/local/lib/")
#include_directories(${OpenCV_INCLUDE_DIRS})
