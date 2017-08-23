cmake_minimum_required (VERSION 2.6)
project (Tutorial)

# The version number.
set (Tutorial_VERSION_MAJOR 1)
set (Tutorial_VERSION_MINOR 0)

# add the binary tree to the search path for include files
# so that we will find TutorialConfig.h
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}/libs/opencv/include")
include_directories("${PROJECT_BINARY_DIR}")

configure_file (
  "${PROJECT_SOURCE_DIR}/TutorialConfig.h.in"
  "${PROJECT_BINARY_DIR}/TutorialConfig.h"
  )

option (USE_MYMATH "Use Tutorial provided math implementation" ON)

if (USE_MYMATH)
add_subdirectory(maths) # This directory must contain a CMakeLists.txt too
set (EXTRA_LIBS ${EXTRA_LIBS} MathFunctions)
endif (USE_MYMATH)

add_executable(Tutorial Tutorial.cpp)
target_link_libraries(Tutorial ${EXTRA_LIBS} )


message(${PROJECT_BINARY_DIR})

cmake_minimum_required(VERSION 3.5)
project(PriorityGraphSensors)

set(CMAKE_VERBOSE_MAKEFILE on)

# Prints the environment variables
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(CMakePrintNothingConfig)  # Looks for CMakePrintNothing.cmake under CMAKE_MODULES_PATH
#include(CMakePrintConfig)  # Looks for CMakePrint.cmake under CMAKE_MODULES_PATH

# Method to use .cmake
#find_package(OpenCV REQUIRED HINTS "${CMAKE_SOURCE_DIR}/../libs/opencv-install/share/OpenCV") # see the list of the modules below that would be loaded.

#Method to use pkgconfig. Assuming PKG_CONFIG_PATH is already set, otherwise set using ENV.
# If PKG_CONFIG_PATH is not set, then CMAKE_PREFIX_PATH can be set to find the .pc files
#set(CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/libs/ffmpeg-install/lib/pkgconfig")
#set(ENV{PKG_CONFIG_PATH}  "${CMAKE_SOURCE_DIR}/libs/opencv-install/lib/pkgconfig")

find_package(PkgConfig REQUIRED)
pkg_check_modules(PKG_OPENCV REQUIRED opencv)
pkg_search_module(PKG_OPENCV REQUIRED opencv)

if ( PKG_OPENCV_FOUND )
message (STATUS "found OPENCV package")
message (STATUS ${PKG_OPENCV_INCLUDE_DIRS})
message (STATUS ${PKG_OPENCV_LIBRARIES})
message (STATUS ${PKG_OPENCV_LDFLAGS})
message (STATUS ${OpenCV_LIBS})
endif (PKG_OPENCV_FOUND)

#-----------
MESSAGE( STATUS " pkgconfigpath:        " $ENV{PKG_CONFIG_PATH} )

include_directories(${PKG_OPENCV_INCLUDE_DIRS})
add_executable(DisplayImage DisplayImage.cpp GreyImage.cpp)
target_link_libraries (DisplayImage ${PKG_OPENCV_LDFLAGS})

add_executable(Disparity Disparity.cpp)
target_link_libraries (Disparity ${PKG_OPENCV_LDFLAGS})

#List of OpenCV*.make
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
#Find the modules ( the modules are coded inside the OpenCVConfig.cmake and OpenCVModules.make )
#-----------
#set(OpenCV_DIR "${CMAKE_SOURCE_DIR}/../libs/opencv-install/share/OpenCV")

#message (STATUS ${Boost_DIR})
#message (STATUS ${OpenCV_DIR})

#if ( OPENCV_FOUND )
#message (STATUS "found OpenCV package")
#endif (OPENCV_FOUND)

#target_link_libraries (Disparity ${OpenCV_LIBS})

# CMAKE_MODULE_PATH -> where to find the packages
# CMAKE_PREFIX_PATH -> where to find the packages
#find_library(OpenCV_CORE NAMES libopencv_core.so HINTS "/local/usr/local/lib/")
#find_library(OpenCV_HIGHGUI NAMES libopencv_highgui.so HINTS "/local/usr/local/lib/")
#find_library(OpenCV_IMGPROC NAMES libopencv_imgproc.so HINTS "/local/usr/local/lib/")
#find_library(OpenCV_IMGCODECS NAMES libopencv_imgcodecs.so HINTS "/local/usr/local/lib/")
#include_directories(${OpenCV_INCLUDE_DIRS})
