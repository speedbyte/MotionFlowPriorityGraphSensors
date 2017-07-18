cmake_minimum_required(VERSION 3.5)
project(VtdSource)

set(CMAKE_CXX_STANDARD 11)

include_directories(${CMAKE_SOURCE_DIR})
add_executable(VTDSource VTDSource.cpp RDBHandler.cc VTDConnector.cpp)


find_package(PkgConfig REQUIRED)

set(ENV{PKG_CONFIG_PATH}  "${CMAKE_SOURCE_DIR}/../../libs/opencv-install/lib/pkgconfig")
set(ENV{PKG_CONFIG_PATH}  "$ENV{PKG_CONFIG_PATH}:${CMAKE_SOURCE_DIR}/../../libs/boost-install/lib/pkgconfig")
set(ENV{PKG_CONFIG_PATH}  "$ENV{PKG_CONFIG_PATH}:${CMAKE_SOURCE_DIR}/../../libs/ffmpeg-install/lib/pkgconfig")

pkg_search_module(pkg_boost REQUIRED boost)
pkg_search_module(pkg_opencv REQUIRED opencv)
pkg_search_module(pkg_ffmpeg_libavcodec REQUIRED libavcodec)
pkg_search_module(pkg_ffmpeg_libswresample REQUIRED libswresample)


include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_BINARY_DIR})
include_directories(${pkg_boost_INCLUDE_DIRS})
include_directories(${pkg_opencv_INCLUDE_DIRS})
include_directories(${pkg_ffmpeg_libavcodec_INCLUDE_DIRS})
include_directories(${pkg_ffmpeg_libswresample_INCLUDE_DIRS})
include_directories("${CMAKE_SOURCE_DIR}/../../utils/gnuplot-iostream")
include_directories(include)



