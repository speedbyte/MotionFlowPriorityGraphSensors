cmake_minimum_required(VERSION 3.8)
project(main)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_FLAGS_DEBUG -D_DEBUG)

find_package(OpenCV EXACT 3.2.0 COMPONENTS core highgui imgcodecs video REQUIRED)
find_package(Boost EXACT 1.64 COMPONENTS filesystem REQUIRED)

set(SOURCE_FILES
        Introduction.h
        presentation_summerschool.cpp noise.cpp run_configuration.cpp kitti_calibration.cpp InputOutput.h)

add_executable(main ${SOURCE_FILES})
target_link_libraries(main ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})