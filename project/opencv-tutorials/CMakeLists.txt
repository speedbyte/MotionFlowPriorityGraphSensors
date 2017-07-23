cmake_minimum_required(VERSION 3.8)
project(opencv_tutorial)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/bin")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/../../libs/opencv-install/share/OpenCV/")

set(BOOST_ROOT "${CMAKE_SOURCE_DIR}/../../libs/boost-install")

find_package(OpenCV HINTS "${CMAKE_SOURCE_DIR}/somepath" REQUIRED)
find_package(Boost COMPONENTS system)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

message("OpenCV ${OpenCV_FOUND} ${OpenCV_INCLUDE_DIRS} ${OpenCV_LIBS}")
message("Boost ${Boost_FOUND} ${Boost_INCLUDE_DIRS} ${Boost_LIBRARY_DIRS} ${Boost_LIBRARIES}")

set(SOURCE_FILES
        main.cpp)

add_executable(opencv_tutorials ${SOURCE_FILES})
target_link_libraries(opencv_tutorials ${OpenCV_LIBS} ${Boost_LIBRARIES} )
install(TARGETS opencv_tutorials DESTINATION bin)