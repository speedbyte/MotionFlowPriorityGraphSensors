cmake_minimum_required(VERSION 3.8)
project(gnuplot_tutorials)

set(CMAKE_CXX_STANDARD 11)

find_package(Boost REQUIRED system iostreams filesystem)
set(SOURCE_FILES
        send2d.cpp)


add_executable(send2d ${SOURCE_FILES})
target_link_libraries(send2d ${Boost_LIBRARIES})

