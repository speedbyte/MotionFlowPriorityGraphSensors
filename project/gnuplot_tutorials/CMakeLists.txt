cmake_minimum_required(VERSION 3.8)
project(gnuplot_tutorials)

set(CMAKE_CXX_STANDARD 11)

find_package(Boost REQUIRED system iostreams filesystem)

add_executable(send2d send2d.cpp)
target_link_libraries(send2d ${Boost_LIBRARIES})

add_executable(example-2d example-data-2d.cc)
target_link_libraries(example-2d ${Boost_LIBRARIES})
