cmake_minimum_required(VERSION 3.9)
project(gnuplot_tutorials)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        demo2.ps)

add_executable(gnuplot_tutorials ${SOURCE_FILES})