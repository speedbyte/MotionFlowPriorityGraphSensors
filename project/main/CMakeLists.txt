cmake_minimum_required(VERSION 3.8)
project(main)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        Introduction.h
        presentation_summerschool.cpp)

add_executable(main ${SOURCE_FILES})