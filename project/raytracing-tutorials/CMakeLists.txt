cmake_minimum_required(VERSION 3.8)
project(raytracing_tutorials)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb3")

find_package(Boost EXACT 1.64 COMPONENTS system REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(raytracing main.cpp)
target_link_libraries(raytracing ${Boost_LIBRARIES})
