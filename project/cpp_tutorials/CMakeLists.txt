cmake_minimum_required(VERSION 3.8)
project(cpp-tutorials)

set(CMAKE_CXX_STANDARD 11)

find_package(Boost REQUIRED system filesystem)

include_directories(${Boost_INCLUDE_DIRS})

set(SOURCE_FILES main.cpp Overload.cpp Template.cpp Larger.cpp)
add_executable(overloading ${SOURCE_FILES})
target_link_libraries(overloading ${Boost_LIBRARIES})

add_executable(stackoverflow stackoverflow.cpp)
target_link_libraries(stackoverflow ${Boost_LIBRARIES})

add_executable(arrays Arrays.cpp)
target_link_libraries(arrays ${Boost_LIBRARIES})

add_executable(random random.cpp)
target_link_libraries(random ${Boost_LIBRARIES})

add_executable(bigO bigO.cpp)
target_link_libraries(bigO ${Boost_LIBRARIES})

add_executable(filesystem filesystem.cpp)
target_link_libraries(filesystem ${Boost_LIBRARIES})

set(CMAKE_MODULE_PATH  "${CMAKE_SOURCE_DIR}")
message("CMAKE_MODULE_PATH is ${CMAKE_MODULE_PATH}")
include(CMakeListsDoxygen)

