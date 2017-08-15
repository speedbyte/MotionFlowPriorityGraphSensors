cmake_minimum_required(VERSION 3.5)
project(cpp_tutorials)

set(CMAKE_CXX_STANDARD 11)
message("list of paths for find_file and find_path ${CMAKE_SYSTEM_INCLUDE_PATH}")
message("list of paths for find_library ${CMAKE_SYSTEM_LIBRARY_PATH}")
message("list of paths for find_program ${CMAKE_SYSTEM_PROGRAM_PATH}")

find_package(libuvc REQUIRED)
set(SOURCE_FILES main.cpp)
add_library( tutorials.lib ${SOURCE_FILES})

#set(BOOST_LIBRARYDIR "/usr/local/lib/")
find_package(Boost REQUIRED system filesystem)

include_directories(${Boost_INCLUDE_DIRS})
message("include directory for boost ist ${Boost_INCLUDE_DIRS}")

aux_source_directory(. SOURCE_FILES)
MESSAGE("   source files: ${SOURCE_FILES}")

add_executable(overloading ${SOURCE_FILES})
target_link_libraries(overloading ${Boost_LIBRARIES})

set(CMAKE_MODULE_PATH  "${CMAKE_SOURCE_DIR}")
message("CMAKE_MODULE_PATH is ${CMAKE_MODULE_PATH}")
include(CMakeListsDoxygen)

set(ENV{PKG_CONFIG_PATH}  "$ENV{PKG_CONFIG_PATH}:${CMAKE_SOURCE_DIR}/../../libs/boost-install/lib/pkgconfig")
