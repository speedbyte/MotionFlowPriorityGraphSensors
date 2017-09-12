cmake_minimum_required(VERSION 3.8)
project(vires_tutorial)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_VERBOSE_MAKEFILE ON)

include_directories(${CMAKE_SOURCE_DIR}/bosch_adsim/include)
include_directories(${CMAKE_SOURCE_DIR}/vires_source/include)

set(SOURCE_FILES
        bosch_adsim/src/rdb_camera_sensor.cpp
        bosch_adsim/src/rdb_codec.cpp
        bosch_adsim/src/rdb_transceiver_shared_memory.cpp
        bosch_adsim/src/rdb_transceiver_tcp.cpp
        bosch_adsim/src/scp_transceiver.cpp
        vires_source/src/RDBHandler.cc )

add_executable(VTD ${SOURCE_FILES})
