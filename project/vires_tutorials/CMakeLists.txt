cmake_minimum_required(VERSION 3.8)
project(VTD)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_VERBOSE_MAKEFILE ON)

message("source dir - " ${CMAKE_SOURCE_DIR})

include_directories(${CMAKE_SOURCE_DIR}/VTDSource/bosch_adsim/include)

set(SOURCE_FILES
        VTDSource/bosch_adsim/src/rdb_camera_sensor.cpp
        VTDSource/bosch_adsim/src/rdb_codec.cpp
        VTDSource/bosch_adsim/src/rdb_transceiver_shared_memory.cpp
        VTDSource/bosch_adsim/src/rdb_transceiver_tcp.cpp
        VTDSource/bosch_adsim/src/scp_transceiver.cpp
        VTDSource/RDBHandler.cc )

add_executable(VTD ${SOURCE_FILES})
