cmake_minimum_required(VERSION 3.8)
project(VTD)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        VTDSource/bosch_adsim/include/adsim/vtd/rdb_camera_sensor.h
        VTDSource/bosch_adsim/include/adsim/vtd/rdb_codec.h
        VTDSource/bosch_adsim/include/adsim/vtd/rdb_transceiver.h
        VTDSource/bosch_adsim/include/adsim/vtd/rdb_transceiver_shared_memory.h
        VTDSource/bosch_adsim/include/adsim/vtd/rdb_transceiver_tcp.h
        VTDSource/bosch_adsim/include/adsim/vtd/scp_transceiver.h
        VTDSource/bosch_adsim/src/rdb_camera_sensor.cpp
        VTDSource/bosch_adsim/src/rdb_codec.cpp
        VTDSource/bosch_adsim/src/rdb_transceiver_shared_memory.cpp
        VTDSource/bosch_adsim/src/rdb_transceiver_tcp.cpp
        VTDSource/bosch_adsim/src/scp_transceiver.cpp
        VTDSource/CMakeLists.txt
        VTDSource/Container.h
        VTDSource/RDBHandler.cc
        VTDSource/RDBHandler.hh
        VTDSource/viRDBIcd.h
        VTDSource/VTDBroadcastComm.cpp
        VTDSource/VTDBroadcastComm.h
        VTDSource/VTDConnector.cpp
        VTDSource/VTDConnector.h
        VTDSource/VTDSource.cpp
        VTDSource/VTDSource.h
        VTDSource/VTDSource.pro)

add_executable(VTD ${SOURCE_FILES})