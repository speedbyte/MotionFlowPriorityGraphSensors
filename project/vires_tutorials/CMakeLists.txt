cmake_minimum_required(VERSION 3.8)
project(vires_tutorial)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(Boost EXACT 1.64 COMPONENTS filesystem REQUIRED)
find_package(Qt5 EXACT 5.5.1 COMPONENTS Core REQUIRED)

message("Qt5 ${Qt5_FOUND} ${Qt5Core_INCLUDE_DIRS} ${Qt5Core_LIBS}")

set(SOURCE_FILES
        VTDSource/bosch_adsim/src/rdb_camera_sensor.cpp
        VTDSource/bosch_adsim/src/rdb_codec.cpp
        VTDSource/bosch_adsim/src/rdb_transceiver_shared_memory.cpp
        VTDSource/bosch_adsim/src/rdb_transceiver_tcp.cpp
        #VTDSource/bosch_adsim/src/scp_transceiver.cpp
        VTDSource/RDBHandler.cc
        vires_wrapper/src/VTDConnector.cpp
        main.cpp
        )

include_directories(${CMAKE_SOURCE_DIR}/VTDSource/vires)
include_directories(${CMAKE_SOURCE_DIR}/VTDSource/bosch_adsim/include)
include_directories(${CMAKE_SOURCE_DIR}/vires_wrapper/include)
include_directories(${QT4_INCLUDE_DIR})

add_executable(VTD ${SOURCE_FILES})

target_link_libraries(VTD pthread ${Boost_LIBRARIES} Qt5::Core)
