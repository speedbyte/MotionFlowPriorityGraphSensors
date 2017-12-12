cmake_minimum_required(VERSION 3.8)
project(vires_tutorial)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -fPIC)
find_package(Boost EXACT 1.64 COMPONENTS filesystem REQUIRED)
find_package(Qt5 EXACT 5.5.1 COMPONENTS Core Gui REQUIRED)

message("Qt5 ${Qt5_FOUND} ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Core_LIBS}")

set(SOURCE_FILES
        # I am using vires RDB implementation
        vires/RDBHandler.cc
        kbhit.cpp vires_common.cpp #main.cpp
        # I am not using bosch rdb implementation because there is no substatial difference between vires and bosch.
        #bosch_adsim/src/rdb_camera_sensor.cpp
        #bosch_adsim/src/rdb_codec.cpp
        #bosch_adsim/src/rdb_transceiver_shared_memory.cpp
        #bosch_adsim/src/rdb_transceiver_tcp.cpp
        #bosch_adsim/src/scp_transceiver.cpp
        )

include_directories(${CMAKE_SOURCE_DIR}/vires)
include_directories(${CMAKE_SOURCE_DIR}/bosch_adsim/include)
include_directories(${QT5_INCLUDE_DIR})
include_directories(${Qt5Gui_INCLUDE_DIRS})

add_library(viresrdb SHARED ${SOURCE_FILES})
#target_link_libraries(shm_trigger_read pthread ${Boost_LIBRARIES} png)


