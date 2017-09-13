cmake_minimum_required(VERSION 3.8)
project(vires_tutorial)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(Boost EXACT 1.64 COMPONENTS filesystem REQUIRED)
find_package(Qt5 EXACT 5.5.1 COMPONENTS Core REQUIRED)

message("Qt5 ${Qt5_FOUND} ${Qt5Core_INCLUDE_DIRS} ${Qt5Core_LIBS}")

set(SOURCE_FILES
        bosch_adsim/src/rdb_camera_sensor.cpp
        bosch_adsim/src/rdb_codec.cpp
        bosch_adsim/src/rdb_transceiver_shared_memory.cpp
        bosch_adsim/src/rdb_transceiver_tcp.cpp
        #bosch_adsim/src/scp_transceiver.cpp
        vires_source/src/RDBHandler.cc
        vires_wrapper/src/VTDConnector.cpp
        main.cpp
        )

add_executable(VTD ${SOURCE_FILES})

target_include_directories(VTD PUBLIC ${CMAKE_SOURCE_DIR}/bosch_adsim/include)
target_include_directories(VTD PUBLIC ${CMAKE_SOURCE_DIR}/vires_source/include)
target_include_directories(VTD PUBLIC ${CMAKE_SOURCE_DIR}/vires_wrapper/include)
target_include_directories(VTD PUBLIC ${QT4_INCLUDE_DIR})

target_link_libraries(VTD pthread ${Boost_LIBRARIES} Qt5::Core)
