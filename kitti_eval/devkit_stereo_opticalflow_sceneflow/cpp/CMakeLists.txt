cmake_minimum_required(VERSION 3.8)
project(devkit_stereo_opticalflow_sceneflow)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb3")

set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/../../../libs/kitti-install)

set(SOURCE_FILES
        evaluate_flow.cpp
        #evaluate_scene_flow.cpp
        )

file(GLOB KITTI_HEADERS "*.h")

add_library(kitti ${SOURCE_FILES})
target_link_libraries(kitti png)

install(TARGETS kitti DESTINATION lib )
install(FILES ${KITTI_HEADERS} DESTINATION include/kitti)