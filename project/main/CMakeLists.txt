cmake_minimum_required(VERSION 3.8)
project(main)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -D_DEBUG -ggdb3")

find_package(OpenCV EXACT 3.2.0 COMPONENTS core highgui videoio imgcodecs video REQUIRED)
find_package(Boost EXACT 1.64 COMPONENTS filesystem iostreams REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/../../kitti_eval)

#set(SOURCE_FILES_KITTI ../../kitti_eval/devkit_stereo_opticalflow_sceneflow/cpp/evaluate_flow.cpp )
set(SOURCE_FILES
        main.cpp run_configuration.cpp kitti_calibration.cpp video_capture.cpp groundTruth.cpp flow.cpp )

add_executable(main ${SOURCE_FILES})
target_link_libraries(main ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} png kitti )

