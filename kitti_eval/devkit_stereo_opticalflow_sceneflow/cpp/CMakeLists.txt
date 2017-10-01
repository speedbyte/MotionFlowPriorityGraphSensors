cmake_minimum_required(VERSION 3.8)
project(devkit_stereo_opticalflow_sceneflow)

set(CMAKE_CXX_STANDARD 11)


set(SOURCE_FILES
        evaluate_flow.cpp
        #evaluate_scene_flow.cpp
        )

add_executable(evaluate_flow ${SOURCE_FILES})
target_link_libraries(evaluate_flow png)