cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_visualizer_viewports)

find_package(PCL EXACT 1.8.0 REQUIRED)
find_package(VTK EXACT 7.0.0 COMPONENTS REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${VTK_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})

message(${VTK_INCLUDE_DIRS})
message(${PCL_INCLUDE_DIRS})
message("Libraries: " ${PCL_LIBRARIES})

add_executable (pcl_visualizer_demo pcl_visualisation_demo.cpp)
target_link_libraries (pcl_visualizer_demo ${PCL_LIBRARIES} ${VTK_LIBRARIES} pcl_visualization)