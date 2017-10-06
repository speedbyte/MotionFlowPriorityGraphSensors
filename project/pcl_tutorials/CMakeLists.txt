cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_visualizer_viewports)

find_package(PCL EXACT 1.8.1.99 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_visualizer_demo pcl_visualisation_demo.cpp)
message("Libraries: " ${PCL_LIBRARIES})

target_link_libraries (pcl_visualizer_demo2 ${PCL_LIBRARIES} pcl_visualization)