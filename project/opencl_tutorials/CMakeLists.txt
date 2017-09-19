cmake_minimum_required(VERSION 3.9)
project(opencl_tutorials)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES oclVectorAdd/NVIDIA_GPU_Computing_SDK/OpenCL/src/oclVectorAdd/oclVectorAdd.cpp)

add_executable(opencl_tutorials ${SOURCE_FILES})