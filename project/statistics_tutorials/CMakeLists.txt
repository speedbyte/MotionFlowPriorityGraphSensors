

cmake_minimum_required(VERSION 2.8)
project(statistics-tutorials)

set(CMAKE_CXX_STANDARD 11)


find_package(OpenCV EXACT 3.2.0 COMPONENTS core REQUIRED)
find_package(Boost EXACT 1.64 COMPONENTS filesystem iostreams REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(statistics DeGroot.cpp)
target_link_libraries(statistics ${OpenCV_LIBS} ${Boost_LIBRARIES})