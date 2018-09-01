cmake_minimum_required(VERSION 3.8)
project(cpp-tutorials)

set(CMAKE_CXX_STANDARD 11)

find_package(Boost REQUIRED system filesystem serialization)
find_package(OpenCV REQUIRED core)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(sort ${OpenCV_INCLUDE_DIRS})


set(SOURCE_FILES main.cpp Overload.cpp Template.cpp Larger.cpp ClassNamespace.cpp)
add_executable(main ${SOURCE_FILES})
target_link_libraries(main ${Boost_LIBRARIES})

add_executable(stackoverflow stackoverflow.cpp)
target_link_libraries(stackoverflow ${Boost_LIBRARIES})

add_executable(arrays Arrays.cpp)
target_link_libraries(arrays ${Boost_LIBRARIES})

add_executable(random random.cpp)
target_link_libraries(random ${Boost_LIBRARIES})

add_executable(bigO bigO.cpp)
target_link_libraries(bigO ${Boost_LIBRARIES})

add_executable(filesystem filesystem.cpp)
target_link_libraries(filesystem ${Boost_LIBRARIES})

add_executable(time time.cpp)
target_link_libraries(time ${Boost_LIBRARIES})

add_executable(boost_try boost_try.cpp)
target_link_libraries(boost_try ${Boost_LIBRARIES})

add_executable(ClassTutorials ClassTutorials.cpp)
target_link_libraries(ClassTutorials)

add_executable(sort sort.cpp)
target_link_libraries(sort ${OpenCV_LIBRARIES})

add_executable(set set.cpp)
target_link_libraries(set )

add_executable(save_object SaveBinaryFile.cpp)
target_link_libraries(save_object ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})


set(CMAKE_MODULE_PATH  "${CMAKE_SOURCE_DIR}")
message("CMAKE_MODULE_PATH is ${CMAKE_MODULE_PATH}")
include(CMakeListsDoxygen)

