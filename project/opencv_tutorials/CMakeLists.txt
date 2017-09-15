cmake_minimum_required(VERSION 3.8)
project(opencv_tutorial)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -D_DEBUG -ggdb3")


#set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/bin")
#list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/../../libs/opencv-install/share/OpenCV/")

#set(BOOST_ROOT "${CMAKE_SOURCE_DIR}/../../libs/boost-install")

#add_definitions(-DWITH_OPENCV2)
#target_compile_definitions(my_target PRIVATE FOO=1 BAR=1)
#set_source_files_properties(foo.cpp PROPERTIES COMPILE_DEFINITIONS -DFOO=1)

find_package(OpenCV COMPONENTS core highgui imgcodecs imgproc REQUIRED)
find_package(Boost COMPONENTS system filesystem iostreams REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

message("OpenCV ${OpenCV_FOUND} ${OpenCV_INCLUDE_DIRS} ${OpenCV_LIBRARIES}")
message("Boost ${Boost_FOUND} ${Boost_INCLUDE_DIRS}  ${Boost_LIBRARIES}")


add_executable(opencv_tutorials main.cpp)
target_link_libraries(opencv_tutorials ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )
add_executable(contour contour.cpp)
target_link_libraries(contour ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )
add_executable(core core.cpp)
target_link_libraries(core ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )
add_executable(draw draw.cpp)
target_link_libraries(draw ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )
add_executable(mat_operations matOperations.cpp)
target_link_libraries(mat_operations ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )
add_executable(pimple pimple.cpp)
target_link_libraries(pimple ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )
add_executable(maths maths.cpp)
target_link_libraries(maths ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )
add_executable(image imageOperations.cpp)
target_link_libraries(image ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )
add_executable(filter filterOperations.cpp)
target_link_libraries(filter ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )
add_executable(data_persistence data_persistence.cpp)
target_link_libraries(data_persistence ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )
