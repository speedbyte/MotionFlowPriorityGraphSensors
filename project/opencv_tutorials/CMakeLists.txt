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

find_package(OpenCV COMPONENTS core highgui imgcodecs imgproc videoio video calib3d REQUIRED)
find_package(Boost COMPONENTS system filesystem iostreams REQUIRED)

#opencv_core opencv_imgproc opencv_flann
#opencv_imgcodecs opencv_videoio opencv_highgui opencv_ml opencv_video
#opencv_objdetect opencv_photo opencv_features2d opencv_calib3d
#opencv_stitching opencv_videostab opencv_shape ${OPENCV_MODULES_PUBLIC} ${OpenCV_LIB_COMPONENTS}

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})


find_path(SAMPLE_DIR NAMES kalman.cpp HINTS ${CMAKE_SOURCE_DIR}/opencv-samples/cpp NO_DEFAULT_PATH)
message(${SAMPLE_DIR})

message("OpenCV ${OpenCV_FOUND} ${OpenCV_INCLUDE_DIRS} ${OpenCV_LIBRARIES}")
message("Boost ${Boost_FOUND} ${Boost_INCLUDE_DIRS}  ${Boost_LIBRARIES}")

file(GLOB_RECURSE cpp_samples RELATIVE src *.cpp)

macro(ocv_list_filterout lst regex)
    foreach(item ${${lst}})
        if(item MATCHES "${regex}")
            list(REMOVE_ITEM ${lst} "${item}")
        endif()
    endforeach()
endmacro()

ocv_list_filterout(cpp_samples Qt_sample)
ocv_list_filterout(cpp_samples "/viz/")

message("files ${cpp_samples}")

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

add_executable(imageOperations imageOperations.cpp)
target_link_libraries(imageOperations ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(filter filterOperations.cpp)
target_link_libraries(filter ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(data_persistence data_persistence.cpp)
target_link_libraries(data_persistence ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(video_operations video_operations.cpp)
target_link_libraries(video_operations ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(kalman ${SAMPLE_DIR}/kalman.cpp)
target_link_libraries(kalman ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(simpleflow ${SAMPLE_DIR}/simpleflow_demo.cpp)
target_link_libraries(simpleflow ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(birdview birdview.cpp)
target_link_libraries(birdview ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(pinhole pinhole.cpp)
target_link_libraries(pinhole ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(camera_calibration camera_calibration.cpp)
target_link_libraries(camera_calibration ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(transform facePoseEstimation.cpp)
target_link_libraries(transform ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )

