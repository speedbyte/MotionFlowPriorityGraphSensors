cmake_minimum_required(VERSION 3.8)
project(main)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -D_DEBUG -ggdb3  -Wsign-conversion")

find_package(OpenCV EXACT 3.2.0 COMPONENTS core highgui videoio imgcodecs video REQUIRED)
find_package(Boost EXACT 1.64 COMPONENTS filesystem iostreams REQUIRED)

#configure_file(Example.h.in Example.h)
#set(VERSION "1.0")
#set(NUMBER 3)
#In example.h.in
##define VERSION "${VERSION}"
##define NUMBER  ${NUMBER}

set(SOURCE_FILES
        KittiSubmitInterface.cpp ObjectMetaData.cpp main.cpp run_configuration.cpp kitti_calibration.cpp
        video_capture.cpp GroundTruthFlow.cpp AlgorithmFlow.cpp kbhit.cpp Dataset.cpp PlotFlow.cpp
        GenerateGroundTruthScene.cpp GenerateGroundTruthSceneInternal.cpp GenerateGroundTruthSceneExternal.cpp GroundTruthObjects.cpp Noise.cpp Noise.h SensorImage.cpp
        SensorImage.h FlowImageExtended.cpp RobustnessIndex.cpp RobustnessIndex.h OpticalFlow.cpp OpticalFlowCollisionVector.cpp OpticalFlowAnalysis.cpp OpticalFlowOcclusion.cpp OpticalFlowPlotter.cpp OpticalFlow.h SimulatedObjects.cpp SimulatedObjects.h Objects.cpp Objects.h Sensors.cpp SensorMetaData.h Utils.h DataProcessingAlgorithm.cpp DataProcessingAlgorithm.h OpticalFlow_later.cpp GenerateViresObjects.cpp GenerateViresObjects.h GenerateCppObjects.cpp BasicObjects.cpp PixelRobustness.cpp VectorRobustness.cpp SortandIntersect.h InterpolateData.cpp InterpolateData.h ObjectImageShapeData.cpp ObjectImageShapeData.h OpticalFlowStichImages.cpp)

add_executable(main ${SOURCE_FILES})
target_link_libraries( main ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} png viresrdb)
