#!/bin/bash

#rm and then add --force rewrites the submodule index.

#deactivate
function remove_submodule
{
    module="devkits/MPI-Sintel-testing_devkit/" 
    git submodule deinit $module
    git rm --cached $module
    rm -rf $module
}

#remove_submodule

#git submodule add --name "VTDSource" /local/internal/VTDSource.git internal/VTDSource
#git submodule add --name "VIRES" /local/internal/VIRES.git internal/VIRES
#git submodule add --name "vires-interface" /local/internal/vires-interface.git internal/vires-interface
#git submodule add --name "VirtualTestDriveFramework" /local/internal/VirtualTestDriveFramework.git internal/VirtualTestDriveFramework
git submodule add --name "overleaf-maths" https://git.overleaf.com/5c4357c779d0c14d49735cbc documentation/overleaf/maths
git submodule add --name "overleaf-marcelBA" https://git.overleaf.com/5d5066e9d655257fb6a03c41 documentation/overleaf/marcelBA
git submodule add --name "cockpit" https://git.overleaf.com/5cb78521e3d1df2ed2cbf4da documentation/overleaf/cockpit
git submodule add --name "overleaf/paper_1" https://git.overleaf.com/5cb787fae3d1df2ed2cbf619 documentation/overleaf/paper_1
#git submodule add --name "rob_devkit_flow" /local/internal/rob_devkit_flow.git submodules/devkits/rob_devkit_flow #http://cvlibs.net:3000/ageiger/rob_devkit.git
#git submodule add --name "rob_devkit_depth" https://github.com/joseph-zhong/KITTI-devkit submodules/devkits/rob_devkit_depth
#git submodule add --name "QtKittiVisualizer" https://github.com/MarkMuth/QtKittiVisualizer.git submodules/devkits/QtKittiVisualizer
#git submodule add --name "kitti-pcl" https://github.com/yanii/kitti-pcl.git submodules/devkits/kitti-pcl
#git submodule add --name "kitti2pcl" https://github.com/jaejunlee0538/kitti2pcl.git submodules/devkits/kitti2pcl
#git submodule add --name "pcl_manipulation" https://github.com/Benzarti-Ilyess/pcl_manipulation.git submodules/algorithms/pcl_manipulation
git submodule add --name "kitti_devkit_stereo_opticalflow_sceneflow" https://github.com/speedbyte/kitti_devkit_stereo_opticalflow_sceneflow.git submodules/devkits/kitti_devkit_stereo_opticalflow_sceneflow
#git submodule add --name "pkg-config-generator" https://github.com/nmante/pkg-config-generator.git submodules/utils/pkg-config-generator
#git submodule add --name "PRSM" https://github.com/vogechri/PRSM submodules/algorithms/PRSM
git submodule add --name "middlebury_devkit_flow"  https://github.com/speedbyte//middlebury_devkit_flow.git submodules/devkits/middlebury_devkit_flow
git submodule add --name "MPI-Sintel-testing_devkit" https://github.com/speedbyte/MPI-Sintel-testing_devkit.git submodules/devkits/MPI-Sintel-testing_devkit
git submodule add --name "CARLA" https://github.com/carla-simulator/carla.git submodules/simulator/CARLA
#git submodule add --name "UnrealEngine" --force https://github.com/EpicGames/UnrealEngine.git submodules/simulator/UnrealEngine
git submodule add --name "CarlaUnreal" --depth 1 https://github.com/CarlaUnreal/UnrealEngine.git submodules/simulator/CarlaUnreal
#git submodule add --name "ffmpeg" https://git.ffmpeg.org/ffmpeg.git submodules/libs/ffmpeg
#git submodule add --name "opencv" https://github.com/opencv/opencv.git submodules/libs/opencv
#git submodule add --name "gnuplot-iostream" https://github.com/dstahlke/gnuplot-iostream.git submodules/libs/gnuplot-iostream
#git submodule add --name "protobuf" https://github.com/google/protobuf submodules/libs/protobuf
#git submodule add --name "libuvc" https://github.com/ktossell/libuvc submodules/libs/libuvc
#git submodule add --name "mrpt" https://github.com/MRPT/mrpt.git submodules/libs/mrpt
#git submodule add --name "pcl" https://github.com/PointCloudLibrary/pcl.git submodules/libs/pcl
#git submodule add --name "graphviz" https://github.com/ellson/graphviz.git submodules/libs/graphviz
#git submodule add --name "libs/pngpp" https://github.com/bysreg/pngpp.git submodules/libs/pngpp
git submodule add --name "vtk" https://gitlab.kitware.com/vtk/vtk.git submodules/libs/vtk
#git submodule add --name "pix2pixHD" https://github.com/NVIDIA/pix2pixHD.git submodules/libs/pix2pixHD
#git submodule add --name "opencv_contrib" https://github.com/opencv/opencv_contrib.git submodules/libs/opencv_contrib
#git submodule add --name "boost" https://github.com/boostorg/boost.git submodules/libs/boost
