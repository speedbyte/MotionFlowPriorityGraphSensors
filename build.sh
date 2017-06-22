#!/bin/bash

while getopts ":t:bfolpcier:dah" opt; do
  case $opt in
    t)
      echo "-t was triggered, Parameter: $OPTARG" >&2
      BUILD_OPTION=$OPTARG
      ;;
    b)
      echo "-b was triggered, Parameter: $OPTARG" >&2
      BOOST_OPTION="y"
      ;;
    f)
      echo "-f was triggered, Parameter: $OPTARG" >&2
      FFMPEG_OPTION="y"
      ;;
    o)
      echo "-o was triggered, Parameter: $OPTARG" >&2
      OPENCV_OPTION="y"
      ;;
    l)
      echo "-l was triggered, Parameter: $OPTARG" >&2
      LIBSVM_OPTION="y"
      ;;
    p)
      echo "-p was triggered, Parameter: $OPTARG" >&2
      PROJECT_OPTION="y"
      ;;
    c)
      echo "-c was triggered, Parameter: $OPTARG" >&2
      CAFFE_OPTION="y"
      ;;
    i)
      echo "-i was triggered, Parameter: $OPTARG" >&2
      IVT_OPTION="y"
      ;;
    e)
      echo "-e was triggered, Parameter: $OPTARG" >&2
      EXTERNAL_OPTION="y"
      ;;
    r)
      echo "-r was triggered, Parameter: $OPTARG" >&2
      PROJECT_RUN_OPTION="y"
      PROTOCONFIG=$OPTARG
      ;;
    a)
      echo "-a was triggered, Parameter: $OPTARG" >&2
      BOOST_OPTION="y"
      FFMPEG_OPTION="y"
      OPENCV_OPTION="y"
      LIBSVM_OPTION="y"
      PROJECT_OPTION="y"
      CAFFE_OPTION="y"
      IVT_OPTION="y"
      EXTERNAL_OPTION="y"
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
    h)
      echo "Options are \
      -tbfolpacieah"
      echo "Option -t means type and has parameters either manual or clean"
      echo "Option -b is meant for boost"
      echo "Option -f is meant for ffmpeg"
      echo "Option -o is meant for opencv"
      echo "Option -l is meant for libsvm"
      echo "Option -p is meant for project"
      echo "Option -c is meant for caffe"
      echo "Option -i is meant for ivt"
      echo "Option -e is meant for external"
      echo "Option -a is meant for all"
      echo "Option -h is meant for this help"
      echo "The same options are valid with -t clean for example <-t clean -o> will only clean opencv whereas <-t clean -a> will clean all"
      exit 1
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
  esac
done

if [ $# -eq 0 ]; then
    echo "The script takes multiple arguement, 0 given. Please type -h to see the help file"
    echo "Continuing the script with default option -a"
    OPENCV_OPTION="y"
    FFMPEG_OPTION="y"
    LIBSVM_OPTION="y"
    BOOST_OPTION="y"
    PROJECT_OPTION="y"
    CAFEE_OPTION="y"
    IVT_OPTION="y"
    EXTERNAL_OPTION="y"
fi


if command -v qmake-qt5; then
    QMAKE=qmake-qt5
else
    QMAKE=qmake
fi


function exit_function
{
    tput setf 7;
    echo "exiting with grace";
    exit 1;
}

function enter_boost_fn
{
cd $SOURCE_DIR
if [ "$BOOST_OPTION" == "y" ] ; then
    tput setf 3
    cd $BOOST_PWD
    echo "Building in $(pwd)"
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue"; fi
    if [ "$BUILD_OPTION" == "clean" ]; then echo "cleaning boost ...."; ./b2 --clean; rm -rf bin.v2; cd ../../; return; fi
    ./bootstrap.sh --prefix=$BOOST_PWD/../boost-install/
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue"; fi
    ./b2 cxxflags="-Wno-unused-local-typedefs -Wstrict-aliasing" install
#    if [ ! -d stage ] ; then
#        if [ ! -f b2 ]; then
#        ./bootstrap.sh --prefix=$BOOST_PWD/../boost-install/
#        fi
#       ./b2 cxxflags="-Wno-unused-local-typedefs -Wstrict-aliasing" install
#    fi
    #ln -s ./boost ./stage/include
    mkdir -p $BOOST_PWD/../boost-install/lib/pkgconfig
    python $SOURCE_DIR/utils/pkg-config-generator/main.py -n Boost -v 1.64.0 -p $BOOST_PWD/../boost-install -o $BOOST_PWD/../boost-install/lib/pkgconfig/boost.pc $BOOST_PWD/../boost-install/lib/
#python main.py -n Boost -v 1.63.0 -p $BOOST_PWD/stage -o ./stage/lib/pkgconfig/boost.pc ./stage/lib/
    cd $SOURCE_DIR
fi
}

function enter_ffmpeg_fn
{
cd $SOURCE_DIR
if [ "$FFMPEG_OPTION" == "y" ] ; then
    tput setf 3
    cd $FFMPEG_PWD
    echo "Building in $(pwd)"
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue"; fi
    if [ "$BUILD_OPTION" == "clean" ]; then echo "cleaning ffmpeg ...."; make distclean; rm -rf $FFMPEG_PWD/../ffmpeg-install/*; cd ../../; return; fi
    echo "Configuring ffmpeg"
    cd $FFMPEG_PWD
    CXXFLAGS="-D__STDC_CONSTANT_MACROS -Wdeprecated-declarations -fPIC" ./configure --prefix="$FFMPEG_PWD/../ffmpeg-install" --bindir="./bin" --enable-shared
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue"; fi
    time make > /dev/null #2>&1
    ret=$(echo $?)
    echo "make in $SOURCE_DIR returned $ret"
    if [ "$ret" == "0" ]; then echo "ffmpeg make successful"; else echo "ffmpeg build terminated with error"; rm -rf build; rm -f config.mak; exit_function; fi
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue"; fi
    time make install >> /dev/null #2>&1
    ret=$(echo $?)
    echo "make-install in $SOURCE_DIR returned $ret"
    if [ "$ret" == "0" ]; then echo "ffmpeg make-install successful"; else echo "ffmpeg make-install terminated with error. Please see the /dev/null file "; exit_function; fi
    cd $SOURCE_DIR
fi
}

function enter_opencv_fn
{
cd $SOURCE_DIR
if [ "$OPENCV_OPTION" == "y" ]; then
    tput setf 2 
    cd $OPENCV_PWD
    echo "Building in $(pwd)"
    if [ "$BUILD_OPTION" == "clean" ]; then echo "cleaning opencv ...."; rm -rf release; rm -rf $FFMPEG_PWD/../opencv-install/*; cd ../../; return; fi
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue";  fi
    echo "Check compatibility"
    if [[ $PKG_CONFIG_PATH_INCLUDE_FFMPEG == "-I$ACTUAL_FFMPEG_INCLUDE_PATH"* ]]; then echo "Correct PKG_CONFIG_PATH_INCLUDE_FFMPEG $PKG_CONFIG_PATH_INCLUDE_FFMPEG"; else echo "Incorrect PKG_CONFIG_PATH_INCLUDE_FFMPEG $PKG_CONFIG_PATH_INCLUDE_FFMPEG"; exit_function; fi
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue";  fi
    echo "Configuring opencv"
    #export PKG_CONFIG_LIBDIR=$PKG_CONFIG_LIBDIR:$SOURCE_DIR/../ffmpeg/build/lib/
    export LD_LIBRARY_PATH=$FFMPEG_PWD/../ffmpeg-install/lib/
    echo $(printenv | grep PKG_CONFIG_PATH)
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue";  fi
    echo "In order to use the opencv 3rd party versions of libraries instead of system ones on UNIX systems you
    should use BUILD_<library_name> CMake flags (for example, -D BUILD_PNG for the libpng library)."
    mkdir -p release; cd release
    cmake -Wno-dev -Wl,-rpath=$FFMPEG_PWD/../ffmpeg-install/lib/ -D ENABLE_PRECOMPILED_HEADERS=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=../../opencv-install -D WITH_GTK=ON ..
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue";  fi
    time make > /dev/null #2>&1
    ret=$(echo $?)
    if [ "$ret" == "0" ]; then echo "opencv make successful"; else echo "opencv make terminated with error. Please see the /dev/null file"; exit_function; fi
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue"; fi
    time make install >> /dev/null #2>&1
    ret=$(echo $?)
    echo "make-install in $SOURCE_DIR returned $ret"
    if [ "$ret" == "0" ]; then echo "opencv make-install successful"; else echo "opencv make-install terminated with error. Please see the /dev/null file "; exit_function; fi
    cd $SOURCE_DIR
fi
}

function enter_libsvm_fn
{
cd $SOURCE_DIR
LIBSVM_PWD="$SOURCE_DIR/libs/libsvm"
if [ "$LIBSVM_OPTION" == "y" ]; then
    tput setf 3 
    cd $LIBSVM_PWD
    echo "Building in $(pwd)"
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue"; fi
    if [ "$BUILD_OPTION" == "clean" ]; then echo "cleaning libsvm ...."; make clean; cd ../../; return; fi
    echo "Making libsvm"
    cd $LIBSVM_PWD
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue"; fi
    time make lib > /dev/null #2>&1
    ret=$(echo $?)
    echo "make in $SOURCE_DIR returned $ret"
    if [ "$ret" == "0" ]; then echo "libsvm make successful"; else echo "libsvm build terminated with error"; make clean; exit_function; fi
    ln -s libsvm.so.2 libsvm.so # making -lsvm possible
    cd $SOURCE_DIR
fi
}

function enter_project_previous_fn
{
cd $SOURCE_DIR
if [ "$PROJECT_OPTION" == "y" ]; then
    tput setf 3
    cd $PROJECT_PWD
    echo "Building in $(pwd)"
    if [ "$BUILD_OPTION" == "clean" ]; then make clean; cd .; return; fi
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue";  fi
    make distclean # remove all libs and, more importantly, all old Makefiles
    ${QMAKE} Project.pro -recursive #-spec linux-g++-64 CONFIG+=debug
    ret=$(echo $?)
    if [ "$ret" == "0" ]; then echo "project qmake conf successful"; else echo "project make terminated with error. Please see the /dev/null file"; exit_function; fi
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue";  fi
    time make #> /dev/null #2>&1
    ret=$(echo $?)
    if [ "$ret" == "0" ]; then echo "project make successful"; else echo "project make terminated with error. Please see the /dev/null file"; exit_function; fi
    cd $SOURCE_DIR
fi
}

function enter_project_fn
{
cd $SOURCE_DIR
if [ "$PROJECT_OPTION" == "y" ]; then
    tput setf 3
    cd $PROJECT_PWD
    echo "Building in $(pwd)"
    mkdir -p $PROJECT_PWD/cmake-build-debug
    cd $PROJECT_PWD/cmake-build-debug
    if [ "$BUILD_OPTION" == "clean" ]; then  echo "cleaning PerceptionSandbox ...."; rm -rf *; cd .; return; fi
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue";  fi
    # remove all libs and, more importantly, all old Makefiles
    cmake -DOpenCV_DIR="${PROJECT_PWD}/../../libs/opencv-install/share/OpenCV" -DBoost_DIR="${PROJECT_PWD}/../../libs/boost-install/share/FindBoost" ..
    ret=$(echo $?)
    if [ "$ret" == "0" ]; then echo "project cmake conf successful"; else echo "project make terminated with error. Please see the /dev/null file"; exit_function; fi
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue";  fi
    time make #> /dev/null #2>&1
    ret=$(echo $?)
    if [ "$ret" == "0" ]; then echo "project make successful"; else echo "project make terminated with error. Please see the /dev/null file"; exit_function; fi
    time make install #> /dev/null #2>&1
    ret=$(echo $?)
    if [ "$ret" == "0" ]; then echo "project make successful"; else echo "project make terminated with error. Please see the /dev/null file"; exit_function; fi
    cd $SOURCE_DIR
fi
}

function enter_project_run_fn
{
#EXAMPLE="object_saliency"
#EXAMPLE="spectral_saliency"
#EXAMPLE="itti_saliency"
#EXAMPLE="highpass_saliency"
#EXAMPLE="gbvs_saliency"
#EXAMPLE="boolean_maps_saliency"
cd $SOURCE_DIR
if [ "$PROJECT_RUN_OPTION" == "y" ]; then
    tput setf 3
    cd $PROJECT_PWD
    echo "Building in $(pwd)"
    cd $PROJECT_PWD/install/bin
    if [ "$BUILD_OPTION" == "clean" ]; then  echo "cleaning PerceptionSandbox ...."; rm -rf *; cd .; return; fi
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue";  fi
    DIR_FRAMEWORK_ROOT=$PROJECT_PWD #"$(dirname "${BASH_SOURCE[0]}")"
    echo "detected framework root: ${DIR_FRAMEWORK_ROOT}"
    #
    # find protobuild binary
    #
    DIR_BIN="${DIR_FRAMEWORK_ROOT}/bin"
    PROTOBUILD_BIN="${DIR_BIN}/protobuild"
    #
    # prepare configuration
    #
    DIR_CONFIG="${DIR_FRAMEWORK_ROOT}/config"
    DIR_DATA="${DIR_FRAMEWORK_ROOT}/data"
    FILE_CONFIG="${DIR_CONFIG}/${PROTOCONFIG}.prototxt"
    export CPATH="$SOURCE_DIR/libs/boost-install/include/:$SOURCE_DIR/libs/opencv-install/include/:$SOURCE_DIR/utils/gnuplot-iostream"
    #
    # library path
    #
    export LD_LIBRARY_PATH="${DIR_FRAMEWORK_ROOT}/lib/:$SOURCE_DIR/libs/boost-install/lib/:$PROJECT_PWD/install/lib:$SOURCE_DIR/libs/opencv-install/lib/"
    #
    # run pipeline processor
    #
    ./protobuild -f $PROJECT_PWD/install/ -c $FILE_CONFIG -r;
    ret=$(echo $?)
    if [ "$ret" == "0" ]; then echo "project run successful"; else echo "project run terminated with error. Please see the /dev/null file"; exit_function; fi
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue";  fi
    cd $SOURCE_DIR
fi
}

function enter_caffe_fn
{
cd $SOURCE_DIR
PROJECT_LIBS="$SOURCE_DIR/libs"
CAFFE_PWD="$SOURCE_DIR/libs/caffe"
if [ "$CAFFE_OPTION" == "y" ]; then
	cd $CAFFE_PWD
	echo "Building in $(pwd)"
	if [ "$BUILD_OPTION" == "clean" ]; then echo "Cleaning caffe"; make clean; cd .; return; fi
	make all -n PROJECT_LIBS=$PROJECT_LIBS
	ret=$(echo $?)
  if [ "$ret" == "0" ]; then echo "caffe make successful"; else echo "caffe make terminated with error. Please see the /dev/null file"; exit_function; fi
  cd $SOURCE_DIR
fi
}

function enter_ivt_fn
{
cd $SOURCE_DIR
IVT_PWD="$SOURCE_DIR/libs/IVT"
if [ "$IVT_OPTION" == "y" ]; then
	cd $IVT_PWD
	echo "Building in $(pwd)"
	./build.sh
	ret=$(echo $?)
  if [ "$ret" == "0" ]; then echo "ivt make successful"; else echo "ivt make terminated with error. Please see the /dev/null file"; exit_function; fi
  cd $SOURCE_DIR
fi
}

function enter_external_algorithm_fn
{
cd $SOURCE_DIR
if [ "$EXTERNAL_OPTION" == "y" ] ; then
    EXTERNAL_PWD="$SOURCE_DIR/external"
    cd $EXTERNAL_PWD
    cd algorithms/VSF-sceneflow-1.0
    echo "Building in $(pwd)"
    if [ "$BUILD_OPTION" == "clean" ]; then echo "cleaning external algorithms ...."; make clean; cd $SOURCE_DIR; return; fi
    make
    cd $SOURCE_DIR
fi
}

function enter_dummy_fn
{
    echo "Do Nothing"
}

SOURCE_DIR=$(pwd)
BOOST_PWD="$SOURCE_DIR/libs/boost"
FFMPEG_PWD="$SOURCE_DIR/libs/ffmpeg"
OPENCV_PWD="$SOURCE_DIR/libs/opencv"
PROJECT_PWD="$SOURCE_DIR/project/PerceptionSandbox"
#PROJECT_PWD="$SOURCE_DIR/project"
export PKG_CONFIG_PATH=$FFMPEG_PWD/../ffmpeg-install/lib/pkgconfig:$BOOST_PWD/../boost-install/lib/pkgconfig:$OPENCV_PWD/../opencv-install/lib/pkgconfig
#TODO : Compare Inode instead of string
enter_boost_fn
    if [ "$BUILD_OPTION" != "clean" ]; then
    PKG_CONFIG_PATH_INCLUDE_BOOST=$(pkg-config --cflags boost)
    echo $PKG_CONFIG_PATH_INCLUDE_BOOST
    ACTUAL_BOOST_INCLUDE_PATH=$BOOST_PWD/../boost-install/include
    echo $ACTUAL_BOOST_INCLUDE_PATH
    if [[ $PKG_CONFIG_PATH_INCLUDE_BOOST == "-I$ACTUAL_BOOST_INCLUDE_PATH"* ]]; then echo "Correct PKG_CONFIG_PATH_INCLUDE_BOOST $PKG_CONFIG_PATH_INCLUDE_BOOST"; else echo "Incorrect PKG_CONFIG_PATH_INCLUDE_BOOST $PKG_CONFIG_PATH_INCLUDE_BOOST"; exit_function; fi
    fi
enter_ffmpeg_fn
    if [ "$BUILD_OPTION" != "clean" ]; then
    PKG_CONFIG_PATH_INCLUDE_FFMPEG=$(pkg-config --cflags libavcodec)
    echo $PKG_CONFIG_PATH_INCLUDE_FFMPEG
    ACTUAL_FFMPEG_INCLUDE_PATH=$FFMPEG_PWD/../ffmpeg-install/include
    echo $ACTUAL_FFMPEG_INCLUDE_PATH
    if [[ $PKG_CONFIG_PATH_INCLUDE_FFMPEG == "-I$ACTUAL_FFMPEG_INCLUDE_PATH"* ]]; then echo "Correct PKG_CONFIG_PATH_INCLUDE_FFMPEG $PKG_CONFIG_PATH_INCLUDE_FFMPEG"; else echo "Incorrect PKG_CONFIG_PATH_INCLUDE_FFMPEG $PKG_CONFIG_PATH_INCLUDE_FFMPEG"; exit_function; fi
    fi
enter_opencv_fn
    if [ "$BUILD_OPTION" != "clean" ]; then
    PKG_CONFIG_PATH_INCLUDE_OPENCV=$(pkg-config --cflags opencv)
    echo $PKG_CONFIG_PATH_INCLUDE_OPENCV
    ACTUAL_OPENCV_INCLUDE_PATH=$SOURCE_DIR/libs/opencv-install/include
    echo $ACTUAL_OPENCV_INCLUDE_PATH
    if [[ $PKG_CONFIG_PATH_INCLUDE_OPENCV == "-I$ACTUAL_OPENCV_INCLUDE_PATH"* ]]; then echo "Correct PKG_CONFIG_PATH_INCLUDE_OPENCV $PKG_CONFIG_PATH_INCLUDE_OPENCV"; else echo "Incorrect PKG_CONFIG_PATH_INCLUDE_OPENCV $PKG_CONFIG_PATH_INCLUDE_OPENCV"; exit_function; fi
    fi
#enter_caffe_fn
#enter_libsvm_fn
#enter_ivt_fn
enter_project_fn
enter_project_run_fn
#enter_external_algorithm_fn
enter_dummy_fn

