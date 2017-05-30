#!/bin/bash

while getopts ":t:bfolscah" opt; do
  case $opt in
    t)
      echo "-t was triggered, Parameter: $OPTARG" >&2
      BUILD_OPTION=$OPTARG
      ;;
    b)
      echo "-b was triggered, Parameter: $OPTARG" >&2
      BOOTSTRAP_OPTION="y"
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
    s)
      echo "-s was triggered, Parameter: $OPTARG" >&2
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
    a)
      echo "-a was triggered, Parameter: $OPTARG" >&2
      OPENCV_OPTION="y"
      FFMPEG_OPTION="y"
      LIBSVM_OPTION="y"
      BOOTSTRAP_OPTION="y"
      PROJECT_OPTION="y"
      CAFFE_OPTION="y"
      IVT_OPTION="y"
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
    h)
      echo "Options are \
      -t \
      -b \
      -f \
      -o \
      -l \
      -p \
      -a \
      -h \
      -c \
      -i \
       "
      echo "Option -t means type and has parameters either manual or clean"
      echo "Option -b is meant for boost"
      echo "Option -f is meant for ffmpeg"
      echo "Option -o is meant for opencv"
      echo "Option -l is meant for libsvm"
      echo "Option -s is meant for image streaming server"
      echo "Option -c is meant for caffe"
      echo "Option -i is meant for ivt"
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
    BOOTSTRAP_OPTION="y"
    PROJECT_OPTION="y"
    CAFEE_OPTION="y"
    IVT_OPTION="y"
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
BOOTSTRAP_PWD="$(pwd)/libs/boost"
if [ "$BOOTSTRAP_OPTION" == "y" ] ; then
    cd $BOOTSTRAP_PWD
    if [ ! -d stage ] ; then
        if [ ! -f b2 ]; then
        ./bootstrap.sh
        fi
	./configure
        ./b2 cxxflags="-Wno-unused-local-typedefs -Wstrict-aliasing"
    fi
    ln -s ./boost ./stage/include
    mkdir ./stage/lib/pkgconfig
    python main.py -n Boost -v 1.63.0 -p $BOOTSTRAP_PWD/stage -o ./stage/lib/pkgconfig/boost.pc ./stage/lib/
    cd ../../
fi
}

function enter_ffmpeg_fn
{
FFMPEG_PWD="$(pwd)/libs/ffmpeg"
if [ "$FFMPEG_OPTION" == "y" ] ; then
    tput setf 3
    cd $FFMPEG_PWD
    echo "Building in $(pwd)"
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue"; fi
    if [ "$BUILD_OPTION" == "clean" ]; then echo "cleaning ffmpeg ...."; make clean; cd ../../; return; fi
    echo "Configuring ffmpeg"
    cd $FFMPEG_PWD
    ./configure --prefix=../ffmpeg-install
    #./conf.sh
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue"; fi
    time make > /dev/null #2>&1
    ret=$(echo $?)
    echo "make in $(pwd) returned $ret"
    if [ "$ret" == "0" ]; then echo "ffmpeg make successful"; else echo "ffmpeg build terminated with error"; rm -rf build; rm -f config.mak; exit_function; fi
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue"; fi
    time make install >> /dev/null #2>&1
    ret=$(echo $?)
    echo "make-install in $(pwd) returned $ret"
    if [ "$ret" == "0" ]; then echo "ffmpeg make-install successful"; else echo "ffmpeg make-install terminated with error. Please see the /dev/null file "; exit_function; fi
    cd ../../
fi
}

function enter_opencv_fn
{
OPENCV_PWD="$(pwd)/libs/opencv"
if [ "$OPENCV_OPTION" == "y" ]; then
    tput setf 2 
    cd $OPENCV_PWD
    echo "Building in $(pwd)"
    if [ "$BUILD_OPTION" == "clean" ]; then echo "cleaning opencv ...."; rm -rf Release; cd ../../; return; fi
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue";  fi
    echo "Check compatibility"
    FFMPEG_PWD="$(pwd)/libs/ffmpeg"
    export PKG_CONFIG_PATH=$FFMPEG_PWD/build/lib/pkgconfig:$BOOTSTRAP_PWD/stage/lib/pkgconfig
    PKG_CONFIG_PATH_FFMPEG=$(pkg-config --cflags libavcodec)
    echo $PKG_CONFIG_PATH_FFMPEG
    ACTUAL_FFMPEG_PATH=$FFMPEG_PWD/../ffmpeg-install/include
    echo $ACTUAL_FFMPEG_PATH
    if [[ $PKG_CONFIG_PATH_FFMPEG == "-I$ACTUAL_FFMPEG_PATH"* ]]; then echo "Correct PKG_CONFIG_PATH_FFMPEG $PKG_CONFIG_PATH_FFMPEG"; else echo "Incorrect PKG_CONFIG_PATH_FFMPEG $PKG_CONFIG_PATH_FFMPEG"; exit_function; fi
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue";  fi
    echo "Configuring opencv"
    #export PKG_CONFIG_LIBDIR=$PKG_CONFIG_LIBDIR:$(pwd)/../ffmpeg/build/lib/
    #export LD_LIBRARY_PATH=$(pwd)/../ffmpeg/build/lib/
    echo $(printenv | grep PKG_CONFIG_PATH)
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue";  fi
    echo "In order to use the opencv 3rd party versions of libraries instead of system ones on UNIX systems you
    should use BUILD_<library_name> CMake flags (for example, -D BUILD_PNG for the libpng library)."
    cmake -Wno-dev -D ENABLE_PRECOMPILED_HEADERS=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=../opencv-install -D WITH_GTK=ON ..
    cd release
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue";  fi
    time make > /dev/null #2>&1
    ret=$(echo $?)
    if [ "$ret" == "0" ]; then echo "opencv make successful"; else echo "opencv make terminated with error. Please see the /dev/null file"; exit_function; fi
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue"; fi
    time make install >> /dev/null #2>&1
    ret=$(echo $?)
    echo "make-install in $(pwd) returned $ret"
    if [ "$ret" == "0" ]; then echo "opencv make-install successful"; else echo "opencv make-install terminated with error. Please see the /dev/null file "; exit_function; fi
    cd ../../../
fi
}

function enter_libsvm_fn
{
LIBSVM_PWD="$(pwd)/libs/libsvm"
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
    echo "make in $(pwd) returned $ret"
    if [ "$ret" == "0" ]; then echo "libsvm make successful"; else echo "libsvm build terminated with error"; make clean; exit_function; fi
    ln -s libsvm.so.2 libsvm.so # making -lsvm possible
    cd ../../
fi
}

function enter_image_streaming_server_fn
{
PROJECT_PWD="$(pwd)"
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
    cd .
fi
}

function enter_caffe_fn
{
PROJECT_LIBS="$(pwd)/libs"
CAFFE_PWD="$(pwd)/libs/caffe"
if [ "$CAFFE_OPTION" == "y" ]; then
	cd $CAFFE_PWD
	echo "Building in $(pwd)"
	if [ "$BUILD_OPTION" == "clean" ]; then echo "Cleaning caffe"; make clean; cd .; return; fi
	#if [ ! -d build ] ; then
	#	mkdir build
	#fi
	#cd build
	#export CMAKE_LIBRARY_PATH=$BOOTSTRAP_PWD/stage/lib/:$CMAKE_LIBRARY_PATH
	#export CMAKE_INCLUDE_PATH=$BOOTSTRAP_PWD:$CMAKE_INCLUDE_PATH
	#cmake ..
  
	make all -n PROJECT_LIBS=$PROJECT_LIBS
	ret=$(echo $?)
  if [ "$ret" == "0" ]; then echo "caffe make successful"; else echo "caffe make terminated with error. Please see the /dev/null file"; exit_function; fi
  cd ../..
fi
}

function enter_ivt_fn
{
IVT_PWD="$(pwd)/libs/IVT"
if [ "$IVT_OPTION" == "y" ]; then
	cd $IVT_PWD
	echo "Building in $(pwd)"
	./build.sh
	ret=$(echo $?)
  if [ "$ret" == "0" ]; then echo "ivt make successful"; else echo "ivt make terminated with error. Please see the /dev/null file"; exit_function; fi
  cd ../..
fi
}

function enter_image_streaming_client_fn
{
IMAGE_PWD="$(pwd)/ImageStreamingClient"
if [ "$IMAGE_OPTION" == "y" ]; then
    tput setf 2 
    cd $IMAGE_PWD
    echo "Building in $(pwd)"
    if [ "$BUILD_OPTION" == "clean" ]; then make clean; cd ../; return; fi
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue";  fi
    ${QMAKE} ImageStreamingClient.pro #-recursive -spec linux-g++-64 CONFIG+=debug
    ret=$(echo $?)
    if [ "$ret" == "0" ]; then echo "image streaming qmake conf successful"; else echo "image streaming qmake conf terminated with error. Please see the /dev/null file"; exit_function; fi
    if [ "$BUILD_OPTION" == "manual" ]; then read -p "Press enter to continue";  fi
    time make #> /dev/null #2>&1
    ret=$(echo $?)
    if [ "$ret" == "0" ]; then echo "image streaming make successful"; else echo "image streaming make terminated with error. Please see the /dev/null file"; exit_function; fi
    cd ../
fi
}


enter_boost_fn
enter_ffmpeg_fn
enter_opencv_fn
enter_caffe_fn
enter_libsvm_fn
enter_ivt_fn
enter_image_streaming_server_fn
enter_image_streaming_client_fn
