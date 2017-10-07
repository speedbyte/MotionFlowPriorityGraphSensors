#subparsers = parser.add_subparsers(title='subcommands', description='')
#parser_opencv = subparsers.add_parser('opencv', help='builds or cleans opencv')
#parser_opencv.set_defaults(func=enter_opencv_fn)

group_opencv = parser.add_mutually_exclusive_group()

'''
    print "Check compatibility"
    if ( PKG_CONFIG_PATH_INCLUDE_FFMPEG == "-I$ACTUAL_FFMPEG_INCLUDE_PATH"):
        print "Correct PKG_CONFIG_PATH_INCLUDE_FFMPEG $PKG_CONFIG_PATH_INCLUDE_FFMPEG"
    else:
        print "Incorrect PKG_CONFIG_PATH_INCLUDE_FFMPEG $PKG_CONFIG_PATH_INCLUDE_FFMPEG"
        sys.exit()

    if (BUILD_OPTION == "manual"):
        raw_input("Press enter to continue")
'''


BOOST_PWD = SOURCE_DIR + "/libs/boost"
FFMPEG_PWD = SOURCE_DIR + "/libs/ffmpeg"
PROJECT_PWD = SOURCE_DIR + "/project/VirtualTestDriveFramework"
# TODO : Compare Inode instead of directory string
# export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig
if ["$BUILD_OPTION" != "clean"]; then
PKG_CONFIG_PATH_INCLUDE_FFMPEG =$(pkg - config - -cflags
libavcodec)
echo $PKG_CONFIG_PATH_INCLUDE_FFMPEG
ACTUAL_FFMPEG_INCLUDE_PATH = / usr / local / include
echo $ACTUAL_FFMPEG_INCLUDE_PATH
if [
    [ $PKG_CONFIG_PATH_INCLUDE_FFMPEG == "-I$ACTUAL_FFMPEG_INCLUDE_PATH" *]]; then print "Correct PKG_CONFIG_PATH_INCLUDE_FFMPEG $PKG_CONFIG_PATH_INCLUDE_FFMPEG"; else print "Incorrect PKG_CONFIG_PATH_INCLUDE_FFMPEG $PKG_CONFIG_PATH_INCLUDE_FFMPEG"; exit_function; fi
fi
# enter_caffe_fn
# enter_libsvm_fn
# enter_ivt_fn
# enter_external_algorithm_fn


def enter_ffmpeg_fn(args):
    if ["$FFMPEG_OPTION" == "y"] ; then
    tput
    setf
    3
    os.chdir(FFMPEG_PWD
    print "Building in " + os.getcwd()
    if (BUILD_OPTION == "manual"):
        then
    read - p
    "Press enter to continue";
    fi
    if ( BUILD_OPTION == "clean"):
        then
    print "cleaning ffmpeg ....";
    make
    distclean;
    rm - rf $FFMPEG_PWD /../ ffmpeg - install / *;
    cd.. /../; return;
    fi
    print "Configuring ffmpeg"
    os.chdir(FFMPEG_PWD
    CXXFLAGS = "-D__STDC_CONSTANT_MACROS -Wdeprecated-declarations -fPIC". / configure - -prefix = "$FFMPEG_PWD/../ffmpeg-install" - -bindir = "./bin" - -enable - shared
    if (BUILD_OPTION == "manual"):
        then
    read - p
    "Press enter to continue";
    fi
    time
    make - j $(getconf
    _NPROCESSORS_ONLN) 2 > & 1
    ret =$(echo $?)
    print "make in $SOURCE_DIR) returned $ret"
    if ["$ret" == "0"];
    then
    print "ffmpeg make successful"; else print "ffmpeg build terminated with error";
    rm - rf
    build;
    rm - f
    config.mak;
    exit_function;
    fi
    if (BUILD_OPTION == "manual"):
        then
    read - p
    "Press enter to continue";
    fi
    time
    make
    install >> / dev / null  # 2>&1
    ret =$(echo $?)
    print "make-install in $SOURCE_DIR) returned $ret"
    if ["$ret" == "0"];
    then
    print "ffmpeg make-install successful"; else print "ffmpeg make-install terminated with error. Please see the /dev/null file ";
    exit_function;
    fi
    find $FFMPEG_PWD /../ ffmpeg - install / lib / pkgconfig - name
    '*.pc' -
    exec sed - i
    's!/local/git/PriorityGraphSensors/libs/ffmpeg/../ffmpeg-install$!/usr/local!'
    {} \;
    os.chdir(SOURCE_DIR)
    fi



def enter_boost_fn(args):
    os.chdir(BOOST_PWD)
    print "Building in " + os.getcwd()
    if (BUILD_OPTION == "manual"):
        sys.stdin()
    "Press enter to continue";
    fi
    if ( BUILD_OPTION == "clean"):
        then
    print "cleaning boost ....";./ b2 - -clean;
    rm - rf
    bin.v2;
    rm - rf $BOOST_PWD /../ boost - install / *;
    cd.. /../; return;
    fi
    if [ ! -d $BOOST_PWD / tools / build]; then print "download build.git by git submodule update --remote"; exit; fi
    time. / bootstrap.sh - -prefix =$BOOST_PWD /../ boost - install
    if (BUILD_OPTION == "manual"): then read -p "Press enter to continue"; fi
    time. / b2
    cxxflags = "-Wno-unused-local-typedefs -Wstrict-aliasing" - j $(getconf
    _NPROCESSORS_ONLN) headers
    install
    ret =$(echo $?)
    print "make in $SOURCE_DIR) returned $ret"
    if [
                "$ret" == "0"]; then print "boost make successful"; else print "boost build terminated with error"; exit_function; fi
    # Copies the
    # cp -Rv boost $BOOST_PWD/../boost-install/include/
    mkdir - p $BOOST_PWD /../ boost - install / lib / pkgconfig
    python $SOURCE_DIR) / utils / pkg - config - generator / main.py - n
    Boost - v
    1.64
    .0 - p $BOOST_PWD /../ boost - install - o $BOOST_PWD /../ boost - install / lib / pkgconfig / boost.pc $BOOST_PWD /../ boost - install / lib /
                                                                                                                                    # python main.py -n Boost -v 1.63.0 -p $BOOST_PWD/stage -o ./stage/lib/pkgconfig/boost.pc ./stage/lib/
                                                                                                                                    os.chdir(
    SOURCE_DIR)
    fi

