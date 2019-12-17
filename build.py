#/usr/bin/env python
# _*_ coding: utf-8 _*_




#CARLA
#sudo add-apt-repository ppa:ubuntu-toolchain-r/test
#sudo apt-get update
#sudo apt-get install build-essential clang-5.0 lld-5.0 g++-7 ninja-build python python-pip python-dev tzdata sed curl wget unzip autoconf libtool
#pip install --user setuptools nose2

#git clone --depth=1 -b release https://github.com/EpicGames/UnrealEngine.git ./UnrealEngine_4.2
#mono-xbuild
#./Setup.sh && ./GenerateProjectFiles.sh && make
# download Assets - Contents.tar.gz
# export UE4_ROOT=~/UnrealEngine_4.19

#LS_COLORS=$LS_COLORS:'di=1;35:' ; export LS_COLORS
#PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/local/git/PriorityGraphSensors/libs/ffmpeg-install/lib/pkgconfig:/local/git/PriorityGraphSensors/libs/boost-install/lib/pkgconfig:/local/git/PriorityGraphSensors/libs/opencv-install/lib/pkgconfig ; export PKG_CONFIG_PATH
#BOOST_ROOT=/local/git/PriorityGraphSensors/libs/boost-install; export BOOST_ROOT

#dependencies - yasm, freetype2

#dependencies unreal - monox-build
# ./setup.sh; ./GenerateProjectfiles.sh; make
# Engien/Binaries/Linux  -> .UE4Editor , here the shaders will be compiled on the first run

import argparse
import subprocess
import os

import sys
import re

SOURCE_DIR = os.path.dirname(os.path.abspath(__file__)) + '/'

def call_shell_command(command):
    ret = subprocess.check_output(command, shell=True, encoding='utf-8')
    if ret == 0:
        print("%s successful" % command)
        return ret
    else:
        print("%s failed" % command)
        return ret
    # except subprocess.CalledProcessError as e:
    #    print e.returncode
    #    print e.output
    #    print e.message

    # apt-cache search ^libflann-dev | cut -d ' ' -f1
    # proc = subprocess.Popen(command.split(' '), stderr=subprocess.PIPE)
    # proc.wait()
    # out, err = proc.communicate()
    # if (proc.returncode != 0 or err != ''):
    #    print command + " failed"
    #    print err
    #    sys.exit(-1)
    # else:
    #    print command + " successful"
    #    return out


def parse_arguements(args):
    if args.VERBOSE_OPTION:
        verbose_string = "ON"
    else:
        verbose_string = "OFF"

    command = "git submodule status | awk '{print $1}'"
    submodule_commit = subprocess.check_output(command, shell=True, encoding='utf-8')
    #submodule_commit = str(submodule_commit_bytes, 'utf-8')
    submodule_commit = submodule_commit.split('\n')

    command = "git submodule status | awk '{print $2}'"
    submodule_dir_bytes = subprocess.check_output(command, shell=True)
    submodule_dir = str(submodule_dir_bytes, 'utf-8')
    submodule_dir = submodule_dir.split('\n')

    build_properties = len(submodule_dir) * [None]

    for count in range(len(submodule_dir) - 1):
        submodule_commit[count] = re.search('\w{40}', submodule_commit[count]).group(0)

    submodule_metadata = list()
    for count in range(len(submodule_dir) - 1):
        os.chdir(submodule_dir[count])
        command = "git show " + submodule_commit[count] + " --pretty=format:\"%ci%d\"|head -n1"
        submodule_tag = subprocess.check_output(command, shell=True)
        submodule_tag = str(submodule_tag, 'utf-8')
        submodule_tag = submodule_tag.strip('\n')
        submodule_metadata.append((submodule_dir[count], submodule_tag))
        if "libs/opencv" == submodule_dir[count]:
            build_properties[count] = args.OPENCV_OPTION
        if "libs/pcl" in submodule_dir[count]:
            build_properties[count] = args.PCL_OPTION
        if "libs/vtk" in submodule_dir[count]:
            build_properties[count] = args.VTK_OPTION
        if "libs/boost" in submodule_dir[count]:
            build_properties[count] = args.BOOST_OPTION
        if "libs/ffmpeg" in submodule_dir[count]:
            build_properties[count] = args.FFMPEG_OPTION
        if "libs/vires-interface" in submodule_dir[count]:
            build_properties[count] = args.VIRES_OPTION
        if "libs/kitti_devkit_stereo_opticalflow_sceneflow" in submodule_dir[count]:
            build_properties[count] = args.KITTI_FLOW_OPTION
        os.chdir(SOURCE_DIR)

    print(build_properties)
    zipped = list(zip(submodule_metadata, build_properties))
    build_option = args.BUILD_OPTION
    make_power = subprocess.check_output("getconf _NPROCESSORS_ONLN", shell=True, encoding='utf-8')
    make_power = str(make_power).strip('\n')
    for count in range(len(zipped)):
        if zipped[count][1]:
            os.chdir(SOURCE_DIR)
            metadata = zipped[count][0][1]
            library = SOURCE_DIR + zipped[count][0][0]
            library_install = SOURCE_DIR + "libs-install/" + zipped[count][0][0] + "-install"

            if build_option == "clean":
                print("cleaning %s" % library);
                os.chdir(library)
                command = "rm -rf " + library + "/cmake-build-debug"
                print(command)
                call_shell_command(command)
                command = "rm -rf " + library_install + "/*"
                print(command)
                call_shell_command(command)
                if "boost" in library_install: 
                    command = "./b2 --clean-all -n && rm -rf bin.v2"
                elif "ffmpeg" in library_install:
                    command = "make clean"
                else:
                    command = "echo cmake-build-debug in " + library + " is already deleted"
#                print command
                call_shell_command(command)
                # SYSTEM INSTALL
                if args.INSTALL_OPTION:
                    command = "rm -rf /usr/local/include/" + zipped[count][0][0][zipped[count][0][0].find('/')+1:]
                    print(command)
                    #call_shell_command(command)
                sys.exit(0)

            os.chdir(library)
            if "kitti" in library_install:
                os.chdir("cpp") 
            print("starting building %s with version %s" % (os.getcwd(), metadata))
            if build_option == "manual":
                input("Press enter to continue")
            call_shell_command("mkdir -p " + library_install)
            command = "mkdir -p " + library_install + "/lib/pkgconfig"
            call_shell_command(command)

            # CONFIGURE
            print("Configuring %s" % library)
            if build_option == "manual":
                input("Press enter to continue")
            os.environ['PKG_CONFIG_PATH'] = "/usr/local/lib/pkgconfig"

            if "boost" in library_install:
                if os.path.isdir(library + "/tools/build") is False:
                    print("git fetch --all --recurse-submodules=yes. Do this in the boost diretory")
                    sys.exit(1)
                command = "./bootstrap.sh --prefix=" + library_install
            elif "ffmpeg" in library_install:
                command = "CXXFLAGS=\"-D__STDC_CONSTANT_MACROS -Wdeprecated-declarations -fPIC\" ./configure --prefix=\"" + library_install + "\" --bindir=\"./bin\" " \
                "--enable-shared " \
                "--enable-avresample " \
                "--enable-gpl " \
                "--disable-libass " \
                "--disable-libfdk-aac " \
                "--enable-libfreetype " \
                "--disable-libmp3lame "\
                "--disable-libopus "\
                "--disable-libtheora "\
                "--disable-libvorbis "\
                "--disable-libvpx "\
                "--enable-libx264 "\
                "--enable-nonfree "\
                "--disable-openssl "
            else:
                extra_cmake_option=""
                if "opencv" in library_install:
                    pkg_config_path_ffmpeg = subprocess.check_output("pkg-config --cflags libavcodec", shell=True, encoding='utf')
                    extra_cmake_option ="-DOPENCV_EXTRA_MODULES_PATH="+library+"/../opencv_contrib/modules "
                    #exit
                    print(extra_cmake_option)
                    if "/usr/local" in pkg_config_path_ffmpeg:
                        print("correct ffmpeg path while building opencv ", pkg_config_path_ffmpeg.strip('\n'))
                    else:
                        if build_option == "manual":
                            input("Press enter to continue")
                        print("Plese build ffmpeg before opencv. Aborting due to incompatibility between ffmpeg and opencv ", pkg_config_path_ffmpeg.strip('\n'))
                        sys.exit(-1)
                call_shell_command("mkdir -p cmake-build-debug")
                os.chdir("cmake-build-debug")
                command  =  "cmake -Wno -dev -Wl,-rpath=/usr/local/lib " +\
                            extra_cmake_option +\
                            "-DENABLE_PRECOMPILED_HEADERS=OFF " \
                            "-DCMAKE_BUILD_TYPE=DEBUG " \
                            "-DCMAKE_INSTALL_PREFIX=" + library_install + " " + \
                            "-DCMAKE_VERBOSE_MAKEFILE=" + verbose_string + " " \
                            " OPENCV_EXTRA_EXE_LINKER_FLAGS=-Wl,-rpath,/usr/local/lib" \
                            "-DCMAKE_USE_OPENSSL:BOOL=ON " \
                            "-DBUILD_TESTING=OFF " \
                            "-DVTK_BUILD_ALL_MODULES=ON " \
                            "-DVTK_DATA_EXCLUDE_FROM_ALL=ON " \
                            "-DVTK_Group_StandAlone=ON " \
                            "-DVTK_Group_Rendering=ON " \
                            "-DVTK_Group_Qt=OFF " \
                            "-DVTK_Group_Views=OFF " \
                            "-DWITH_GTK=ON " \
                            "-DWITH_PNG=ON " \
                            "-DWITH_CUDA=OFF " \
                            "-DWITH_MATLAB=OFF " \
                            "-DBUILD_JPEG=ON " \
                            "-DBUILD_2d=ON " \
                            "-DBUILD_CUDA=OFF " \
                            "-DBUILD_GPU=OFF " \
                            "-DBUILD_apps=OFF " \
                            "-DBUILD_common=ON " \
                            "-DBUILD_examples=ON " \
                            "-DBUILD_features=ON " \
                            "-DBUILD_filters=ON " \
                            "-DBUILD_geometry=ON " \
                            "-DBUILD_global_tests=OFF " \
                            "-DBUILD_io=ON " \
                            "-DBUILD_kdtree=ON " \
                            "-DBUILD_keypoints=ON " \
                            "-DBUILD_ml=OFF " \
                            "-DBUILD_octree=OFF " \
                            "-DBUILD_outofcore=OFF " \
                            "-DBUILD_people=OFF " \
                            "-DBUILD_recognition=OFF " \
                            "-DBUILD_registration=OFF " \
                            "-DBUILD_sample_consensus=ON " \
                            "-DBUILD_search=OFF " \
                            "-DBUILD_segmentation=OFF " \
                            "-DBUILD_simulation=OFF " \
                            "-DBUILD_stereo=OFF " \
                            "-DBUILD_surface=OFF " \
                            "-DBUILD_surface_on_nurbs=OFF " \
                            "-DBUILD_tools=ON " \
                            "-DBUILD_tracking=ON " \
                            "-DBUILD_visualization=ON " \
                            ".."
            print(command)
            if build_option == "manual":
                input("Press enter to continue")
            call_shell_command(command)
            if build_option == "manual":
                input("Press enter to continue")

            # MAKE
            if "boost" in library_install:
                command = "./b2 cxxflags=\"-Wno-unused-local-typedefs -Wstrict-aliasing\" headers install -j " + make_power
            else:
                command = "time make -j " + make_power
            print(command)
            if build_option == "manual":
                input("Press enter to continue")
            ret = call_shell_command(command)
            if ( ret != 0 ):
                print("ffmpeg build terminated with error")
                sys.exit(1) # rm -rf build; rm -f config.mak;; fi            
            if build_option == "manual":
                input("Press enter to continue")

            # MAKE INSTALL
            if "boost" in library_install:
                pass
            else:
                command = "time make install"
                if build_option == "manual":
                    input("Press enter to continue")
                print(command)
                call_shell_command(command)
                if build_option == "manual":
                    input("Press enter to continue")

            # POST INSTALL
            print("Post install")
            if "boost" in library_install:
                command = "python " + SOURCE_DIR + "utils/pkg-config-generator/main.py -n Boost -v 1.64.0 -p " + library_install + " -o " + library_install + "/lib/pkgconfig/boost.pc " + library_install + "/lib/"

            elif "ffmpeg" in library_install:
                command = "find " + library_install + "/lib/pkgconfig/ -name '*.pc' -exec sed -i 's!" + library_install + "!/usr/local!' {} \;"

            else:
                command = ""
                print("no post install command")

            print(command)
            if build_option == "manual":
                input("Press enter to continue")
            ret = call_shell_command(command)

            os.chdir(SOURCE_DIR)

            # SYSTEM INSTALL
            if args.INSTALL_OPTION:
                #kinit
                #aklogs
                command = "rsync -avr " + library_install + "/ ~/tmp/"
                print(command)
                if build_option == "manual":
                    input("Press enter to continue")
                call_shell_command(command)
                # copy back to /usr/local
                command = "sudo rsync -avr ~/tmp/ /usr/local/"
                print(command)
                if build_option == "manual":
                    input("Press enter to continue")
                call_shell_command(command)
                command = "rm -rf ~/tmp"
                print(command)
                if build_option == "manual":
                    input("Press enter to continue")
                call_shell_command(command)



parser = argparse.ArgumentParser()
#subparsers = parser.add_subparsers(title='subcommands', description='')
#External Modules
parser.add_argument('--opencv', action='store_true', dest='OPENCV_OPTION', help='builds opencv')
parser.add_argument('--pcl', action='store_true', dest='PCL_OPTION', help='builds pcl')
parser.add_argument('--vtk', action='store_true', dest='VTK_OPTION', help='builds vtk')
parser.add_argument('--boost', action='store_true', dest='BOOST_OPTION', help='builds boost')
parser.add_argument('--ffmpeg', action='store_true', dest='FFMPEG_OPTION', help='builds ffmpeg')
parser.add_argument('--vires', action='store_true', dest='VIRES_OPTION', help='builds vires')
parser.add_argument('--kitti_flow', action='store_true', dest='KITTI_FLOW_OPTION', help='builds kitti flow')
#Generic options
parser.add_argument('-t', dest="BUILD_OPTION", choices=('clean', 'manual'), help='build parameters')
parser.add_argument('-v', action='store_true', dest='VERBOSE_OPTION', help='build parameters')
parser.add_argument('-i', action='store_true', dest='INSTALL_OPTION', help='build parameters')
#Tests
parser.add_argument('--test', action='store_true', help='test script')

parser.set_defaults(func=parse_arguements)

if len(sys.argv) == 1:
    print((parser.format_help()))

results = parser.parse_args()
results.func(results)


#parser = argparse.ArgumentParser()
#parser.add_argument('host', help = "Host to schedule. Local fqdn used if not specified.", nargs = '?'  default=alias)
#parser.add_argument('duration', type = int, help = 'Duration of downtime (minutes), 15 if not specified', default=15)
#parser.add_argument('-d', '--debug', action='store_true', help = 'Print debug info')

#g = p.add_mutually_exclusive_group()
#g.add_argument('-l', '--flexible', help = "Use f_L_exible downtime (used by default)", action='store_true')
#g.add_argument('-f', '--fixed', help = 'Use _F_ixed downtime', action="store_false")
#mode2 = p.add_argument_group('show')
#mode2.add_argument('-s', '--show', help = 'show downtimes for host', action="store_true")
#args = p.parse_args()


