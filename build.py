#!/usr/bin/env python
# _*_ coding: utf-8 _*_

import argparse
import subprocess
import os

import sys
import re

SOURCE_DIR = os.path.dirname(os.path.abspath(__file__)) + '/'


def call_shell_command(command):
    ret = subprocess.check_call(command, shell=True)
    if ret == 0:
        print "%s successful" % command
    else:
        print "%s failed" % command
        sys.exit(1)

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
    submodule_commit = subprocess.check_output(command, shell=True)
    submodule_commit = submodule_commit.split('\n')

    command = "git submodule status | awk '{print $2}'"
    submodule_dir = subprocess.check_output(command, shell=True)
    submodule_dir = submodule_dir.split('\n')

    build_properties = len(submodule_dir) * [None]

    for count in range(len(submodule_dir) - 1):
        submodule_commit[count] = re.search('\w{40}', submodule_commit[count]).group(0)

    submodule_metadata = list()
    for count in range(len(submodule_dir) - 1):
        os.chdir(submodule_dir[count])
        command = "git show " + submodule_commit[count] + " --pretty=format:\"%ci%d\"|head -n1"
        submodule_tag = subprocess.check_output(command, shell=True)
        submodule_tag = submodule_tag.strip('\n')
        submodule_metadata.append((submodule_dir[count], submodule_tag))
        if "libs/opencv" in submodule_dir[count]:
            build_properties[count] = args.OPENCV_OPTION
        if "libs/pcl" in submodule_dir[count]:
            build_properties[count] = args.PCL_OPTION
        if "libs/vtk" in submodule_dir[count]:
            build_properties[count] = args.VTK_OPTION
        os.chdir(SOURCE_DIR)

    print build_properties
    zipped = zip(submodule_metadata, build_properties)
    build_option = args.BUILD_OPTION
    make_power = subprocess.check_output("getconf _NPROCESSORS_ONLN", shell=True)
    make_power = make_power.strip('\n')
    for count in range(len(zipped)):
        if zipped[count][1]:
            os.chdir(SOURCE_DIR)
            metadata = zipped[count][0][1]
            library = SOURCE_DIR + zipped[count][0][0]
            library_install = library + "-install"
            print "starting building %s with version %s" % (library, metadata)
            if build_option == "clean":
                print "cleaning %s" % library;
                command = "rm -rf cmake-build-debug"
                print command
                call_shell_command(command)
                command = "rm -rf " + library_install + "/*"
                print command
                call_shell_command(command)
                sys.exit(0)
            if args.INSTALL_OPTION:
                command = "sudo rsync -pavr " + library_install + "/ /usr/local/"
                call_shell_command(command)
                continue
            call_shell_command("mkdir -p " + library_install)
            os.chdir(library)
            print "Building in " + os.getcwd()
            if build_option == "manual":
                raw_input("Press enter to continue")
            print "Configuring %s" % library
            if build_option == "manual":
                raw_input("Press enter to continue")
            call_shell_command("mkdir -p cmake-build-debug")
            os.chdir("cmake-build-debug")

            command  =  "/usr/bin/cmake -Wno -dev -Wl,-rpath=/usr/local/lib " \
                        \
                        "-DENABLE_PRECOMPILED_HEADERS=OFF " \
                        "-DCMAKE_BUILD_TYPE=DEBUG " \
                        "-DCMAKE_INSTALL_PREFIX=" + library_install + " " + \
                        "-DCMAKE_VERBOSE_MAKEFILE=" + verbose_string + " " \
                        "-DCMAKE_USE_OPENSSL:BOOL=ON " \
                        "-DBUILD_TESTING=OFF " \
                        "-DVTK_BUILD_ALL_MODULES=ON " \
                        "-DVTK_DATA_EXCLUDE_FROM_ALL=ON " \
                        "-DVTK_Group_StandAlone=ON " \
                        "-DVTK_Group_Rendering=ON " \
                        "-DVTK_Group_Qt=OFF " \
                        "-DVTK_Group_Views=OFF " \
                        "-DWITH_GTK=ON " \
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

            print command
            if build_option == "manual":
                raw_input("Press enter to continue")
            call_shell_command(command)
            if build_option == "manual":
                raw_input("Press enter to continue")
            command = "time make -j " + make_power
            print command
            if build_option == "manual":
                raw_input("Press enter to continue")
            call_shell_command(command)
            if build_option == "manual":
                raw_input("Press enter to continue")
            command = "time make install"
            if build_option == "manual":
                raw_input("Press enter to continue")
            call_shell_command(command)
            if build_option == "manual":
                raw_input("Press enter to continue")
            os.chdir(SOURCE_DIR)


parser = argparse.ArgumentParser()

parser.add_argument('--test', action='store_true', help='test script')
parser.add_argument('--opencv', action='store_true', dest='OPENCV_OPTION', help='builds opencv')
parser.add_argument('--pcl', action='store_true', dest='PCL_OPTION', help='builds pcl')
parser.add_argument('--vtk', action='store_true', dest='VTK_OPTION', help='builds vtk')
parser.add_argument('-t', dest="BUILD_OPTION", choices=('clean', 'manual'), help='build parameters')
parser.add_argument('-v', action='store_true', dest='VERBOSE_OPTION', help='build parameters')
parser.add_argument('-i', action='store_true', dest='INSTALL_OPTION', help='build parameters')

parser.set_defaults(func=parse_arguements)

if len(sys.argv) == 1:
    print (parser.format_help())

results = parser.parse_args()
results.func(results)
