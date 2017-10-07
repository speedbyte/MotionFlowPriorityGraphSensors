#!/usr/bin/env python
# _*_ coding: utf-8 _*_

import argparse
import subprocess
import os

import sys



SOURCE_DIR = os.path.dirname(os.path.abspath(__file__))
OPENCV_PWD = SOURCE_DIR + "/libs/opencv"
OPENCV_INSTALL_PWD = SOURCE_DIR + "/libs/opencv-install"

def call_shell_command(command):
    proc = subprocess.check_call(command.split(' '), stderr=subprocess.PIPE)
    #proc = subprocess.Popen(command.split(' '), stderr=subprocess.PIPE)
    #proc.wait()
    #out, err = proc.communicate()
    #if (proc.returncode != 0 or err != ''):
    #    print command + " failed"
    #    print err
    #    sys.exit(-1)
    #else:
    #    print command + " successful"
    #    return out


def enter_opencv_fn(args):
    os.chdir(SOURCE_DIR)
    print args.OPENCV_OPTION
    build_option = args.BUILD_OPTION
    print build_option
    os.chdir(OPENCV_PWD)
    print "Building in " + os.getcwd()
    if build_option == "clean":
        print "cleaning opencv ....";
        call_shell_command("rm -rf cmake-build-debug")
        call_shell_command("rm -rf OPENCV_INSTALL_PWD/*")
    elif build_option == "manual":
        raw_input("Press enter to continue")
    print "Configuring opencv"
    if build_option == "manual":
        raw_input("Press enter to continue")
    call_shell_command("mkdir -p cmake-build-debug")
    os.chdir("cmake-build-debug")
    command = "cmake -Wno -dev -Wl,-rpath=/usr/local/lib -DENABLE_PRECOMPILED_HEADERS=OFF " \
              "-DCMAKE_BUILD_TYPE=DEBUG -DCMAKE_INSTALL_PREFIX=SOURCE_DIR/libs/opencv-install " \
              "-DWITH_GTK=ON -DBUILD_JPEG=ON .."
    call_shell_command(command)
    if build_option == "manual":
        raw_input("Press enter to continue")
    command = "time make -j 8"
    call_shell_command(command)
    if build_option == "manual":
        raw_input("Press enter to continue")
    command = "time make install"
    call_shell_command(command)
    os.chdir(SOURCE_DIR)


def enter_pcl_fn(args):
    os.chdir('libs/pcl/cmake-build-debug')
    out = call_shell_command("pwd")
    if ("libs/pcl/cmake-build-debug" in out):
        out = call_shell_command("cmake -DCMAKE_INSTALL_PREFIX=../pcl-install ..")


def enter_install_fn(args):
    INSTALL_OPTION=args.dest
    if ( INSTALL_OPTION == "y" ):
        call_shell_command("gksudo bash copybuild.sh")


def enter_dummy_fn(args):
    print "Do Nothing"


def parse_arguements(args):
    if args.OPENCV_OPTION:
        enter_opencv_fn(args)

parser = argparse.ArgumentParser()

parser.add_argument('-o', '--opencv', action='store_true', dest='OPENCV_OPTION', help='builds opencv')
parser.add_argument('-p', '--pcl', action='store_true', dest='PCL_OPTION', help='builds pcl')
parser.add_argument('-t', dest="BUILD_OPTION", choices=('clean', 'manual'), help='build parameters')

parser.set_defaults(func=parse_arguements)

if len(sys.argv) == 1:
    print (parser.format_help())

#results = parser.parse_args()
#print results.BUILD_OPTIONS

results = parser.parse_args()
results.func(results)