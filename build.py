#!/usr/bin/env python
# _*_ coding: utf-8 _*_

import argparse
import subprocess
import os

import sys



SOURCE_DIR = os.path.dirname(os.path.abspath(__file__))

def call_shell_command(command):
    proc = subprocess.check_call(command.split(' '), stderr=subprocess.PIPE)
    #apt-cache search ^libflann-dev | cut -d ' ' -f1
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
    pass

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
    list_of_options = list()
    opencv_properties = ("opencv", 3.2, "2016-12-23")
    pcl_properties = ("pcl", "pcl-1.8.0", "2016-06-14")
    vtk_properties = ("vtk", "v6.0.0", "2013-06-12")
    list_of_options.append(opencv_properties)
    list_of_options.append(pcl_properties)
    list_of_options.append(vtk_properties)

    build_properties = (args.OPENCV_OPTION, args.PCL_OPTION, args.VTK_OPTION)

    zipped = zip(list_of_options, build_properties)
    build_option = args.BUILD_OPTION
    make_power = subprocess.check_output("getconf _NPROCESSORS_ONLN", shell=True)
    make_power = make_power.strip('\n')
    for count in range(len(zipped)):
        if zipped[count][1]:
            module = SOURCE_DIR + "/libs/" + zipped[count][0][0]
            module_install = module + "-install"
            call_shell_command("mkdir -p "+ module_install)
            print "starting building %s" % module
            os.chdir(SOURCE_DIR)
            os.chdir(module)
            print "Building in " + os.getcwd()
            if build_option == "clean":
                print "cleaning opencv ....";
                call_shell_command("rm -rf cmake-build-debug")
                call_shell_command("rm -rf libs/" + module_install + "/*")
            elif build_option == "manual":
                raw_input("Press enter to continue")
            print "Configuring %s" %(module)
            if build_option == "manual":
                raw_input("Press enter to continue")
            call_shell_command("mkdir -p cmake-build-debug")
            os.chdir("cmake-build-debug")
            command = "cmake -Wno -dev -Wl,-rpath=/usr/local/lib -DENABLE_PRECOMPILED_HEADERS=OFF " \
                      "-DCMAKE_BUILD_TYPE=DEBUG -DCMAKE_INSTALL_PREFIX=" + module_install + " " + \
                      "-DWITH_GTK=ON -DBUILD_JPEG=ON .."
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

parser.set_defaults(func=parse_arguements)

if len(sys.argv) == 1:
    print (parser.format_help())

results = parser.parse_args()
results.func(results)