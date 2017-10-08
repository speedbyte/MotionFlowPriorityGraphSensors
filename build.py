#!/usr/bin/env python
# _*_ coding: utf-8 _*_

import argparse
import subprocess
import os

import sys
import re


SOURCE_DIR = os.path.dirname(os.path.abspath(__file__))
SOURCE_DIR = SOURCE_DIR + '/'
def call_shell_command(command):
    subprocess.check_call(command.split(' '), stderr=subprocess.PIPE)
    #except subprocess.CalledProcessError as e:
    #    print e.returncode
    #    print e.output
    #    print e.message

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


def enter_dummy_fn(args):
    print "Do Nothing"


def parse_arguements(args):

    if args.VERBOSE_OPTION:
        verbose_string="ON"
    else:
        verbose_string="OFF"

    command = "git submodule status | gawk '{print $1}'"
    submodule_commit = subprocess.check_output(command, shell=True)
    submodule_commit = submodule_commit.split('\n')

    command = "git submodule status | gawk '{print $2}'"
    submodule_dir = subprocess.check_output(command, shell=True)
    submodule_dir = submodule_dir.split('\n')

    build_properties = len(submodule_dir)*[None]

    for count in range(len(submodule_dir)-1):
        submodule_commit[count] = re.search('\w{40}', submodule_commit[count]).group(0)

    submodule_metadata = list()
    for count in range(len(submodule_dir)-1):
        os.chdir(submodule_dir[count])
        command = "git show " + submodule_commit[count] + " --pretty=format:\"%ci%d\"|head -n1"
        submodule_tag = subprocess.check_output(command, shell=True)
        submodule_tag = submodule_tag.strip('\n')
        submodule_metadata.append((submodule_dir[count], submodule_tag ))
        if ( "libs/opencv" in submodule_dir[count]):
            build_properties[count] = args.OPENCV_OPTION
        if ( "libs/pcl" in submodule_dir[count]):
            build_properties[count] = args.PCL_OPTION
        if ( "libs/vtk" in submodule_dir[count]):
            build_properties[count] = args.VTK_OPTION
        os.chdir(SOURCE_DIR)

    print build_properties
    zipped = zip(submodule_metadata, build_properties)
    build_option = args.BUILD_OPTION
    make_power = subprocess.check_output("getconf _NPROCESSORS_ONLN", shell=True)
    make_power = make_power.strip('\n')
    for count in range(len(zipped)):
        if zipped[count][1] == True:
            os.chdir(SOURCE_DIR)
            metadata = zipped[count][0][1]
            module = SOURCE_DIR + zipped[count][0][0]
            module_install = module + "-install"
            print "starting building %s with version %s" % (module, metadata)
            if ( args.INSTALL_OPTION ):
                command = "sudo rsync -pavr " + module_install + "/ /usr/local/"
                call_shell_command(command)
                sys.exit(0)
            call_shell_command("mkdir -p " + module_install)
            os.chdir(module)
            print "Building in " + os.getcwd()
            if build_option == "clean":
                print "cleaning %s" %module;
                call_shell_command("rm -rf cmake-build-debug")
                call_shell_command("rm -rf " + module_install + "/*")
                sys.exit(0)

            elif build_option == "manual":
                raw_input("Press enter to continue")
            print "Configuring %s" %(module)
            if build_option == "manual":
                raw_input("Press enter to continue")
            call_shell_command("mkdir -p cmake-build-debug")
            os.chdir("cmake-build-debug")
            command = "cmake -Wno -dev -Wl,-rpath=/usr/local/lib -DENABLE_PRECOMPILED_HEADERS=OFF " \
                "-DCMAKE_BUILD_TYPE=DEBUG -DCMAKE_INSTALL_PREFIX=" + module_install + " " + \
                "-DCMAKE_VERBOSE_MAKEFILE=" + verbose_string + " " \
                "-DVTK_Group_StandAlone=ON -DVTK_Group_Rendering=ON -DVTK_Group_Qt=OFF -DVTK_Group_Views=OFF " \
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
parser.add_argument('-v', action='store_true', dest='VERBOSE_OPTION', help='build parameters')
parser.add_argument('-i', action='store_true', dest='INSTALL_OPTION', help='build parameters')

parser.set_defaults(func=parse_arguements)

if len(sys.argv) == 1:
    print (parser.format_help())

results = parser.parse_args()
results.func(results)