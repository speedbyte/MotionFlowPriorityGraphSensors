#!/usr/bin/env python
# _*_ coding: utf-8 _*_

import argparse
import subprocess
import os

import sys

def pcl_prepare(args):
    print "i am in pcl"
    os.chdir('libs/pcl/cmake-build-debug')
    command = "ls -l"
    subprocess.Popen("pwd", stdout=subprocess.PIPE)
    subprocess.check_call(command.split(' '))
    if ( "libs/pcl/cmake-build-debug" in sys.stdout):
        print "success, we can proceed"
    else:
        print "failure"
    command = "cmake _D CMAKE_INSTALL_PREFIX .."
    subprocess.check_call(command.split(' '))


parser = argparse.ArgumentParser()
subparsers = parser.add_subparsers(title='subcommands', description='')

#
parser_resync = subparsers.add_parser('pcl', help='builds or cleans pcl')
parser_resync.set_defaults(func=pcl_prepare)
parser_resync.add_argument('-c', '--clean', help='cleans pcl', type=str)
parser_resync.add_argument('-b', '--build', help='builds pcl', type=str)

if len(sys.argv) == 1:
    print parser.format_help()

args = parser.parse_args()
args.func(args)


