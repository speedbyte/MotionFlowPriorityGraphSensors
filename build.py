#!/usr/bin/env python
# _*_ coding: utf-8 _*_

import argparse
import subprocess
import os

import sys

def call_shell_command(command):
    proc = subprocess.Popen(command.split(' '), stderr=subprocess.PIPE, stdout=subprocess.PIPE)
    proc.wait()
    out, err = proc.communicate()
    if ( proc.returncode != 0  or err != '') :
        print command + " failed"
        print err
        sys.exit(-1)
    else:
        print command + " successful"
        return out


def pcl_prepare(args):

    os.chdir('libs/pcl/cmake-build-debug')
    out = call_shell_command("pwd")
    if ( "libs/pcl/cmake-build-debug" in out):
        out = call_shell_command("cmake -DCMAKE_INSTALL_PREFIX=../pcl-install ..")


parser = argparse.ArgumentParser()
subparsers = parser.add_subparsers(title='subcommands', description='')

parser_resync = subparsers.add_parser('pcl', help='builds or cleans pcl')
parser_resync.set_defaults(func=pcl_prepare)
parser_resync.add_argument('-c', '--clean', help='cleans pcl', type=str)
parser_resync.add_argument('-b', '--build', help='builds pcl', type=str)

if len(sys.argv) == 1:
    print (parser.format_help())

args = parser.parse_args()
args.func(args)


