#!/usr/bin/env python
# _*_ coding: utf-8 _*_

import argparse

import sys

def opencv():
    pass


parser = argparse.ArgumentParser()
subparsers = parser.add_subparsers(title='subcommands', description='')

#
parser_resync = subparsers.add_parser('opencv', help='builds or cleans opencv')
parser_resync.set_defaults(func=opencv)
parser_resync.add_argument('-c', '--clean', help='cleans opencv', type=str)
parser_resync.add_argument('-b', '--build', help='builds opencv', type=str)

if len(sys.argv) == 1:
    print parser.format_help()

args = parser.parse_args()
args.func(args)



