# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from setuptools import setup, Extension

import fnmatch
import os
import platform
import sys


def get_libcarla_extensions():
    include_dirs = ['dependencies/include']
    library_dirs = ['dependencies/lib']
    libraries = []

    if os.name == "posix":
        if platform.dist()[0] == "Ubuntu":
            pwd = os.path.dirname(os.path.realpath(__file__))
            pylib = "libboost_python%d%d.a" % (sys.version_info.major,
                                               sys.version_info.minor)
            extra_link_args = [
                os.path.join(pwd, 'dependencies/lib/librpc.a'),
                os.path.join(pwd, 'dependencies/lib', pylib)]
            extra_compile_args = [
                '-fPIC', '-std=c++14', '-DBOOST_ERROR_CODE_HEADER_ONLY', '-Wno-missing-braces'
            ]
            # @todo Why would we need this?
            include_dirs += ['/usr/lib/gcc/x86_64-linux-gnu/7/include']
            library_dirs += ['/usr/lib/gcc/x86_64-linux-gnu/7']
            extra_link_args += ['/usr/lib/gcc/x86_64-linux-gnu/7/libstdc++.a']
    else:
        raise NotImplementedError

    def walk(folder, file_filter='*'):
        for root, _, filenames in os.walk(folder):
            for filename in fnmatch.filter(filenames, file_filter):
                yield os.path.join(root, filename)

    depends = [x for x in walk('source/libcarla')]
    depends += [x for x in walk('dependencies')]

    def make_extension(name, sources):
        return Extension(
            name,
            sources=sources,
            include_dirs=include_dirs,
            library_dirs=library_dirs,
            libraries=libraries,
            extra_compile_args=extra_compile_args,
            extra_link_args=extra_link_args,
            language='c++14',
            depends=depends)

    sources = ['source/libcarla/libcarla.cpp']
    sources += [x for x in walk('dependencies/include', '*.cpp')]

    print('compiling:\n  - %s' % '\n  - '.join(sources))

    return [make_extension('carla.libcarla', sources)]


setup(
    name='carla',
    version='0.9.0',
    package_dir={'': 'source'},
    packages=['carla'],
    ext_modules=get_libcarla_extensions(),
    license='MIT License',
    description='Python API for communicating with the CARLA server.',
    url='https://github.com/carla-simulator/carla',
    author='The CARLA team',
    author_email='carla.simulator@gmail.com',
    include_package_data=True)
