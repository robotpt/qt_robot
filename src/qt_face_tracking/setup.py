#!/usr/bin/env python3.6

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_dirs = {}

d = generate_distutils_setup(
    packages=package_dirs.keys(),
    package_dir=package_dirs,
)
setup(**d)
