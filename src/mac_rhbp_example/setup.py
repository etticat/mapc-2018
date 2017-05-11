#! /usr/bin/env python2
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['agent_common'],
    package_dir={'': 'src'},
    scripts=['']
)

setup(**setup_args)
