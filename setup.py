#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['navigation', 'utils', 'agent', 'restaurant'],
   package_dir={'': 'src'}
)

g = generate_distutils_setup(
   packages=['msg'],
   package_dir={'': ""}
)

setup(**d)
setup(**g)
