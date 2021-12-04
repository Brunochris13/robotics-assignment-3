#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['navigation'],
   package_dir={'': 'src'}
)

f = generate_distutils_setup(
   packages=['utils'],
   package_dir={'': 'src'}
)

g = generate_distutils_setup(
   packages=['msg'],
   package_dir={'': ""}
)

setup(**d)
setup(**f)
setup(**g)
