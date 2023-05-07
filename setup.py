#!/usr/bin/python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    scripts=['src/car_control/control_callbacks.py',
             'src/car_control/control_classes.py',
             'src/car_control/utils.py'],
    packages=['car_control'],
    package_dir={'': 'src'}
)

setup(**setup_args)
