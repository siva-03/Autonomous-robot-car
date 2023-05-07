from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['scripts'],
    scripts=['control_callbacks',
             'control_classes',
             'listen_control',
             'motor_talker',
             'pid_talker',
             'utils'],
    package_dir={'': 'src'}
)

setup(**d)