from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['car_control'],
    scripts=['scripts/control_callbacks',
             'scripts/control_classes',
             'scripts/listen_control',
             'scripts/motor_talker',
             'scripts/pid_talker',
             'scripts/utils'],
    package_dir={'': 'src'}
)

setup(**d)