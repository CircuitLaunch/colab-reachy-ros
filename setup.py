from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['motion_control', 'mask_detection', 'state_machine'],
    package_dir={'': 'src'},
)

setup(**d)
