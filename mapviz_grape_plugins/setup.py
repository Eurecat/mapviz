## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['mapviz_grape_plugins'],
    package_dir={'': 'src'},
    requires=[ 'geometry_msgs', 'sensor_msgs', 'rospy', 'tf']
)

setup(**setup_args)