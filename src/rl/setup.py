from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ppo_control','ddpg_control'],
    package_dir={'': ''}
)
setup(**d)