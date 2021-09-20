#!/bin/sh

#install all of the relevant ROS packages needed to run the simulator
rosdep update; rosdep install -i --from-path src -y

pip install pycryptodomex
pip install rospkg
pip install do-mpc
pip install gnupg
pip install scipy
pip install defusedxml
pip install dataclasses