#!/bin/sh

#install all of the relevant ROS packages needed to run the simulator
rodep update; rosdep install -i --from-path src -y

pip install pycryptodomex
