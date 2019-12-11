#!/bin/sh

#install all of the relevant ROS packages needed to run the simulator

sudo apt-get install -y ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-control ros-kinetic-ackermann-msgs ros-kinetic-joy
sudo apt-get install -y ros-kinetic-teb-local-planner ros-kinetic-move-base ros-kinetic-navigation ros-kinetic-hector-slam ros-kinetic-driver-common

pip install rospkg
pip uninstall em
pip install empy 
pip install -r requirements.txt