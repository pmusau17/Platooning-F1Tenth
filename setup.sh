#!/bin/sh

#install all of the relevant ROS packages needed to run the simulator
sudo apt-get update
sudo apt-get install -y ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-control ros-kinetic-ackermann-msgs ros-kinetic-joy
sudo apt-get install -y ros-kinetic-teb-local-planner ros-kinetic-move-base ros-kinetic-navigation ros-kinetic-hector-slam ros-kinetic-driver-common ros-kinetic-actionlib 
sudo apt-get install -y ros-kinetic-pcl-conversions ros-kinetic-pcl-msgs ros-kinetic-pcl-ros libeigen3-dev libproj-dev libmove-base-msgs-dev
sudo ln -s /usr/include/eigen3/Eigen /usr/local/include/Eigen

pip install rospkg
pip install defusedxml
pip install PySide2
pip uninstall em
pip install empy 
pip install imutils
pip install -r requirements-gpu.txt
