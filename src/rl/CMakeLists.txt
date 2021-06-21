cmake_minimum_required(VERSION 2.8.3)
project(rl)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  geometry_msgs
  tf
)

# install python modules
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs geometry_msgs gazebo_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


### Install python nodes
install(PROGRAMS 
  nodes/ars_controller.py
  nodes/ddpg_controller.py
  nodes/ddpg.py
nodes/ppo.py
nodes/sac_controller.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch and other files 
install(DIRECTORY
  config
  models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)