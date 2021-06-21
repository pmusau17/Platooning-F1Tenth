cmake_minimum_required(VERSION 3.5)
project(computer_vision)

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
add_definitions(-DSETUPTOOLS_DEB_LAYOUT=OFF)

# install python modules
catkin_python_setup()

# Specify catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs geometry_msgs gazebo_ros
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

### Install python nodes
install(PROGRAMS 
nodes/ensemble_manager.py
nodes/image_listener.py
nodes/multi_image_listener.py
nodes/ros_classifier_fused.py
nodes/ros_classifier.py
nodes/ros_dave.py
nodes/ros_ensemble_node.py
nodes/ros_transfer_imagenet.py
nodes/synchronize_img_command_lidar.py
nodes/synchronize_img_command.py
evaluation/analyze_classification_model.py  
evaluation/analyze_discrete.py  
evaluation/analyze_e2e.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch and other files 
install(DIRECTORY
        launch
        logs
        models
        model_test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
