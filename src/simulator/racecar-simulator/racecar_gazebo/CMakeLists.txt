cmake_minimum_required(VERSION 3.5.0)
project(racecar_gazebo)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


### Install python nodes
install(PROGRAMS 
scripts/gazebo_odometry2.py
scripts/gazebo_odometry_gen.py
scripts/gazebo_odometry.py
scripts/kill_simulation.py
scripts/spawn_cone.py
scripts/spawn_zigzag.py
scripts/floating_box_odometry.py
scripts/floating_box_odom_to_tf.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch and other files 
install(DIRECTORY
  worlds
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)