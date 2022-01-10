
cmake_minimum_required(VERSION 2.8.3)
project(mpc)
find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  race
)

catkin_package(
  CATKIN_DEPENDS 
    ackermann_msgs
    geometry_msgs
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

### Install python nodes
install(PROGRAMS 
scripts/constants.py
scripts/mpc_drive.py
scripts/mpc_drive_hyperplanes.py
scripts/mpcc_hyperplanes.py
scripts/mpc_gen.py
scripts/point.py
scripts/solving_bottom.py
scripts/solving_upper.py
scripts/template_model.py
scripts/template_mpc.py
scripts/template_mpc_hyperplanes.py
scripts/visualize_point_2.py
scripts/visualize_point.py
scripts/visualize_points.py
scripts/mpc_bicycle.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch and other files 
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

