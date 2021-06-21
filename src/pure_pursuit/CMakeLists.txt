cmake_minimum_required(VERSION 3.5)
project(pure_pursuit)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)


catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

### Install python nodes
install(PROGRAMS 
scripts/pure_pursuit_angle.py
scripts/pure_pursuit.py
scripts/visualize_markers.py
scripts/waypoint_logger_pure_pursuit.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch and other files 
install(DIRECTORY
        launch
        maps
        rviz_cfg
        waypoints
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})