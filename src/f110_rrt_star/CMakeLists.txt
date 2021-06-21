cmake_minimum_required(VERSION 3.5.0)
project(f110_rrt_star)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
# set (CMAKE_CXX_FLAGS -O3)
set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES f110_rrt
 CATKIN_DEPENDS ackermann_msgs geometry_msgs roscpp rospy sensor_msgs std_msgs tf
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rrt_node node/rrt_node.cpp src/rrt.cpp)
add_dependencies(rrt_node ${f110_rrt_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(rrt_node
  ${catkin_LIBRARIES}
)

### Install laser converter node 
install(TARGETS 
  rrt_node
  DESTINATION lib/${PROJECT_NAME}
)