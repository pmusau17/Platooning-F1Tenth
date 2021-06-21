cmake_minimum_required(VERSION 3.5)
project(race)


# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX or CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

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
  laser_geometry
)

#Generate messages in the 'msg' folder
add_message_files(
		FILES
    angle_msg.msg
    prediction.msg
    velocity_msg.msg
		drive_param.msg
		drive_values.msg
    pid_input.msg
    stamped_ttc.msg
    interval.msg
    obstacle_list.msg 
)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES race
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs geometry_msgs gazebo_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(laser_converter_node src/laser_to_pointcloud.cpp)
target_link_libraries(laser_converter_node ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIR})


### Install laser converter node 
install(TARGETS 
  laser_converter_node
  DESTINATION lib/${PROJECT_NAME}
)


# Install launch and other files 
install(DIRECTORY
        launch
        rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})