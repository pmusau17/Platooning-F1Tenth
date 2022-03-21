cmake_minimum_required(VERSION 3.5)
project(race)


# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  sensor_msgs
  visualization_msgs
  std_msgs
  message_generation
  geometry_msgs
  tf
  roscpp_serialization
  laser_geometry
)

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

## This macro ensures
## modules and global scripts declared therein get installed
#catkin_python_setup()

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
    reach_tube.msg
)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES race
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs geometry_msgs gazebo_ros visualization_msgs
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
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# add cpp executable
add_executable(laser_converter_node src/laser_to_pointcloud.cpp)
target_link_libraries(laser_converter_node ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES}) 


### Install laser converter node 
install(TARGETS 
  laser_converter_node
  DESTINATION lib/${PROJECT_NAME}
)

### Install python nodes
install(PROGRAMS 
scripts/ttc.py scripts/analyze_odom.py
scripts/collect_lidar_classification_data.py
scripts/collision_logger.py
scripts/collision_tracker.py
scripts/compute_progress.py
scripts/compute_safety_percentage.py
scripts/decision_manager.py
scripts/disparity_extender_vanderbilt_gen.py
scripts/disparity_extender_vanderbilt.py
scripts/follow_lead_gen.py
scripts/follow_lead.py
scripts/gen_map.py
scripts/gen_sysid_data.py
scripts/get_obstacles.py
scripts/independent_sub.py
scripts/keyboard_gen.py
scripts/keyboard.py
scripts/laser_converter.py
scripts/lidar_classification_node.py
scripts/message_to_tf_gen.py
scripts/message_to_tf.py
scripts/pfc.py
scripts/plot_reachability_results_multiple.py
scripts/plot_reachability_results.py
scripts/potential_field_controller.py
scripts/process_and_plot_lidar_data.py
scripts/process_dynamic_obstacles.py
scripts/reset_world.py
scripts/rtreach_decision_manager.py
scripts/safety_controller1.py
scripts/safety_controller.py
scripts/safety_node.py
scripts/save_grid.py
scripts/sim_connector_gen.py
scripts/sim_connector.py
scripts/speed_node.py
scripts/transform_obstacles.py
scripts/ttc.py
scripts/visualize_wall_markers.py
scripts/wall_follower.py
scripts/lidar_to_cartesian.py
scripts/disparity_extender_ftg.py
scripts/dynamic_box_oscillation.py
scripts/send_initial_pose.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch and other files 
install(DIRECTORY
        launch
        rviz
        cfg
        logs
        maps
        models
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

  