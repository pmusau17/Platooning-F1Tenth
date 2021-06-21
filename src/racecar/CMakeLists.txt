cmake_minimum_required(VERSION 3.5.0)
project(racecar)

find_package(catkin REQUIRED)


catkin_package(
  CATKIN_DEPENDS
    razor_imu_9dof
    tf
    tf2_ros
    urg_node
    joy
    rosbag
    rostopic
    rviz
    gmapping
    hector_mapping
    robot_pose_ekf
)

install(DIRECTORY 
  launch
  config 
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

