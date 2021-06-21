cmake_minimum_required(VERSION 3.5.0)
project(multi_object_tracking_lidar)

set(CMAKE_CXX_STANDARD 14)


find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)
find_package( OpenCV REQUIRED )

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_executable( kf_tracker src/main.cpp )
target_link_libraries ( kf_tracker ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
 
## Mark executables and/or libraries for installation
install(TARGETS kf_tracker
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

