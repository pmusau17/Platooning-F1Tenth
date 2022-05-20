
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
scripts/collision_logger.py             
scripts/constants.py              
scripts/point.py                     
scripts/template_model_hype.py       
scripts/visualize_point_2.py
scripts/compute_track_progress.py       
scripts/mpc_bicycle.py            
scripts/solving_bottom.py            
scripts/template_model.py            
scripts/visualize_point.py
scripts/computing_hyperplanes_final.py  
scripts/mpcc_hyperplanes.py       
scripts/solving.py                   
scripts/template_mpc_black_box.py    
scripts/visualize_points.py
scripts/computing_hyperplanes_left.py   
scripts/mpc_drive_hyperplanes.py  
scripts/solving_upper.py             
scripts/template_mpc_hyperplanes.py
scripts/computing_hyperplanes.py        
scripts/mpc_drive.py              
scripts/template_model_bicycle.py    
scripts/template_mpc.py
scripts/computing_hyperplanes_right.py  
scripts/mpc_gen.py                
scripts/template_model_black_box.py  
scripts/test.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch and other files 
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

