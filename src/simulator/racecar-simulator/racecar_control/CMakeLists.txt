cmake_minimum_required(VERSION 3.5.0)
project(racecar_control)

find_package(catkin REQUIRED)

catkin_package()

### Install python nodes
install(PROGRAMS 
    scripts/keyboard_teleop.py  
    scripts/servo_commands_gen.py  
    scripts/servo_commands.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch and other files 
install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)