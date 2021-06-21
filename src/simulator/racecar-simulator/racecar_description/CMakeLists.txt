cmake_minimum_required(VERSION 3.5.0)
project(racecar_description)

find_package(catkin REQUIRED)

catkin_package()

# Install launch and other files 
install(DIRECTORY
  meshes
  models
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)