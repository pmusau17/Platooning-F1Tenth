# Install script for directory: /home/musaup/Documents/Research/Platooning-F1Tenth/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/musaup/Documents/Research/Platooning-F1Tenth/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/musaup/Documents/Research/Platooning-F1Tenth/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/musaup/Documents/Research/Platooning-F1Tenth/install" TYPE PROGRAM FILES "/home/musaup/Documents/Research/Platooning-F1Tenth/build/catkin_generated/installspace/_setup_util.py")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/musaup/Documents/Research/Platooning-F1Tenth/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/musaup/Documents/Research/Platooning-F1Tenth/install" TYPE PROGRAM FILES "/home/musaup/Documents/Research/Platooning-F1Tenth/build/catkin_generated/installspace/env.sh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/musaup/Documents/Research/Platooning-F1Tenth/install/setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/musaup/Documents/Research/Platooning-F1Tenth/install" TYPE FILE FILES "/home/musaup/Documents/Research/Platooning-F1Tenth/build/catkin_generated/installspace/setup.bash")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/musaup/Documents/Research/Platooning-F1Tenth/install/setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/musaup/Documents/Research/Platooning-F1Tenth/install" TYPE FILE FILES "/home/musaup/Documents/Research/Platooning-F1Tenth/build/catkin_generated/installspace/setup.sh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/musaup/Documents/Research/Platooning-F1Tenth/install/setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/musaup/Documents/Research/Platooning-F1Tenth/install" TYPE FILE FILES "/home/musaup/Documents/Research/Platooning-F1Tenth/build/catkin_generated/installspace/setup.zsh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/musaup/Documents/Research/Platooning-F1Tenth/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/musaup/Documents/Research/Platooning-F1Tenth/install" TYPE FILE FILES "/home/musaup/Documents/Research/Platooning-F1Tenth/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/gtest/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/f110-fall2018-skeletons/cartographer_config/f110_description/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/f110-fall2018-skeletons/system/joystick_drivers/joystick_drivers/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/f110-fall2018-skeletons/system/racecar/racecar/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/f110-fall2018-skeletons/simulator/racecar-simulator/racecar_control/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/f110-fall2018-skeletons/simulator/racecar-simulator/racecar_description/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/f110-fall2018-skeletons/system/serial/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/f110-fall2018-skeletons/system/ackermann_msgs/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/f110-fall2018-skeletons/system/vesc/vesc/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/f110-fall2018-skeletons/system/vesc/vesc_msgs/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/a_stars_pure_pursuit/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/f110-fall2018-skeletons/labs/gap_finding/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/f110-fall2018-skeletons/labs/lab_pure_pursuit/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/f110-fall2018-skeletons/system/racecar/ackermann_cmd_mux/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/f110-fall2018-skeletons/system/hokuyo_node/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/f110-fall2018-skeletons/system/joystick_drivers/joy/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/particle_filter/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/f110-fall2018-skeletons/simulator/f1_10_sim/race/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/f110-fall2018-skeletons/simulator/racecar-simulator/racecar_gazebo/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/f110-fall2018-skeletons/system/vesc/vesc_ackermann/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/f110-fall2018-skeletons/system/vesc/vesc_driver/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/f110-fall2018-skeletons/labs/wall_following/cmake_install.cmake")
  include("/home/musaup/Documents/Research/Platooning-F1Tenth/build/f110-fall2018-skeletons/system/waypoint_logger/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/musaup/Documents/Research/Platooning-F1Tenth/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
