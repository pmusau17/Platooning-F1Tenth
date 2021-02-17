# Simulator

The simulation package is a meta-package that contains several packages designed to get the F1Tenth Car running in the Gazebo. It was derived from the 
f110-upenn-course, and we have made several changes to the packages in order to enable multi-vehicle simulation, reinforcement learning training, and computer vision experiments. 
Some of the changes include the addtion of a contact sensor that enables detection of collisions with objects in the environment as well as with other vehicles.

Inside the folder [racecar_simulator](racecar_simulator) you will find the following folders:

- [racecar_gazebo](racecar-simulator/racecar_gazebo)
    - This folder contains the gazebo world files describing the various racecar environments, along with scripts that "simulate" odometry.
- [racecar_description](racecar-simulator/racecar_description)
    - This folder contains the racecar description (.xacro,.macros,.gazebo) files that describe the cars texture and physics properties.
- [racecar_control](racecar-simulator//racecar_control)
    - This folder sets up the ros_control packages used to send actuation commands to the F1Tenth Car.




