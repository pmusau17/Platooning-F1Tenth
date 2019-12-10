# Car-Platooning

# Repository Organization

**f110-fall2018-skeletons**: F1Tenth Simulation Code

**particle_filter**: A fast particle filter localization algorithm developped by Corey Walsh et al.

**a_stars_pure_pursuit**: ROS package for a pure pursuit motion planner developped by Siddharth Singh

# Installation 

We assume that you have ROS Kinetic and Gazebo installed. If not please installed these packages using the following link: [Install ROS-Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu).

Once you have ROS-Kinetic installed. Run the following commands:

```bash
$ source /opt/ros/kinetic/setup.bash
$ mkdir catkin_ws
$ cd catkin_ws
$ git init
$ git remote add origin [my-repo]
$ git fetch
$ git checkout origin/master -ft
```
After everything has been cloned in to catkin_ws (or the folder name of your choosing). Run:

```bash 
$ chmod u+x setup.bash
$ ./setup.bash
$ catkin_make
```


# Algorithms
 
 **Disparity Extender**
 
 **Following Algorithm**

# Running Keyboard nodes

To run a node to tele-operate the car via the keyboard run the following in a new terminal:

``rosrun race keyboard_gen.py racecar1``

'racecar1' can be replaced with 'racecar' 'racecar2' as well. 
