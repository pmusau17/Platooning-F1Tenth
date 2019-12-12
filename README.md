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
$ git clone https://github.com/pmusau17/Platooning-F1Tenth
$ cd Platooning-F1Tenth
```
After everything has been cloned in to Platooning-F1Tenth Run:

```bash 
$ chmod u+x setup.bash
$ ./setup.bash
$ catkin_make
```

# Algorithms
 
 **Disparity Extender**
 
 This algorithm was originally proposed by Nathan Otterness et al. in the following [blog post](https://www.nathanotterness.com/2019/04/the-disparity-extender-algorithm-and.html). The main idea behind this algorithm is to search for the farthest distance that the car can travel in a straight line and go in that direction. The algorithm makes use of LIDAR Scans and using the array of distances given by the LIDAR we must filter the ranges in order to find the set of distances tat are safely reachable by driving in a straight line. The basis of this filtering comes from the fact that the LIDAR "thinks" of itself as a single point, but in reality it's on a car that has some width. Thus each time we get a reading from the LIDAR, we take the raw array of LIDAR samples (mapping angles to distances), and identify disparities between subsequent points in the LIDAR samples. For each pair of points around a disparity, we pick the point at the closer distance and calculate the number of LIDAR samples needed to cover half the width of the car at this distance. Using this filtered list of distances we find the index of the farthest index, compute our steering angle and drive in that direction. A more detailed analysis of this algorithm can be found in the above blog post.
 
 To see a demonstration of this algorithm run the following in two seperate terminals:
 
 ```bash 
 $ roslaunch race multi_parametrizeable.launch  
 $ roslaunch race multicar_disparity_extender.launch
 ```
 
 **Following Algorithm**
 
 The task of this project was to get a series of 1/10 scales RC cars to platoon in a reliable manner. As a proof of concept we made a few assumptions. The first assumption is that each car precisely knows its own position and orientaion in the map frame. The second assumption is that the the cars could reliably communicate their position and velocity with each other. However the platooning would occur on a racetrack. Thus there would be times that the following cars would not be able to see the lead car. Thus we needed to design boh a lateral and longitudnal controller that maintained a specified distance between the ego car and the lead car. However if any failures occured in the system, the cars would need to be able to switch to a fallback controller from the following controller. The following controller makes use of the pure pursuit algorith proposed by R. Craig Coulter in the paper ["Implementation of the Pure Pursuit Path 'hcking Algorithm"](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf). To compute the steering angle we use the pure pursuit algorithm to find a goal point in the lead car's position history that consists of the most recent 100 positions. These positions are transmitted from the lead car to the ego car via TCP. The longitudnal distance is observed using the ego car's LIDAR and a PD controller is used to control the ego car's velocity. The fallback controller is the disparity extender controller. One of the benefits of this algorithm is that it can do some obstacle avoidance "out of the box" making it a reliable fallback controller. 
 
 To see a demonstration of this algorithm run the following in two seperate terminals:
 
 ```bash
 $ roslaunch race multi_parametrizeable.launch  
 $ roslaunch race platoon.launch
 ```
 
**Object Tracking**

One of the ways that we can switch from the following controller to the disparity extender controller is by using an object tracking algorithm. If our assumption about having accurate information about the position of each car is no longer valid then we can also platoon visually. Thus we also provide object tracking capabilities. The object tracking code is found in the Object_Tracking folder.


# Running Keyboard nodes

To run a node to tele-operate the car via the keyboard run the following in a new terminal:

``rosrun race keyboard_gen.py racecar1``

'racecar1' can be replaced with 'racecar' 'racecar2' as well. 

test update
