# Racecar-Pure_Pursuit-Planner
ROS package for a motion planner used in head-to-head autonomous racing.

The ROS package works coupled with the F110 repository for simulations. 
Add the repository and follow the instructions for installing as given in the readme of the repository. 


git clone https://github.com/mlab-upenn/f110-fall2018-skeletons.git

Clone the repository in the source folder of your ROS Workspace. 

clone the a_stars_pure_pursuit in the same Source folder. 

These dependencies are important to install the package: 

1. CSV
2. numpy
3. Scipy
5. clone the repository 
6. Locate the scripts Locate the python scripts ‘pure_pursuit.py’ and make them executable using the
following command.<br />
	$ chmod +x “script name”<br />
*if on compilation or launching the simulation, there is an error where the launch file is not able to locate the node of a python file, just locate that python file and make it executable by the above command<br />

7. source the current setup file from the ROS workspace and make the workspace using command catkin_make<br />
8. Roslaunch the package: <br />
	$ roslaunch a_stars_pure_pursuit pure_pursuit_sim.launch<br />


For any issues mail on sidsingh@seas.upenn.edu or Singh.sid930@gmail.com

This is the reference video: 
https://www.youtube.com/watch?v=UBTp22Loq4o&feature=youtu.be


![](ppt.gif)
