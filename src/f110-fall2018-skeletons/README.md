# f110-fall2018-course
Skeleton codes for F110 Fall 2018 course at UPenn.

In order to add the contents of this repository to your workspace on your local machine, do the following:

Clone this repository into a folder on your computer
```bash
$ cd ~/sandbox (or whatever folder you want to work in)
$ git clone https://github.com/mlab-upenn/f110-fall2018-skeletons.git
```

We begin by creating a workspace. You can call your workspace anything, for the purposes of this setup guide we call our workspace f110_ws. In your root folder, execute these commands:
```bash
$ mkdir -p ~/f110_ws/src
```

Copy the contents of this repository into ~/f110_ws/src
```bash
$ cp -r f110-fall2018-skeletons f110_ws/src
```

You will need to install these with apt-get in order for the car and Gazebo simulator to work.
```bash
$ sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-control ros-kinetic-ackermann-msgs ros-kinetic-joy
```

Make all the Python scripts executable (by default they are set to non-executable when downloaded from Github, or anywhere for that matter)
```bash
find . -name "*.py" -exec chmod +x {} \;
```

Then build:
```bash
$ cd ~/f110_ws
$ catkin_make
```

The generated devel and build folders at the root of the workspace are where the linked libraries and the compiled code in machine language resides. Source the environment using
```bash
$ source devel/setup.bash
```

You can now run a launch file using the following. 
```bash
$ roslaunch gap_finding gap_finding_sim.launch
```

TROUBLESHOOTING
If for some reason you get a build error, try deleting the CMakeLists.txt file and running catkin_make again. 
