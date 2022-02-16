# MPC 

Documentation loading...(I promise)

### Installation of MPC Libraries

``
$ source setup.sh

install [HSL](https://www.do-mpc.com/en/latest/installation.html). Find the instructions in the aforementioned link, you will need to get a license to download the software.
``

### Running the mpc setup:

- mpc_model:=0 (MPC with Follow the Gap)
- mpc_model:=1 (MPC with Pure Pursuit)
- mpc_model:=2 (MPC with Pure Pursuit Nonlinear Model)

```
$ roslaunch mpc mpc.launch mpc_model:=0
```

### Installing the reachability library

**Disclaimer and Warning**: Anaconda and ros noetic don't like each other too much so for the purposes of installation, kindly comment it out for the installation. You can reactivate it afterwards.


Step 1: Clone rtreach (not in the Platooning-F1Tenth Folder):

```
$ git clone https://github.com/pmusau17/rtreach_f1tenth.git -b noetic-port
```

Step 2: Build and Source Platooning-F1Tenth 

```
$ cd Platooning-F1Tenth && colcon build --symlink-install && source install/setup.bash
```

Step 3: Run the mpc launch file 

```
$ roslaunch mpc mpc.launch
```

<!-- rtreach

**In the same terminal**

```
$ cd /path/to/rtreach_f1tenth/
$ ./build_rtreach.sh 
$ cd ../rtreach_ros/
$ source install/setup.bash && roslaunch mpc mpc.launch
```
**returning to previous sucessful installation**

I'm working on refactoring the code so you don't have to do edit the ```LD_LIBRARY_PATH```. But for now:
```
$ cd /path/to/rtreach_f1tenth
$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/src
$ cd /path/to/rtreach_ros
$ source install/setup.bash
$ roslaunch mpc mpc.launch mpc_model....
``` -->
