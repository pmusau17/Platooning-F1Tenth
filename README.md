# F1Tenth: Platooning, Computer Vision, Reinforcement Learning, Path Planning

**I'm working on porting this repo to ROS2 and Noetic. I'll update this readme when it's complete. For now if you want to help with the port or experiment with it checkout the corresponding branch.**

# Table of Contents


1. [Introduction](#introduction)
2. [Installation](#Installation)
3. [Runtime Verification](#RuntimeVerification)
4. [Zero-Shot Policy Transfer](#ZeroShot)
5. [Platooning](#Platooning)
6. [Computer Vision](#ComputerVision)
7. [Reinforcement Learning](#ReinforcementLearning)
8. [System Identification](https://github.com/pmusau17/Platooning-F1Tenth/tree/master/src/race/sys_id)
9. [Docker](#Docker)
10. [Developers](#Developers)


# Introduction <a name="introduction"></a>


Welcome! This repository contains a host of [ROS](http://wiki.ros.org/ROS/Introduction) packages for the [F1Tenth Autonomous Racing Competition](http://f1tenth.org/). In this repository you will find ROS packages designed for both simulation and the F1Tenth hardware platform. The simulation is based on a Gazebo based virtual racing environment. Our goal is to develop the F1Tenth platform for AI research to experiment with deep learning, computer vision, reinforcement learning, and path planning for autonomous racing.  

If you have any questions or run into any problems. Feel free to send me an [email](mailto:patrick.musau@vanderbilt.edu) or to post an issue and I'll do my best to get back to you promptly. If you think something needs more documentation or explaining please send me an email too (I mean it).  

![Three_Car_Sim](./images/three_car_platoon.gif "Three Car Simulation")

# Installation <a name="Installation"></a>


### Install Necessary Environments
<hr /> 

Installation has been tested on Ubuntu 16.04 and 18.04 LTS. We highly recommend using the dockerized version of the simulator. The instructions can be found at the botton of this file. For a native installation see below.

The computer vision packages contained in this repository assume your system is GPU enabled. If your system is GPU enabled, you will need to install cuda and [cudnn](https://developer.nvidia.com/cudnn-download-survey). New versions of cuda are released periodically each year. Thus to keep this repo up to date, we refer you to nvidia's installation guide [here](https://developer.nvidia.com/cuda-10.1-download-archive-update2?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1604). Cudnn requires membership in the NVIDIA developer program and you can register for this program [here](https://developer.nvidia.com/cudnn-download-survey).

For ease of python library installation we highly recommend using anaconda. Installation of Anaconda can be found [here](https://docs.anaconda.com/anaconda/install/linux/). ROS still uses [python 2](https://ubuntu.com/blog/psa-for-ros-users-some-things-to-know-as-python-2-approaches-eol) and if you want to use python 3, we leave that adventure to you. 

Once anaconda installed run the following: 

```
$ conda create --name ros27 python=2.7 && conda activate ros27
```

**Note:** If your system is not gpu enabled, change the requirements in [setup.sh](setup.sh) to requirements-cpu.txt.


### Installing Pytorch
<hr /> 

To install pytorch kindly visit the following [link](https://pytorch.org/get-started/locally/).

### Installing Reinforcement Learning Libraries
<hr /> 

Our reinforcement Learning Approaches leverage a couple of custom libraries. To install them run the following. 

```
pip install gdown
gdown https://drive.google.com/uc?id=152KL7JzDReYdg6quBznL9WtC0I5IvNx6
```

Unzip the rl_library.zip file. 

```
cd rl_library
pip install -e .
```


### Install the Repo
<hr /> 

If you already have these environments set up, then go with this choice. We assume that you have ROS Kinetic and Gazebo installed. If not please installed these packages using the following link: [Install ROS-Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu).

Once you have ROS-Kinetic installed. Run the following commands:

```bash
$ source /opt/ros/kinetic/setup.bash
$ git clone https://github.com/pmusau17/Platooning-F1Tenth
$ cd Platooning-F1Tenth
```
After everything has been cloned in to Platooning-F1Tenth Run:

```bash 
$ chmod u+x setup.bash
$ ./setup.sh
$ catkin_make
$ source devel/setup.bash
```
### Troubleshooting 
<hr /> 

If you get the error: ImportError: No module named catkin_pkg.packages and you are using anaconda. Run the following command: ```conda install -c auto catkin_pkg```


# Runtime Verification<a name="RuntimeVerification"></a>

Recent advances in Artificial Intelligence research have led to the emergence of machine learning models deployed within safety critical systems, where their tremendous ability to deal with complex data makes them particularly useful for sensing, actuation, and control. Despite the prolific advances enabled by machine learning methods, such systems are notoriously difficult to assure. The challenge here is that some models, such as neural networks, are "black box" in nature, making verification and validation difficult, and sometimes infeasible. In this repo, we provide the code used for our evaluation of the use of a real-time reachability algorithm in order to reason about the safety of a 1/10 scale open source autonomous vehicle platform known as F1/10. Our regime allows us to (a) provide provable guarantees of safety and (b) detect potentially unsafe scenarios in the context of autonomous racing.

There are two options for interacting with our code, a native installation or running the code through Docker. Our methods were run on a Dell Optiplex 7050 running Ubuntu 16.04 LTS, ROS Kinetic, and equipped with a GeForce GTX 1080 GPU. The real-time reachability code is avaliable [here](https://github.com/pmusau17/rtreach_f1tenth) and is required in order to run experiments.

### Native Install 
<hr /> 

To use runtime verification code, you must first step clone and build this repo as a rospackage. The instructions for doing so are provided above. Once this is done. Please add the path to the devel/setup.bash to your ~/.bashrc file, as shown below. **Change this path to the one on your system**.

```
source /path/to/Platooning-F1Tenth/devel/setup.bash 
```

One this is done. Either close and reopen your terminal or run the following ```source ~/.bashrc```

Next clone the [rtreach_f1tenth](https://github.com/pmusau17/rtreach_f1tenth) repository outside of this one and run the following: 

```
cd rtreach_ft1enth
 ./build_rtreach.sh
 cd ../rtreach_ros && source devel/setup.bash
```

With that you are now ready to run the experiments. 

### Docker 
<hr /> 

Instruction for building and running the dockerized version of the runtime-verification approach are provided [here](https://github.com/pmusau17/rtreach_f1tenth).


### Running the Experiments (Native Installation)
<hr /> 

We provide launch files for running the relevant experiments:
- [sim_for_rtreach_multi_agent.launch](src/race/launch/sim_for_rtreach_multi_agent.launch)
  -  This file launches a simulation with two vehicles. The vehicle for which we are reasoning about safety is racecar2. As a demonstration we have implemented a naive simplex architechture (simple switch based on safety signal). The launch file allows for various parameters to be set, such as the number of cars (2 or 3), the velocity used a setpoint for racecar2, the reach_time utilized for verification, the walltime alloted for reachset computation, and the number reachset boxes to displayed in rviz.
- [sim_for_rtreach_multi_agent_batch.launch](src/race/launch/sim_for_rtreach_multi_agent_batch.launch)
  -  This file is the launch file used for the empirical evaluation of our regime. It is used by the following [bash script](src/race/batch_scripts/run_model_safety_comparison.sh). The evaluation was done with respect to varying speeds and different types of controllers. The options can be found in the aformentioned bash script. The results of these experiments were stored in the [logs](src/race/logs/) folder and summarized in the following [jupyter notebook](src/race/logs/IROS_Experiments.ipynb). 
- [sim_for_rtreach_multi_agent_multiple_controller.launch](src/race/launch/sim_for_rtreach_multi_agent_multiple_controller.launch)
  - One of the motivations in utilizing the runtime verification approach is that it abstracts away the need to analyze the underlying controller and instead focuses on the effects of control decisions on the system's future states. Utilizing our regime, one could evaluate the safety of a set of controllers at runtime, and select the one with the highest performance that has been determined safe. This is what this launch file provides.

Example Launch 

```
roslaunch race sim_for_rtreach_multi_agent.launch number_of_cars:=3
```

# Zero-Shot Policy Transfer<a name="ZeroShot"></a>

There are few technologies that hold as much promise in achieving safe, accessible, and convenient transportation as autonomous vehicles. However, as recent years have demonstrated, safety and reliability remain the most critical challenges, especially in complex domains. Autonomous racing has demonstrated unique benefits in that researchers can conduct research in controlled environments allowing for experimentation with approaches that are too risky to evaluate on public roads. In this repo you will find scripts for evaluating two leading methods for training neural network controllers, Reinforcement Learning and Imitation Learning, for the autonomous racing task. We compare their viability by analyzing their performance and safety when deployed in novel scenarios outside their training via zero-shot policy transfer. 

The controllers were evaluated through a variety of scenarios that test their ability to still perform optimally in scenarios outside of their training. These scenarios include changing the constant speed value, adding obstacles to the track, evaluating on a different track, and a real-world evaluation on our [hardware platform](https://github.com/pmusau17/F1TenthHardware). We tracked the number of collisions as well as the number of collisions that occured during the experiments. 


### Docker 
<hr /> 

Build the docker containers as described here: [Docker](#Docker).

Run the experiments as follows: **make sure you've run +xhost: docker**

```
./docker/run_docker.sh
```

This should open a terminal within the docker container. Run the following in the terminal:

```
source devel/setup.bash && roslaunch race model_comparison.launch world_number:=0 algorithm:=0 velocity:=1.0
```

To change the algorithm, velocity, and track edit the parameters in the above command. The mapping is as follows:

```
world_number:=0 -> track_porto
world_number:=1 -> racecar_walker
world_number:=2 -> track_barca
world_number:=3 -> racear_tunnel
world_number:=4 -> track_levine

algorithm:=0 -> end-to-end controller
algorithm:=1 -> soft actor critic (rl)
algorithm:=2 -> ddpg (rl)
algorithm:=3 -> end-to-end controller (trained on all tracks)
algorithm:=4 -> end-to-end controller (trained only on track_barca)
algorithm:=5 -> end-to-end controller (trained on only racecar walker)
```

### Running the Experiments (Native Installation)
<hr /> 

The experiments without obstacles can be run by the following [bash script](src/race/batch_scripts/run_model_comparison.sh) and those with obstacles can be run with this [script](src/race/batch_scripts/run_model_comparison_obstacles.sh).
- [model_comparison.launch](src/race/launch/model_comparison.launch)
  - This launch file allows for a selection of 5 different race tracks (Our experiments involved 3 of the 5), 6 different controllers (Our experiments involved 4 of the 6), the speed is also is parametrizeable.   
- [model_comparison_obstacles.launch](src/race/launch/model_comparison_obstacles.launch)
  - This file is similar to the above file but also allows for a number of obstacles to be randomly allocated within the environment. We set a random seed to make the experiments reproducible. 

Example launch: 

```
roslaunch race model_comparison.launch world_number:=0 algorithm:=0 velocity:=1.0
```


# Platooning Algorithms<a name="Platooning"></a>

 
### Disparity Extender
<hr /> 
 
 This algorithm was originally proposed by Nathan Otterness et al. in the following [blog post](https://www.nathanotterness.com/2019/04/the-disparity-extender-algorithm-and.html). The main idea behind this algorithm is to search for the farthest distance that the car can travel in a straight line and go in that direction. The algorithm makes use of LIDAR Scans and using the array of distances given by the LIDAR we must filter the ranges in order to find the set of distances tat are safely reachable by driving in a straight line. The basis of this filtering comes from the fact that the LIDAR "thinks" of itself as a single point, but in reality it's on a car that has some width. Thus each time we get a reading from the LIDAR, we take the raw array of LIDAR samples (mapping angles to distances), and identify disparities between subsequent points in the LIDAR samples. For each pair of points around a disparity, we pick the point at the closer distance and calculate the number of LIDAR samples needed to cover half the width of the car at this distance. Using this filtered list of distances we find the index of the farthest index, compute our steering angle and drive in that direction. A more detailed analysis of this algorithm can be found in the above blog post.
 
 To see a demonstration of this algorithm run the following in two seperate terminals:
 
 ```bash 
 $ roslaunch race multi_parametrizeable.launch  
 $ roslaunch race multicar_disparity_extender.launch
 ```
 

![Two_Car_Sim](./images/two_car_sim.gif "Two Car Simulation")


### Following Algorithm
<hr /> 

 The task of this project was to get a series of 1/10 scales RC cars to platoon in a reliable manner. As a proof of concept we made a few assumptions. The first assumption is that each car precisely knows its own position and orientaion in the map frame. The second assumption is that the the cars could reliably communicate their position and velocity with each other. However the platooning would occur on a racetrack. Thus there would be times that the following cars would not be able to see the lead car. Thus we needed to design boh a lateral and longitudnal controller that maintained a specified distance between the ego car and the lead car. However if any failures occured in the system, the cars would need to be able to switch to a fallback controller from the following controller. The following controller makes use of the pure pursuit algorithm proposed by R. Craig Coulter in the paper ["Implementation of the Pure Pursuit Path Tracking Algorithm"](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf). To compute the steering angle we use the pure pursuit algorithm to find a goal point in the lead car's position history that consists of the most recent 100 positions. These positions are transmitted from the lead car to the ego car via TCP. The longitudnal distance is observed using the ego car's LIDAR and a PD controller is used to control the ego car's velocity. The fallback controller is the disparity extender controller. One of the benefits of this algorithm is that it can do some obstacle avoidance "out of the box" making it a reliable fallback controller. 
 
 To see a demonstration of this algorithm run the following in two seperate terminals:
 
 Terminal 1: 
 ```bash
 $ roslaunch race multi_parametrizeable.launch  
 ```
 
 Terminal 2: 
 ```bash  
 $ roslaunch race platoon.launch
 ```

# Changing The Number of Cars and The Track


Changing the number of cars can be done in the [multi_parametrizeable.launch](src/f110-fall2018-skeletons/simulator/f1_10_sim/race/multi_parametrizeable.launch) at the top of the file. You can experiment with two or three car experiments. Beyond that Gazebo operates painfully slow. 

To experiment with two or three cars, pass the argument number_of_cars to the multi_paramterizable launch file as shown below:

```
$ roslaunch race multi_parametrizable.launch number_of_cars:=2
```

or 

```
$ roslaunch race multi_parametrizable.launch number_of_cars:=3
```

At the moment, the launch files are designed to use the track_porto world file. However you can change this in the launch files by editing the world_name parameter.

For single car experiments you can change the track by editing the world_name parameter in [f1_tenth_devel.launch](src/f110-fall2018-skeletons/simulator/f1_10_sim/race/f1_tenth_devel.launch) at the top of the file.

# Running the Experiments (Native Installation).

If you would like to run the platooning experiments without the launch files. You can execute the following commands in seperate terminals. 

Terminal 1:

```
$ roslaunch race multi_parametrizable.launch number_of_cars:=2
```

or 

```
$ roslaunch race multi_parametrizable.launch number_of_cars:=3
```

Terminal 2:

The lead car will use whatever driving algorithm you select to navigate the track. In this case we utilized the disparity extender. 

``` 
$ rosrun race disparity_extender_vanderbilt.py
```
Terminal 3:

The syntax for running the following algorithm is the following:

``` 
$ rosrun race follow_lead_gen.py (lead_car name) (ego car name)
```

Example: 

``` 
$ rosrun race follow_lead_gen.py racecar racecar2
```

Terminal 4:

``` 
$ rosrun race follow_lead_gen.py racecar2 racecar3
```

# Computer Vision <a name="ComputerVision"></a>

Right now most things are limited to a single car. Multi-car experiments are a work in progress. Details can be found in the [Computer Vision](src/computer_vision) package.

![Error Analysis](./images/Figure_2.png "Error Analysis")

# Reinforcement Learning <a name="ReinforcementLearning"></a>

These methods are designed for training a vehicle follower, which we plan on expanding to platooning. However, we might 
expand this to train a single racer in the future.

Details can be found in the [Reinforcement Learning](src/rl) package.

# Reset the Environment

If the car crashes or you want to start the experiment again. Simply run:



to restart the experiment.

# Running teleoperation nodes

To run a node to tele-operate the car via the keyboard run the following in a new terminal:

```bash
$ rosrun race keyboard_gen.py racecar
```

'racecar' can be replaced with 'racecar1' 'racecar2' if there are multiple cars. 

Additionally if using the f1_tenth_devel.launch file, simply type the following:

```bash
$ roslaunch race f1_tenth_devel.launch enable_keyboard:=true
```

# Docker <a name="Docker"></a>

The first thing you will need to install is [NVIDIA-Docker](https://github.com/NVIDIA/nvidia-docker) which is used to containerize and run GPU accelerated workloads. The computer vision packages in this repository will run faster if you have a GPU. Visit the above link to do so.

Additionally we make use of [Docker-Compose](https://docs.docker.com/compose/install/)  to define and run the simulation. Kindly install this as well. 

To build the docker image use the Dockerfile located in this repository. 

```bash
$ docker build -t simulator -f docker/Dockerfile .
```

Test if the image builds correctly by running: 

```bash
$ docker container run --rm --runtime=nvidia -it -e DISPLAY --net=host --env="QT_X11_NO_MITSHM=1" -v /tmp/.X11-unix:/tmp/.X11-unix simulator
```

In order to  enable the use of graphical user interfaces within Docker containers such as Gazebo and Rviz give docker the rights to access the X-Server with:

```bash
$ xhost +local:docker
``` 

This command allows one to connect a container to a host's X server for display **but it is not secure.** It compromises the access control to X server on your host. So with a little effort, someone could display something on your screen, capture user input, in addition to making it easier to exploit other vulnerabilities that might exist in X.
 
**So When you are done run :** 

```bash
$ xhost -local:docker 
``` 

to return the access controls that were disabled with the previous command

To run the simulation: 

```bash
$ docker container run --rm --name=sim --runtime=nvidia -it -e DISPLAY --net=host --env="QT_X11_NO_MITSHM=1" -v /tmp/.X11-unix:/tmp/.X11-unix simulator
```

To teleoperate the car or run experiments run the following:

```bash
$ docker container exec -it sim bash 
```

Then run: 
```bash 
$ source devel/setup.bash && rosrun race keyboard.py
```

# Developers <a name="Developers"></a> 

This work was made possible by the following team of graduate and undergraduate students from Vanderbilt University working at the [Institute for Software Integrated Systems](https://www.isis.vanderbilt.edu/).

* [Patrick Musau](https://www.linkedin.com/in/musaup/)
* [Diego Manzanas Lopez](https://www.linkedin.com/in/diego-manzanas-3b4841106/)
* [Nathaniel (Nate) Hamilton](https://www.linkedin.com/in/nathaniel-hamilton-b01942112/)
* [Diandry Rutayisire](https://www.linkedin.com/in/diandry-rutayisire-298a45153/)
* [Tim Darrah](https://www.linkedin.com/in/timothydarrah/)
* [Latif Gbadamoshie](https://www.linkedin.com/in/abdul-latif-gbadamoshie/)
* [Shreyas Ramakrishna](https://www.linkedin.com/in/shreyasramakrishna/)
* [Tim Krentz](https://www.linkedin.com/in/tim-krentz-15042585/)
