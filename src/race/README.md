# Race:

![Alt Text](images/race.gif)

This package contains the nodes, launch files, msg files, and infrastructure needed to run the racecar autonomously in a race-track/circuit. The nodes currently available in this package are: 


## Launch Files

The launch files have numerous configurable parameters. We invite you to view them in order to tailor them to your needs.
- [f1_tenth_devel.launch](f1_tenth_devel.launch) 
   - This file will start up a single car by default in the porto environment. No driving nodes are started by default, allowing the user to specify which strategy should be used.
- [f1_tenth_rl.launch](f1_tenth_rl.launch)
   -  This file is similar to f1_tenth_devel but starts the car in a more favorable position within the porto track in order to ease the initial training learning curve. 
- [multi_parametrizeable.launch](multi_parametrizeable.launch)
   - This launch file is used for multi-agent experiments. By default it launches two vehicles but it also allows for 3 cars. 
- [model_comparison.launch](model_comparison.launch)
   - This launch file allows for comparisons between machine learning strategies across different racetracks. It is used by the bash script [run_e2e_model_comparison.sh](batch_scripts/run_e2e_model_comparison.sh). The strategies that we have evaluated using this file include Soft Actor Critic and Deep Deterministic Policy Gradient reinforcement learning strategies as well as end-to-end computer vision techniques.
- [multicar_disparity_extender.launch](multicar_disparity_extender.launch)
   - This file launches a gap following algorithm for use in the multi-agent context provided by the launch file [multi_parametrizeable.launch](multi_parametrizeable.launch).
- [platoon.launch](platoon.launch)
   - This file launches a platooning strategy for two or three cars. The lead car uses a gap following algorithm to navigate while the following cars use a pure-pursuit path tracking algorithm for navigation.

### Real Time Reachability Launch Files
The following launch files require that the following [repository](https://github.com/pmusau17/rtreach_f1tenth) be built and sourced. These launch files set up the simulation environment that enables a runtime assurance architechture to be used. More details can be found at the above link.

- [sim_for_rtreach.launch](sim_for_rtreach.launch)
   -  Creates runtime-assurance architechture for a single vehicle in the presence of a fixed number of static obstacles (cones). The controller that is used for navigation by default is a vision based end-to-end learning regime.
- [sim_for_rtreach_batch.launch](sim_for_rtreach_batch.launch)
   - Allows for the runtime assurance architechture to be evaluated across numerous experimental runs. In order to analyze the runtime performance as well as efficacy in preventing collisions.
- [sim_for_rtreach_rl.launch](sim_for_rtreach_rl.launch)
   - Similar to [sim_for_rtreach.launch](sim_for_rtreach.launch). However the controller that is used is a reinforcement learning controller trained using Augmented Random Search. 
- [sim_for_rtreach_batch_rl.launch](sim_for_rtreach_batch_rl.launch)
   - Allows for the runtime assurance architechture to be evaluated across numerous experimental runs. In order to analyze the runtime performance as well as efficacy in preventing collisions. 
- [sim_for_rtreach_batch_worlds.launch](sim_for_rtreach_batch_worlds.launch)
   - Allows for the runtime assurance architechture to be evaluated across numerous racetrack environments. Default controller is the vision-based end-to-end learning controller.
- [sim_for_rtreach_dynamic.launch](sim_for_rtreach_dynamic.launch)
   - This file creates the run-time assurance architechture in the presence of dynamic obstacles. It makes use of a multi-object object tracking algorithm to classify and track objects within the environment.
- [sim_for_rtreach_multi_agent.launch](sim_for_rtreach_multi_agent.launch)
   - Creates runtime-assurance architechture for a single vehicle in the presence of a fixed number of dynamic obstacles (other agents).
- [slam.launch](slam.launch)
   - Launch file for SLAM algorithms. This file makes use of the gmapping library. For simple racetracks this library is sufficient. However for large racetracks with multiple loops we recommend using Google Cartographer.
- [sys_id.launch](sys_id.launch)
   -  Launch file used to collect data for model estimation of the F1Tenth Vehicle.



## Nodes     

### [Disparity Extender](scripts/disparity_extender_vanderbilt_gen.py)
- Implementation of reactive navigation via a follow the gap technique developped by UNC
### [Follow Lead](scripts/follow_lead_gen.py)
- Node used within platooning to steer an ego vehicle to track the position of the lead car.
### [Keyboard](scripts/keyboard_gen.py)
- Keyboard teleoperation node 
### [Potential Field Controller](scripts/pfc.py)
- Reactive naviation method using artifical potential field techniques.
### [Tf Transform](scripts/message_to_tf.py)
- Transform publisher for proper visualization in rviz
### [Reset World](scripts/reset_world.py)
- Node that executes a service call to reset gazebo environment
### [Speed Node](scripts/speed_node.py)
- Node that issues speed setpoints in a system architechture where speed and steering control are seperate tasks.
### [Decision Manager](scripts/decision_manager.py)
- Decision manager used within a system architechture where speed and steering are independent tasks
### [Contact Listner](scripts/contact_tracker.py)
The contact listener node is a component designed to detect collision between the racecar and other simulated objects in the envrionment. It has two configurations. The first configuration, simply prints a message whenever a collision occurs. The second configuration resets the envrionment. This useful for reinforcement learning tasks.

To run the node execute the following after the simulation is up and running:

```
$  rosrun race collision_tracker.py /racecar
```
To enable resetting the world run:

```
$  rosrun race collision_tracker.py /racecar 1
```
### [Analyze Odom](scripts/analyze_odom.py)
- Node that analyzes odometry information of the vehicle.
### [Generate System Id](scripts/gen_sysid_data.py)
- Node that generates system identification data for the f1tenth model. It logs (x,y,linear_speed,theta) (throttle, steering angle)
### [Safety Controller](scripts/safety_conroller.py)
- Simplistic safety controller based on the disparity extender.
### [System Identification](https://github.com/pmusau17/Platooning-F1Tenth/tree/master/src/race/sys_id)
- Matlab System Identification packages for the f1tenth simulator.

<br/>

**Special Thanks:** This package was inspired by teams at MIT and UPenn

