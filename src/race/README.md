# Race:

![Alt Text](images/race.gif)

This package contains the nodes, launch files, msg files, and infrastructure needed to run the racecar autonomously in a race-track/circuit. The nodes currently available in this package are: 


   

     

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

    ```bash
    $  rosrun race collision_tracker.py /racecar
    ```

    To enable resetting the world run: 

     ```bash
    $  rosrun race collision_tracker.py /racecar 1
    
### [Analyze Odom](scripts/analyze_odom.py)
- Node that analyzes odometry information of the vehicle.
### [Generate System Id](scripts/gen_sysid_data.py)
- Node that generates system identification data for the f1tenth model. It logs (x,y,linear_speed,theta) (throttle, steering angle)
### [Safety Controller](scripts/safety_conroller.py)
- Simplistic safety controller based on the disparity extender.

<br/>

**Special Thanks:** This package was inspired by teams at MIT and UPenn

