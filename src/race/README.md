# Race:

![Alt Text](images/race.gif)

This package contains the nodes, launch files, msg files, and infrastructure needed to run the racecar autonomously in a race-track/circuit. The nodes currently available in this package are: 

### [Contact Listner](scripts/contact_listener_node.py)
    The contact listener node is a component designed to detect collision between the racecar and other simulated objects in the envrionment. It has two configurations. The first configuration, simply prints a message whenever a collision occurs. The second configuration resets the envrionment. This useful for reinforcement learning tasks.

    To run the node execute the following after the simulation is up and running:

    ```bash
    $  rosrun race contact_listener_node.py /racecar
    ```

    To enable resetting the world run: 

     ```bash
    $  rosrun race contact_listener_node.py /racecar 1
    ```

    
### [Decision Manager](scripts/decision_manager.py)
### [Disparity Extender](scripts/disparity_extender_vanderbilt_gen.py)
### [Follow Lead](scripts/follow_lead_gen.py)
### [Keyboard](scripts/keyboard_gen.py)
### [Potential Field Controller](scripts/pfc.py)
### [Tf Transform](scripts/message_to_tf.py)
### [Reset World](scripts/reset_world.py)
### [Speed Node](scripts/speed_node.py)


**Special Thanks:** This package was inspired by teams at MIT and UPenn

