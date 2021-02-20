# Racecar Gazebo

This folder contains the launch files, world files, and other material needed to create the vehicle's environment. This package contains three important nodes. 

- [kill_simulation](scripts/kill_simulation.py):
  - Some of the experiments that we run require a timeout. For example, when testing the efficacy of a reinforcement learning policy, we run 100 experiments of 60 seconds each across several different racetracks. This allows the simulation to terminate after a fixed period.

- [spawn_cone_simulation](scripts/kill_simulation.py)
  - Navigation in the presence of static obstacles is also an interesting reseearch problem. The spawn cone script allows for the random assignment of a configurable number cones within the racetrack environment. This can be done in two main ways. The first strategy uses a file containing the free points within the racetrack envrionment to randomly allocate cones. The second strategy allows for cones to spawned in fixed locations. This can be done by tailoring the spawn code file to your needs.

- [gazebo_odometry](scripts/gazebo_odometry_gen.py)
  -  This file uses the ground truth state information from gazebo to be transmitted as an odometry message. This allows those who are working on localization techniques to test the efficacy of their methods.
