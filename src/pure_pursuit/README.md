# Path Tracking: Pure Pursuit

Implementation of the pure pursuit algorithm proposed by R. Craig Coulter in the paper ["Implementation of the Pure Pursuit Path Tracking Algorithm"](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf).

### Description 

The pure pursuit algorithm is a path tracking algorithm that seeks to determine the speed and steering angle neccessary to control a robot to follow a specified path. A path consists of a set of points representing the positional coordinates of a particular course. Intuitively, what the algorithm does is obtain a goal-point, then aim the vehicle
towards that point. The algorithm has a single tunable parameter called the lookahead distance and that dictates how the goal point is selected. 

### Waypoints 

To collect waypoints you must specify two things:  The world name and the name of the car launched in the simulation. The available world names are listed below. 
As an example to collect waypoints in the track_porto world run the following and then drive the car manually with the keyboard:

Terminal 1:

``` 
roslaunch race f1_tenth_devel.launch world_name:=track_porto enable_keyboard:=true
```


Terminal 2: 
``` 
rosrun pure_pursuit waypoint_logger_pure_pursuit.py /racecar
```

#### Available Worlds 

- track_barca: Large racetrack usually used to evaluate efficacy of algorithms developed on racetracks
- track_levine: Racetrack inspired by the Levine building at UPenn
- track_porto: Simple racetrack with small and wide turns
- racecar_walker: Beautiful track with multiple paths,
- racecar_cones: Empty world with safety cones spaced evenly.
- racecar_tunnel: World with car located in a tunnel with high walls
- racecar_parking_1: Racecar world for you to test your paralle parking skills
- lab_track2: Simple racetrack modeled after laboratory space at Vanderbilt University



### Running Experiments

```
rosrun pure_pursuit pure_pursuit.py /racecar track_porto_26780.csv
```

### Visualizing Experiments
```
rosrun pure_pursuit visualize_markers.py track_porto_26780.csv
```
