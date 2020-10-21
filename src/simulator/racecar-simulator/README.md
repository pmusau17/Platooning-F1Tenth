Racecar Simulator
=================
Forked from www.github.com/mlab-upenn/racecar-simulator, which was forked from
www.github.com/mit-racecar/racecar-simulator. 

Used Google Sketchup to model Levine Building at Penn, 2nd floor hallways right
outside the mLab. Added a .dae file. World units are in meters and are pretty
accurate based on architectural floor plans obtained from the University.

You will need to git clone the following repositories in order for this to work.
The structure will look something like this: sims_ws/src/

Into this src folder you will git clone this repository as well as the following:
git clone https://github.com/wjwwood/serial.git
git clone https://github.com/mit-racecar/racecar.git
git clone https://github.com/crisscrosskao/racecar-simulator
git clone https://github.com/mit-racecar/vesc.git
git clone https://gitub.com/ros-drivers/ackermann_msgs.git
git clone https://github.com/mlab-upenn/f1_10_sim.git

Run "roslaunch race f1_tenth.launch". Note that f1_tenth.launch file will
need to be modified so that it loads the track_levine.world instead of the default
track_barca.world.

Additionally, included in this repository is the Sketchup file for the levine_track.
Feel free to improve on it and make modifications.
