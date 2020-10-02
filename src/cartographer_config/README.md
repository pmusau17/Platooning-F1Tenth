# Installing Cartographer

To install cartographer create a new workspace outside of this one. 

```
mkdir carto_ws
```

Create a file called cartographer_ros.rosinstall and place the following lines within this file:

```
- git: {local-name: cartographer, uri: 'https://github.com/cartographer-project/cartographer.git', version: 'master'}
- git: {local-name: cartographer_ros, uri: 'https://github.com/cartographer-project/cartographer_ros.git', version: 'master'}
- git: {local-name: ceres-solver, uri: 'https://ceres-solver.googlesource.com/ceres-solver.git', version: '1.13.0'}
```

Then: 

```
$ cd carto_ws
$ wstool merge -t src ../cartographer_ros.rosinstall
$ src/cartographer/scripts/install_proto3.sh
$ sudo rosdep init
$ rosdep update
$ rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
```

Build and install: 

```
catkin_make_isolated --install --use-ninja
source install_isolated/setup.bash
```

Test the installation: 
```
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag
roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag
```

### Once the Cartographer demo is working: 

- Copy the f110_2d_viz.launch f110_2d.launch into the src/cartographer_ros/cartographer_ros/launch/ directory. 
- Copy the f110_2d.lua into the src/cartographer_ros/cartographer_ros/configuration_files/ directory
