#pull from the osrf full kinetic build
FROM nvidia/cudagl:9.0-base-ubuntu16.04
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES},display

#install ros
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y ros-kinetic-desktop-full && rosdep update && apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
RUN apt-get update &&  apt-get install -y ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-control ros-kinetic-ackermann-msgs ros-kinetic-joy
RUN apt-get update &&  apt-get install -y ros-kinetic-teb-local-planner ros-kinetic-move-base ros-kinetic-navigation ros-kinetic-hector-slam ros-kinetic-driver-common ros-kinetic-actionlib


#install pip
RUN apt-get install -y python-pip && apt-get install -y python3-pip

RUN pip install rospkg defusedxml PySide2
RUN pip install empy 

#navigate to the home directory
WORKDIR home
RUN mkdir -p catkin_ws
WORKDIR catkin_ws
RUN git clone https://github.com/pmusau17/Platooning-F1Tenth src
RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash && catkin_make'



