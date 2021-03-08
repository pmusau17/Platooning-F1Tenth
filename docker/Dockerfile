#pull from the osrf full kinetic build
#FROM nvidia/cudagl:9.0-base-ubuntu16.04
FROM nvidia/cudagl:10.1-base-ubuntu16.04
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

#Need these packages for debugging
RUN apt-get install -y nano
RUN apt-get install -y net-tools

RUN pip install --upgrade pip && apt-get remove -y python-enum34 python-pyasn1-modules && pip2 install numpy scipy &&  pip2 install tensorflow-gpu && apt-get install -y python-opencv && apt-get install -y ros-kinetic-cv-bridge ros-kinetic-vision-opencv && pip install pathlib

#navigate to the home directory
WORKDIR home
RUN pip install gdown
RUN gdown https://drive.google.com/uc?id=152KL7JzDReYdg6quBznL9WtC0I5IvNx6
RUN apt-get install unzip && unzip rl_library.zip 
WORKDIR rl_library
RUN pip install -e .
WORKDIR ..


RUN git clone https://github.com/pmusau17/Platooning-F1Tenth
WORKDIR Platooning-F1Tenth 

# try eigen three installation
RUN git pull && apt-get update && apt-get install -y ros-kinetic-pcl-conversions ros-kinetic-pcl-msgs ros-kinetic-pcl-ros libeigen3-dev libproj-dev libmove-base-msgs-dev
RUN ln -s /usr/include/eigen3/Eigen /usr/local/include/Eigen
RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash && catkin_make'

# get the model files so that there's no lag in starting gazebo
RUN apt-get install wget && wget -l 2 -nc -r "http://models.gazebosim.org/" --accept gz && mkdir /usr/share/gazebo-7/models/
WORKDIR models.gazebosim.org
RUN ls /usr/share/gazebo && ls /usr/share/gazebo-7 && for i in *; do tar -zvxf "$i/model.tar.gz";  done && mv * /usr/share/gazebo-7/models/
WORKDIR .. 
RUN rm -r models.gazebosim.org

RUN apt-get install -y ros-kinetic-gazebo-ros-pkgs

# Pytorch python 2.7
RUN pip install torch torchvision
RUN pip install future
RUN pip install keras==2.3.1

CMD /bin/bash -c "source devel/setup.bash && roslaunch race f1_tenth_devel.launch"