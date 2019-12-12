#!/bin/bash


### NOTE! READ THIS FIRST
# also note, read through the script before executing, there are other notes
<<'NVIDIA-DRIVERS' (this is a comment block)

	this will vary depending on your system and install
	(1) in a terminal execute the command: nvidia-smi
 	you should get the following output (or similar)

Thu Dec 12 08:02:44 2019       
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 418.56       Driver Version: 418.56       CUDA Version: 10.1     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  GeForce RTX 2060    Off  | 00000000:01:00.0 Off |                  N/A |
| N/A   41C    P8     5W /  N/A |    351MiB /  5904MiB |      9%      Default |
+-------------------------------+----------------------+----------------------+
                                                                               
+-----------------------------------------------------------------------------+
| Processes:                                                       GPU Memory |
|  GPU       PID   Type   Process name                             Usage      |
|=============================================================================|
|    0      1159      G   /usr/lib/xorg/Xorg                           214MiB |
|    0      1967      G   compiz                                        46MiB |
|    0      2857      G   ...uest-channel-token=14945353663812225716    89MiB |
+-----------------------------------------------------------------------------+

	also (2) in a terminal execute the command: prime-select query
	you should get the following output:

	nvidia

	if (1) is good and (2) isn't, then run the command: sudo prime-select nvidia
	and reboot. if (1) is not good then boot in recovery mode and drop to a shell 
	session. run the following commands: 
	apt-get remove --purge nvidia* 
		followed by: 
	ubuntu-drivers devices 
		you should see your nvidia drivers. then run:
	ubuntu-drivers autoinstall
		reboot

	if anything else consult google, there are plenty of guides available on this
	you can continue with the installation but wait to install cuda and opencv
'''
NVIDIA-DRIVERS


##################################################################################################


# first get anaconda downloading in the background
# valid as of Dec 12 2019
read -p "Do you already have anaconda downloaded? (y/n) " -n 1 -r
echo    # (optional) move to a new line
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
    nohup wget -O anaconda_installer.sh https://repo.anaconda.com/archive/Anaconda3-2019.10-Linux-x86_64.sh > /dev/null 2>&1 &
fi


# get opencv downloads going too
read -p "Do you already have anaconda downloaded? (y/n) " -n 1 -r
echo    # (optional) move to a new line
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
    nohup wget -O opencv.zip https://github.com/Itseez/opencv/archive/3.2.0.zip & > /dev/null 2>&1 &
    nohup wget -O opencv_contrib.zip https://github.com/Itseez/opencv_contrib/archive/3.2.0.zip & > /dev/null 2>&1 &
fi

# get cuda downloads going. Note, to download cudnn, you must have an nvidia account. 
# you want cuda9.2 and cudnn7.6 (it will also specify for 9.2)
read -p "Please download cuda 9.2 and cudnn7.6 (the one that works with 9.2). (y to continue) " -n 1 -r
echo    # (optional) move to a new line
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
	exit 1;
fi



# next begin installing packages 
sudo apt-get update -y
sudo apt-get install -y build-essential cmake git unzip pkg-config checkinstall yasm unzip
sudo apt-get install -y python2.7-dev python3-dev
sudo apt-get install -y libjpeg-dev libjpeg8-dev libtiff5 libtiff5-dev libjasper1 libjasper-dev libpng12-dev libpng++-dev
sudo apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libv4l-dev v4l-utils libavresample-dev libavresample3 libavresample-ffmpeg2
sudo apt-get install -y libxvidcore-dev libx264-dev x264 libxine2-dev libxine2 libdc1394-22-dev libdc1394-22 ffmpeg
sudo apt-get install -y libhdf5-serial-dev libhdf5-dev graphviz graphviz-dev 
sudo apt-get install -y libopenblas-base libopenblas-dev libatlas3-base libatlas-base-dev libatlas-dev gfortran 
sudo apt-get install -y liblapack3 liblapack-dev liblapacke liblapacke-dev
sudo apt-get install -y libgtk-3-dev libgtk2.0-dev libtbb-dev libfaac-dev libmp3lame-dev libtheora-dev 
sudo apt-get install -y qt5-default libqt5opengl5-dev 
sudo apt-get install -y protobuf-compiler libprotobuf-dev libprotoc-dev libprotozero-dev libprotobuf-c-dev 
sudo apt-get install -y linux-source linux-headers-generic 
sudo apt-get install -y libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
sudo apt-get install -y libvorbis-dev libopencore-amrnb-dev libopencore-amrwb-dev
sudo apt-get install -y libgoogle-glog-dev libgflags-dev
sudo apt-get install -y libgphoto2-dev libeigen3-dev doxygen
sudo apt-get install -y openjdk-8-jdk openjdk-8-jre 

# now install cuda
# NOTE, select no when it asks you to install the driver, we already did this!
# NOTE, generally select yes to all other prompts

### user input to check if cuda is done

read -p "verify the files 'cuda_9.2.148_396.37_linux.run' and 'cudnn-9.2-linux-x64-v7.6.4.38.tgz' are in the Downloads folder." -n 1 -r
echo    # (optional) move to a new line
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
	exit 1;
fi


cd ~/Downloads
chmod +x cuda_9.2.148_396.37_linux.run
sudo ./cuda_9.2.148_396.37_linux.run
printf '\n\n ###   autogenerated from cuda installation' >> ~/.bashrc
printf '\nexport PATH=/usr/local/cuda-9.2/bin:$PATH' >> ~/.bashrc
printf '\nexport LD_LIBRARY_PATH=/usr/local/cuda-9.2/lib64' >> ~/.bashrc
source ~/.bashrc
cd /usr/local/cuda-9.2/samples/1_Utilities/deviceQuery
sudo make
./deviceQuery


read -p "Results == pass? (y/n): " -n 1 -r
echo    # (optional) move to a new line
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
	echo "consult one of the many install guides on google."; exit 1;
fi


##### get user input here #####
cd ~/Downloads
tar -zxf cudnn-9.2-linux-x64-v7.6.4.38.tgz
cd cuda/
sudo cp -P lib64/* /usr/local/cuda/lib64/
sudo cp -P include/* /usr/local/cuda/include/


# anaconda should be done by now
read -p "Ensure anaconda has finished downloading before continuing (y): " -n 1 -r
echo    # (optional) move to a new line
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
	exit 1;
fi

#NOTE generally select y to all default options
cd ~/Downloads
chmod +x Anaconda3-2019.10-Linux-x86_64.sh 
./Anaconda3-2019.10-Linux-x86_64.sh 



#install sublime, will need to open some files for editing during installation
read -p "Do you already have sublime? (y/n): " -n 1 -r
echo    # (optional) move to a new line
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
	wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
	echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
	sudo apt-get update -y
	sudo apt-get install -y sublime-text
fi

# next install opencv3.2, 3.4 seems to be problematic with ros for some reason

#### user input to check if opencv is done




# sometimes the installations don't link or install in the directory we want
sudo ln -s /usr/include/lapacke.h /usr/include/x86_64-linux-gnu
sudo cp /usr/lib/x86_64-linux-gnu/libopenblas.so.0 /usr/lib/libopenblas.so.0
sudo cp /usr/lib/x86_64-linux-gnu/libopenblas.so /usr/lib/libopenblas.so
# to verify
# ldconfig -p | grep libopenblas

# fix openblas cmake


# now install ros
read -p "do you already have ros-kinetic? (y/n): " -n 1 -r
echo    # (optional) move to a new line
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
	sudo apt-get update
	sudo apt-get install -y ros-kinetic-desktop-full
fi
















