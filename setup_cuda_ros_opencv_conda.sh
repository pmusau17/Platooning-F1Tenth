#!/bin/bash


### NOTE! READ THIS FIRST ABOUT NVIDIA DRIVERS
<<'NVIDIA-DRIVERS'

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

BASE_DIR=$(pwd)

read -p "have you verified the configuration of your nvidia drivers?" -n 1 -r 
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
	echo 'read the notes at the top of this script and confirm your configuraiton'
	exit 1;
fi

echo
echo 'ensure all downloads necessary for this script are placed in the ~/Downloads folder'
echo 'press any key to continue.'
read KEY

# first get anaconda downloading in the background
# valid as of Dec 12 2019
read -p "Do you already have anaconda downloaded? (y/n) " -n 1 -r
echo    
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
	cd ~/Downloads
    nohup wget -O anaconda_installer.sh https://repo.anaconda.com/archive/Anaconda3-2019.10-Linux-x86_64.sh > /dev/null 2>&1 &
fi


# get opencv downloads going too
read -p "Do you already have opencv3.4 downloaded? (y/n) " -n 1 -r
echo  
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
	cd ~/Downloads
    nohup wget -O opencv.zip https://github.com/Itseez/opencv/archive/3.4.4.zip & > /dev/null 2>&1 &
    nohup wget -O opencv_contrib.zip https://github.com/Itseez/opencv_contrib/archive/3.4.4.zip & > /dev/null 2>&1 &
fi

# get cuda downloads going. Note, to download cudnn, you must have an nvidia account. 
# you want cuda9.2 and cudnn7.6 (it will also specify for 9.2)
read -p "Have you already downloaded cuda9.2 run file and the patch? (y/n): " -n 1 -r
echo  
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
	nohup wget -O cuda9.2.run https://developer.nvidia.com/compute/cuda/9.2/Prod2/local_installers/cuda_9.2.148_396.37_linux & > /dev/null 2>&1 &
	nohup wget -O cuda9.2.patch.run https://developer.nvidia.com/compute/cuda/9.2/Prod2/patches/1/cuda_9.2.148.1_linux & > /dev/null 2>&1 &
fi

echo 'please go to developer.nvidia.com/rdp/cudnn-download and download cudnn7.6.5(9.2) "cuDNN Library for Linux" to the ~/Downloads folder.'
echo 'press any key to continue.'
read KEY



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

read -p "verify the files 'cuda_9.2.148_396.37_linux.run', 'cuda_9.2.148.1_linux.run', and 'cudnn-9.2-linux-x64-v7.6.4.38.tgz' are in the Downloads folder." -n 1 -r
echo   
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
	exit 1;
fi

read -p "select no when the installer asks to install the driver. (y to acknowledge and continue)" -n 1 -r
echo   
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
	exit 1;
fi


chmod +x cuda9.2.run
sudo ./cuda9.2.run
printf '\n\n ###   autogenerated from cuda installation' >> ~/.bashrc
printf '\nexport PATH=/usr/local/cuda-9.2/bin:$PATH' >> ~/.bashrc
printf '\nexport LD_LIBRARY_PATH=/usr/local/cuda-9.2/lib64' >> ~/.bashrc
source ~/.bashrc
chmod +x cuda_9.2.patch.run 
sudo ./cuda_9.2.patch.run
source ~/.bashrc
cd /usr/local/cuda-9.2/samples/1_Utilities/deviceQuery
sudo make
./deviceQuery


read -p "Results == pass? (y/n): " -n 1 -r
echo   
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
	echo "consult one of the many install guides on google."; 
	cd $BASE_DIR
	exit 1;
fi

echo "enter the filename for the cudnn-9.2 linux download (they update versions): "
read CUDNN_FILENAME

cd ~/Downloads
tar -zxf $CUDNN_FILENAME
cd cuda/
sudo cp -P lib64/* /usr/local/cuda/lib64/
sudo cp -P include/* /usr/local/cuda/include/


# anaconda should be done by now
read -p "Ensure anaconda has finished downloading before continuing (y): " -n 1 -r
echo 
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
	exit 1;
fi

cd ~/Downloads
chmod +x Anaconda3-2019.10-Linux-x86_64.sh 
./Anaconda3-2019.10-Linux-x86_64.sh 
source ~/anaconda3/etc/profile.d/conda.sh
cd $BASE_DIR
echo 'enter the name for your ros/python2.7 anaconda environment: '
read ROS_ENV_NAME
conda env create -n $ROS_ENV_NAME -f environment.yml python=2.7


echo 'enter the name for your deep learning / python3.6 anaconda environment: '
read DL_ENV_NAME
echo 'we will reinstall python=3.6 after the environment is created'
echo 'we use 3.6 because of the cuda+tf+keras+pytorch stack'
conda env create -n $DL_ENV_NAME -f dl36.yml 
conda activate $DL_ENV_NAME
conda install python=3.6
conda deactivate
conda deactivate


read -p "Do you already have sublime? (y/n): " -n 1 -r
echo 
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
	cd ~/Downloads
	wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
	echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
	sudo apt-get update -y
	sudo apt-get install -y sublime-text
	cd $BASE_DIR
fi


read -p "Ensure opencv.zip and opencv_contrib.zip have finished downloading before continuing (y): " -n 1 -r
echo  
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
	exit 1;
fi
cd ~/Downloads
unzip opencv.zip
unzip opencv_contrib.zip 
mv opencv-3.4.4/ opencv
mv opencv_contrib-3.4.4/ opencv_contrib
# sometimes the installations don't link or install in the directory we want
FILE=/usr/include/x86_64-linux-gnu/lapacke.h
if [[ -f "$FILE" ]]; then
	echo 'lapack check yes.'
else
	sudo ln -s /usr/include/lapacke.h /usr/include/x86_64-linux-gnu
fi
FILE=/usr/lib/libopenblas.so
if [[ -f "$FILE" ]]; then
	echo 'openblas libraries good to go.'
else
	FILE=/usr/lib/x86_64-linux-gnu/libopenblas.so
	if [[ -f "$FILE" ]]; then
		sudo cp /usr/lib/x86_64-linux-gnu/libopenblas.so.0 /usr/lib/libopenblas.so.0
		sudo cp /usr/lib/x86_64-linux-gnu/libopenblas.so /usr/lib/libopenblas.so
	else
		echo 'libopenblas did not install correctly. consult google.'
		exit 1;
	fi
fi
sudo cp /usr/include/lapacke* /usr/include/openblas/
# to verify
# ldconfig -p | grep libopenblas

PY2_PATH=/home/$USER/anaconda3/envs/$ROS_ENV_NAME
PY3_PATH=/home/$USER/anaconda3/envs/$DL_ENV_NAME



cd opencv
mkdir build 
cd build

cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_CUDA=ON -D INSTALL_PYTHON_EXAMPLES=ON \
-D OPENCV_EXTRA_MODULES_PATH=~/Downloads/opencv_contrib/modules -D OPENCV_ENABLE_NONFREE=ON \
-D WITH_TBB=ON -D WITH_V4L=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D BUILD_EXAMPLES=ON -D WITH_CUBLAS=ON -D CUDA_FAST_MATH=1 \
-D PYTHON3_EXECUTABLE=$PY3_PATH/bin/python3 -D PYTHON3_PACKAGES_PATH=$PY3_PATH/lib/python3.6/site-packages \
-D PYTHON2_EXECUTABLE=$PY2_PATH/bin/python -D PYTHON2_PACKAGES_PATH=$PY2_PATH/lib/python2.7/site-packages \
-D PYTHON2_INCLUDE_DIR=$PY2_PATH/include -D PYTHON3_INCLUDE_DIR=$PY3_PATH/include \
-D PYTHON2_NUMPY_INCLUDE_DIRS=$PY2_PATH/lib/python2.7/site-packages/numpy/core/include \
-D PYTHON3_NUMPY_INCLUDE_DIRS=$PY3_PATH/lib/python3.6/site-packages/numpy/core/include \
-D PYTHON2_LIBRARY=$PY2_PATH/lib/libpython2.7.so -D PYTHON3_LIBRARY=$PY3_PATH/lib/libpython3.6m.so \
-D PYTHON_DEFAULT_EXECUTABLE=$PY3_PATH/bin/python3 -D ENABLE_FAST_MATH=1 -D BUILD_SHARED_LIBS=ON ..

make -j$(expr $(nproc) - 2)
sudo make install
sudo ldconfig

sudo cp /usr/local/python/cv2/python-2.7/cv2.so $PY2_PATH/lib/python2.7/cv2.so
sudo cp /usr/local/python/cv2/python-2.7/cv2.so ~/py27_cv2_backup.so
sudo cp /usr/local/python/cv2/python-3.6/cv2.cpython-36m-x86_64-linux-gnu.so $PY3_PATH/lib/python3.6/cv2.so
sudo cp /usr/local/python/cv2/python-3.6/cv2.cpython-36m-x86_64-linux-gnu.so ~/py36_cv2_backup.so

echo 'opencv backups for python2.7 and python3.6 have been placed in your home folder'
echo 'press any key to continue'
read KEY

cd $BASE_DIR
echo 'lets test our opencv builds... (any key to continue): '
read KEY
conda activate $ROS_ENV_NAME
python tmp_build_files/test_cv_py2.py
conda deactivate
conda activate $DL_ENV_NAME
python tmp_build_files/test_cv_py3.py
conda deactivate

read -p "did both tests print out <CV2-VERSION>?" -n 1 -r
echo   
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
	echo 'hmmm, something went wrong.';
	exit 1;
fi

#rm -rf tmp_build_files

read -p "would you like to remove the files associated with the opencv install?" -n 1 -r
echo   
if [[ ! $REPLY =~ ^[Nn]$ ]]
then
	cd ~/Downloads
	rm opencv.zip
	rm opencv_contrib.zip
	rm -rf opencv
	rm -rf opencv_contrib
fi

#sudo mv /usr/local/python/cv2/python-3.6/cv2.cpython-36m-x86_64-linux-gnu.so cv2.so

# now install ros
read -p "do you already have ros-kinetic? (y/n): " -n 1 -r
echo   
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
	conda activate $ROS_ENV_NAME
	sudo apt-get install -y python-rosinstall python-rosinstall-generator python-wstool
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
	sudo apt-get update
	sudo apt-get install -y ros-kinetic-desktop-full
	printf '\n\n ###   autogenerated from ros installation' >> ~/.bashrc
	printf '\nsource /opt/ros/kinetic/setup.bash' >> ~/.bashrc
	source ~/.bashrc
fi
















