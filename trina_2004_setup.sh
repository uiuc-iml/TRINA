#!/bin/bash

echo "Setting up TRINA on Ubuntu 20.04"

if [ "$0" = "$BASH_SOURCE" ]; then
    echo "Error: Script must be sourced"
    exit 1
fi

if [ "$EUID" -eq 0 ]
  then echo "Error: Script should not run in sudo"
  exit
fi

sudo apt install git
sudo apt install python3-pip
pip3 install pyserial open3d trimesh

echo "Instaling ROS Noetic"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "Instaling SLAM package"
sudo apt install -y ros-noetic-slam-gmapping

echo "Creating workspace for other ROS packages"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
cd ..
catkin_make
# echo "source /home/motion/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
cd

echo "Installing Klamp't from source"
sudo apt install -y g++ cmake git libglpk-dev python3-dev libxmu-dev libxi-dev qt5-default libassimp-dev ffmpeg
git clone https://github.com/krishauser/Klampt
cd Klampt/Cpp/Dependencies; make unpack-deps; make deps
cd ../../
cmake .
make Klampt
make apps
make python
sudo make python-install
cd

echo "installing crucial trina libraries"
cd TRINA/Resources/UR5e_Control_API/PyUniversalRobot-master
sudo python3 setup.py install
cd

echo "installing redis and setting up the databases"
mkdir trina-database-server
cd trina-database-server
wget http://download.redis.io/releases/redis-5.0.4.tar.gz
tar xzf redis-5.0.4.tar.gz
cd redis-5.0.4/deps
make hiredis lua jemalloc linenoise
cd ..
make
alias redis-server=$PWD/src/redis-server
alias redis-cli=$PWD/src/redis-cli
cd ..
echo "installing dependencies necessary for compiling redisjson"
sudo apt-get install cargo libclang-dev==8.0

cd ~/trina-database-server
echo "installing redisjson"
git clone https://github.com/RedisLabsModules/redisjson.git
cd redisjson
git checkout 1.0
make
cd ..

echo "cloning configuration file from TRINA"
cp ~/TRINA/redis.conf ~/trina-database-server/redis.conf
cd

# echo "Installing reem from source"
# git clone https://github.com/krishauser/reem
# cd reem
# sudo python3 setup.py install
# cd

source ~/.bashrc
echo "installing necessary python libraries"
pip install unidecode scipy numpy pandas rejson redis PyOpenGL open3d jupyter jupyter-contrib-nbextensions h5py
pip install reem

# echo "Install pyrealsense2 from source"
# git clone https://github.com/achirkin/librealsense/
# cd librealsense
# mkdir build && cd build
# cmake ../ -DFORCE_RSUSB_BACKEND=true -DBUILD_PYTHON_BINDINGS=true -DCMAKE_BUILD_TYPE=release
# make
# sudo make install
# ./scripts/setup_udev_rules.sh
# echo "export PYTHONPATH=$PYTHONPATH:/usr/local/lib" >> ~/.bashrc
# source ~/.bashrc

echo "Installing CUDA"
sudo wget -O /etc/apt/preferences.d/cuda-repository-pin-600 https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/7fa2af80.pub
sudo add-apt-repository "deb http://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /"
sudo apt install cuda
echo 'export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}' >> ~/.bashrc
cd

echo "Installing ZED for sensor module"
wget https://download.stereolabs.com/zedsdk/3.3/cu111/ubuntu20
chmod +x ubuntu20
./ubuntu20
# wget https://download.stereolabs.com/zedsdk/3.3/ubuntu20/cu111/py38
# mv py38 pyzed-3.3-cp38-cp38-linux_x86_64.whl
# pip3 install pyzed-3.3-cp38-cp38-linux_x86_64.whl

cd ~/TRINA/robot_v2/websocket_client-0.56.0
sudo python3 setup.py install

cd ~/TRINA

