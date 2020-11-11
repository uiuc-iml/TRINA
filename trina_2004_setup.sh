#!/bin/bash

echo "Fresh install on ubuntu 20"
if [ "$EUID" -eq 0 ]
  then echo "You should not run this script as root, or cd command will fail"
  exit
fi
sudo apt install git
sudo apt install python3-pip
cd

echo "instaling ROS Noetic"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "instaling other ros libraries"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/ros-perception/openslam_gmapping
git clone https://github.com/ros-perception/slam_gmapping
cd ~/catkin_ws
catkin_make
echo "source /home/motion/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
cd

echo "installing klampt from source"
echo "installing basic libraries first"
sudo apt-get install build-essential libglpk-dev libxi-dev qt5-default
sudo apt-get install libassimp-dev
echo "cloning klampt repository"
git clone https://github.com/krishauser/Klampt
cd Klampt/Cpp/Dependencies; make unpack-deps; make deps
cd ../../
cmake .
make Klampt
make apps
make python3
sudo make python3-install
sudo apt-get install ffmpeg
cd

echo "installing crucial trina libraries"
cd TRINA/Resources/UR5e_Control_API/PyUniversalRobot-master
sudo python3 setup.py install
cd

echo "installing redis and setting up the databases"
mkdir database-server
cd database-server
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

cd ~/database-server
echo "installing redisjson"
git clone https://github.com/RedisLabsModules/redisjson.git
cd redisjson
git checkout 1.0
make
cd ..

echo "cloning configuration file from TRINA"
cp ~/TRINA/redis.conf ~/database-server/redis.conf

echo "installing reem from source"
cd
git clone https://github.com/krishauser/reem
cd reem
sudo python3 setup.py install

source ~/.bashrc
echo "installing necessary python libraries"
pip3 install --user unidecode scipy numpy pandas rejson redis PyOpenGL open3d jupyter jupyter-contrib-nbextensions h5py
# installing klampt from pip for now
pip3 install klampt

echo "install pyrealsense2 from source as no 3.8 binding existing yet"
cd
git clone https://github.com/achirkin/librealsense/
cd librealsense
./scripts/setup_udev_rules.sh
./scripts/patch-realsense-ubuntu-lts.sh
mkdir build
cd build
cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true -DENFORCE_METADATA:bool=true
make -j($nproc)
sudo make install
export PYTHONPATH=$PYTHONPATH:/usr/local/lib
echo "export PYTHONPATH=$PYTHONPATH:/usr/local/lib" >> ~/.bashrc

echo "installing cuda"
sudo wget -O /etc/apt/preferences.d/cuda-repository-pin-600 https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/7fa2af80.pub
sudo add-apt-repository "deb http://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /"
sudo apt install cuda
echo 'export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}' >> ~/.bashrc

cd
echo "installing ZED for sensor module"
wget https://download.stereolabs.com/zedsdk/3.3/cu111/ubuntu20
chmod +x ubuntu20
./ubuntu20
wget https://download.stereolabs.com/zedsdk/3.3/ubuntu20/cu111/py38
mv py38 pyzed-3.3-cp38-cp38-linux_x86_64.whl
python3 -m pip install pyzed-3.3-cp38-cp38-linux_x86_64.whl

cd
cd TRINA/robot_v2/websocket_client-0.56.0
sudo python3 setup.py install

pip3 install pyserial

cd ~/TRINA

