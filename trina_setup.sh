#!/bin/bash

versionstring=$(cat /etc/issue)
echo 'printing this'
echo $versionstring
if echo "$versionstring" | grep -q "18"; then
	echo "ubuntu 18 detected - proceeding with ubuntu 18 installation"
	version=18

else
	if echo "$versionstring" | grep -q "16"; then
		echo "ubuntu 16 detected, proceeding with ubuntu 16 installation"
		version=16
	else
		echo "invalid ubuntu version for this installer - please refer to the TRINA
		advanced installation manual for additional instructions"
		sleep 5s
		exit "Invalid Ubuntu for autoinstall"
	fi

fi

echo "installing basic programs and libraries"
sudo apt-get install git
sudo apt-get install vim
sudo apt-get install python-pip
sudo apt-get install python3-pip
sudo apt-get install python-setuptools
sudo apt-get install python3-setuptools
cd


a=18
# installing ros for the appropriate version of Ubuntu

if [[ "$version" = "$a" ]]; then
	echo "instaling ROS Melodic"
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
	sudo apt update
	sudo apt install ros-melodic-desktop-full
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
	source ~/.bashrc
	sudo apt install python-rosdep python-rosinstall
	pip install -U rosdep rosinstall_generator wstool rosinstall
	sudo apt install build-essential
	sudo rosdep init
	sudo apt install ros-melodic-slam-gmapping
	rosdep updatepip2 install unidecode PyOpenGL rejson==0.3.0
	pip3 install --user pyqt5
	echo "installing qt5 libraries"
	sudo apt-get install python3-pyqt5
	sudo apt-get install pyqt5-dev-tools
	sudo apt-get install qttools5-dev-tools
	source ~/.bashrc

else
	echo "installing ROS Kinetic"
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
	sudo apt-get update
	sudo apt-get install ros-kinetic-desktop-full
	sudo apt-get install ros-kinetic-slam-gmapping
	echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
	source ~/.bashrc
	echo "installing qt4 and other ros libraries"
	sudo apt-get install python-qt4
	sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
	sudo rosdep init
	rosdep update


	echo "installing python 3.6 since your system python is not compatible with some of the libraries"
	sudo apt-get install software-properties-common
	sudo add-apt-repository ppa:deadsnakes/ppa
	sudo apt-get update
	sudo apt-get install python3.6
	pip3 install virtualenv
	virtualenv --python=/usr/bin/python3.6 ~/trina_env/ 
	source ~/trina_env/bin/activate

	echo "alias trina_env=\"source ~/trina_env/bin/activate\"" >> ~/.bashrc
	source ~/.bashrc




fi

echo "installing klampt from source"

echo "installing basic libraries first"

sudo apt-get install g++ cmake git libglpk-dev python-dev python-opengl libxmu-dev libxi-dev qt5-default

sudo apt-get install libassimp-dev

echo "cloning klampt repository"

cd

git clone https://github.com/krishauser/Klampt

cd Klampt/Cpp/Dependencies; make unpack-deps; make deps

cd ../../

cmake .

make Klampt

make apps

make python

sudo make python-install

# Eventually when we migrate to python3 we need to install klampt from source 
# on the python3 virtual env
sudo make python3-install

sudo apt-get install ffmpeg


cd

echo "installing crucial trina libraries"

cd TRINA/Resources/UR5e_Control_API/PyUniversalRobot-master
sudo python2 setupPy2.py install & sudo python3 setup.py install

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

echo "installing reem from source"
cd
git clone https://github.com/krishauser/reem
cd reem
sudo python2 setup.py install
sudo python3 setup.py install --

source ~/.bashrc
echo "installing necessary python libraries"

python2 -m pip install "setuptools<45"

if [[ "$version" = "$a" ]]; then
	pip3 install --user unidecode scipy numpy pandas rejson redis PyOpenGL open3d jupyter jupyter-contrib-nbextensions
	
	pip2 install unidecode PyOpenGL rejson==0.3.0 future --no-binary :all:

	cd ~/

	pip2 install --user scipy==1.2.2 --no-dependencies

	pip2 install open3d jupyter

	cd TRINA/robot_v2/websocket_client-0.56.0

	sudo python3 setup.py install && sudo python2 setup.py install

else

	sudo apt-get install software-properties-common
	sudo add-apt-repository ppa:ubuntu-toolchain-r/test
	sudo apt-get update
	sudo apt-get install gcc-4.9
	sudo apt-get upgrade libstdc++6
	pip3 install unidecode scipy numpy pandas rejson redis PyOpenGL open3d jupyter jupyter-contrib-nbextensions
	
	pip2 install unidecode PyOpenGL rejson==0.3.0 future --no-binary :all:

	cd ~/
	# installing klampt from pip for now
	pip3 install klampt
	pip2 install --user scipy==1.2.2 --no-dependencies

	pip2 install open3d jupyter

	cd TRINA/robot_v2/websocket_client-0.56.0

	sudo ~/trina_env/bin/python3 setup.py install && sudo python2 setup.py install

fi

pip2 install pyarrow opencv-python==4.1.2.30
pip install pyarrow opencv-python

cd ~/TRINA
