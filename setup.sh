#!/bin/bash

# 1. Set WSL default version and install Ubuntu (if running on WSL)
# Uncomment the following two lines for WSL setup if required:
# wsl --set-default-version 2
# wsl --install -d Ubuntu-20.04

# 2. Update apt repositories and upgrade packages
sudo add-apt-repository universe multiverse restricted -y && \
sudo apt update && sudo apt upgrade -y

# 3. Install core development tools and libraries
sudo apt install -y git \
                    python3 python3-pip python3-dev \
                    nvidia-cuda-toolkit \
                    build-essential cmake g++ gdb \
                    libeigen3-dev libopencv-dev \
                    libyaml-cpp-dev python3-yaml \
                    libboost-all-dev libcurl4-openssl-dev \
                    libxml2-dev libbz2-dev \
                    curl python3-rosdep python3-rosinstall \
                    python3-rosinstall-generator python3-wstool \
                    ros-noetic-desktop-full

# 4. Set up ROS Noetic and environment variables
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "export GAZEBO_IP=127.0.0.1" >> ~/.bashrc
echo "export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0" >> ~/.bashrc
echo "export LIBGL_ALWAYS_INDIRECT=0" >> ~/.bashrc
source ~/.bashrc

sudo rosdep init && \
rosdep update

# 5. Install PX4 SITL
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh -y
make px4_sitl_default gazebo

# 6. Install Ardupilot SITL and dependencies
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git checkout Copter-4.0.4
git submodule update --init --recursive || git config --global url.https://.insteadOf git://
Tools/environment_install/install-prereqs-ubuntu.sh -y
source ~/.profile
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w # Initialize SITL with default configuration

# 7. Setup Gazebo and Ardupilot-Gazebo Plugin
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt-get install -y gazebo11 libgazebo11-dev

cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build && cd build
cmake ..
make -j4
sudo make install
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
source ~/.bashrc

# 8. Install MAVROS, MAVLink, and IQ Sim
cd ~
mkdir -p ~/catkin_ws1/src
cd ~/catkin_ws1
catkin init
wstool init ~/catkin_ws1/src
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y
catkin build
echo "source ~/catkin_ws1/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo ~/catkin_ws1/src/mavros/mavros/scripts/install_geographiclib_datasets.sh

# 9. Clone and build IQ Sim and GNC
cd ~/catkin_ws1/src
git clone https://github.com/Intelligent-Quads/iq_sim.git
git clone https://github.com/Intelligent-Quads/iq_gnc.git
catkin build
echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws1/src/iq_sim/models" >> ~/.bashrc
source ~/.bashrc

# 10. Install QGroundControl
cd ~
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage
chmod +x ./QGroundControl.AppImage

echo "Setup complete. To run QGroundControl, execute ./QGroundControl.AppImage."
echo "To run ArduCopter SITL with Gazebo, run the commands in the README."
