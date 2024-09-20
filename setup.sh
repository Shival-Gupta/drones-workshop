#!/bin/bash

# 2. Update apt repositories and upgrade packages
sudo add-apt-repository universe multiverse restricted && \
sudo apt update && \
sudo apt upgrade -y

# 3. Install core development tools and libraries (combined into single command)
sudo apt install -y git \
                    python3 python3-pip python3-dev \
                    nvidia-cuda-toolkit \
                    build-essential cmake g++ gdb \
                    libeigen3-dev libopencv-dev \
                    libyaml-cpp-dev python3-yaml \
                    libboost-all-dev libcurl4-openssl-dev \
                    libxml2-dev libbz2-dev

# 4. Install ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
sudo apt install -y curl && \
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
sudo apt update && \
sudo apt install -y ros-noetic-desktop-full 

# 5. Source ROS setup, install tools, and initialize rosdep (combined)
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# 6. Add environment variables to .bashrc
echo "export GAZEBO_IP=127.0.0.1" >> ~/.bashrc
echo "export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0" >> ~/.bashrc
echo "export LIBGL_ALWAYS_INDIRECT=0" >> ~/.bashrc

source ~/.bashrc && \
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential && \
sudo rosdep init && \
rosdep update

# 7. Install PX4 SITL 
git clone https://github.com/PX4/PX4-Autopilot.git && \
cd PX4-Autopilot && \
bash ./Tools/setup/ubuntu.sh # Consider adding `-y` or other flags if the script supports it
make px4_sitl_default gazebo # or make px4_sitl_default gazebo [model_name]
