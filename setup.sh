#!/bin/bash

# Function to log start and end of steps
log_step() {
    echo -e "\n========================================"
    echo "Step $1: $2"
    echo "========================================"
}

# Step 1: Add necessary apt repositories
log_step "1" "Adding apt repositories"
sudo apt-add-repository universe -y && \
sudo apt-add-repository multiverse -y && \
sudo apt-add-repository restricted -y
if [ $? -eq 0 ]; then
    echo "Step 1 completed successfully!"
else
    echo "Error in Step 1. Exiting..."
    exit 1
fi

# Step 2: Update repositories and upgrade packages
log_step "2" "Updating repositories and upgrading packages"
sudo apt update && sudo apt upgrade -y
if [ $? -eq 0 ]; then
    echo "Step 2 completed successfully!"
else
    echo "Error in Step 2. Exiting..."
    exit 1
fi

# Step 3: Install core development tools and libraries
log_step "3" "Installing core development tools and libraries"
sudo apt install -y git \
                    python3 python3-pip python3-dev \
                    build-essential cmake g++ gdb \
                    libeigen3-dev libopencv-dev \
                    libyaml-cpp-dev python3-yaml \
                    libboost-all-dev libcurl4-openssl-dev \
                    libxml2-dev libbz2-dev
if [ $? -eq 0 ]; then
    echo "Step 3 completed successfully!"
else
    echo "Error in Step 3. Exiting..."
    exit 1
fi

# Step 4: Set up ROS Noetic repository and install ROS
log_step "4" "Setting up ROS Noetic repository and installing ROS"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
sudo apt install -y curl && \
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
sudo apt update && \
sudo apt install -y ros-noetic-desktop-full
if [ $? -eq 0 ]; then
    echo "Step 4 completed successfully!"
else
    echo "Error in Step 4. Exiting..."
    exit 1
fi

# Step 5: Initialize rosdep and install ROS tools
log_step "5" "Initializing rosdep and installing ROS tools"
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential && \
sudo rosdep init && \
rosdep update
if [ $? -eq 0 ]; then
    echo "Step 5 completed successfully!"
else
    echo "Error in Step 5. Exiting..."
    exit 1
fi

# Step 6: Install PX4 SITL
log_step "6" "Installing PX4 SITL"
cd ~ && \
git clone https://github.com/PX4/PX4-Autopilot.git && \
cd PX4-Autopilot && \
bash ./Tools/setup/ubuntu.sh -y && \
make px4_sitl_default gazebo
if [ $? -eq 0 ]; then
    echo "Step 6 completed successfully!"
else
    echo "Error in Step 6. Exiting..."
    exit 1
fi

# Step 7: Install Ardupilot SITL and dependencies
log_step "7" "Installing Ardupilot SITL and dependencies"
cd ~ && \
git clone https://github.com/ArduPilot/ardupilot.git && \
cd ardupilot && \
git checkout Copter-4.0.4 && \
git submodule update --init --recursive || git config --global url.https://.insteadOf git:// && \
Tools/environment_install/install-prereqs-ubuntu.sh -y && \
source ~/.profile && \
cd ~/ardupilot/ArduCopter && \
sim_vehicle.py -w # Initialize SITL with default configuration
if [ $? -eq 0 ]; then
    echo "Step 7 completed successfully!"
else
    echo "Error in Step 7. Exiting..."
    exit 1
fi

# Step 8: Setup Gazebo and Ardupilot-Gazebo Plugin
log_step "8" "Setting up Gazebo and Ardupilot-Gazebo Plugin"
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
sudo apt update && \
sudo apt-get install -y gazebo11 libgazebo11-dev && \
cd ~ && \
git clone https://github.com/khancyr/ardupilot_gazebo.git && \
cd ardupilot_gazebo && \
mkdir build && cd build && \
cmake .. && \
make -j4 && \
sudo make install && \
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc && \
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
source ~/.bashrc
if [ $? -eq 0 ]; then
    echo "Step 8 completed successfully!"
else
    echo "Error in Step 8. Exiting..."
    exit 1
fi

# Step 9: Install MAVROS, MAVLink, and IQ Sim
log_step "9" "Installing MAVROS, MAVLink, and IQ Sim"
cd ~ && \
mkdir -p ~/catkin_ws1/src && \
cd ~/catkin_ws1 && \
catkin init && \
wstool init ~/catkin_ws1/src && \
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall && \
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall && \
wstool merge -t src /tmp/mavros.rosinstall && \
wstool update -t src && \
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y && \
catkin build && \
echo "source ~/catkin_ws1/devel/setup.bash" >> ~/.bashrc && \
source ~/.bashrc && \
sudo ~/catkin_ws1/src/mavros/mavros/scripts/install_geographiclib_datasets.sh && \
cd ~/catkin_ws1/src && \
git clone https://github.com/Intelligent-Quads/iq_sim.git && \
git clone https://github.com/Intelligent-Quads/iq_gnc.git && \
catkin build
if [ $? -eq 0 ]; then
    echo "Step 9 completed successfully!"
else
    echo "Error in Step 9. Exiting..."
    exit 1
fi

# Step 10: Install QGroundControl
log_step "10" "Installing QGroundControl"
cd ~ && \
sudo usermod -a -G dialout $USER && \
sudo apt-get remove modemmanager -y && \
wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage && \
chmod +x ./QGroundControl.AppImage
if [ $? -eq 0 ]; then
    echo "Step 10 completed successfully!"
    echo "Setup complete. To run QGroundControl, execute ./QGroundControl.AppImage."
else
    echo "Error in Step 10. Exiting..."
    exit 1
fi

echo "========================================"
echo "All steps completed successfully!"
echo "========================================"
