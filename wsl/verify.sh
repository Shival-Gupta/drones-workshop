#!/bin/bash

# Function to check installed packages
check_package() {
    dpkg -l | grep -q $1
    if [ $? -eq 0 ]; then
        echo "$1 is installed."
    else
        echo "$1 is NOT installed."
    fi
}

# Check Ubuntu version
echo "Checking Ubuntu version..."
ubuntu_version=$(lsb_release -rs)
if [ "$ubuntu_version" == "20.04" ]; then
    echo "Ubuntu version is 20.04."
else
    echo "This script is intended for Ubuntu 20.04, found version: $ubuntu_version."
fi

# Check for essential packages
echo "Checking essential packages..."
packages=("git" "python3" "python3-pip" "ros-noetic-desktop-full" "gazebo11" "mavros")
for pkg in "${packages[@]}"; do
    check_package $pkg
done

# Check ROS environment
if [ -f ~/catkin_ws1/devel/setup.bash ]; then
    echo "ROS workspace environment is set up."
else
    echo "ROS workspace environment is NOT set up."
fi

# Check if Gazebo is running
if pgrep -x "gazebo" > /dev/null; then
    echo "Gazebo is running."
else
    echo "Gazebo is NOT running."
fi

# Check for QGroundControl
if [ -f "./QGroundControl.AppImage" ]; then
    echo "QGroundControl is available."
else
    echo "QGroundControl is NOT available."
fi

echo "Verification complete!"
