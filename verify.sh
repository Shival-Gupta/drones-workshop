#!/bin/bash

# Function to check the installation of a package
check_package() {
    if dpkg -l | grep -q "$1"; then
        echo "$1 is installed."
    else
        echo "$1 is not installed."
    fi
}

# Log file
LOG_FILE="installation_verification.log"

# Clear previous log
echo "Verification Log - $(date)" > "$LOG_FILE"

# Check for essential packages
echo "Checking essential packages..." | tee -a "$LOG_FILE"
check_package "ros-noetic-desktop-full" | tee -a "$LOG_FILE"
check_package "gazebo11" | tee -a "$LOG_FILE"
check_package "python3-pip" | tee -a "$LOG_FILE"
check_package "mavros" | tee -a "$LOG_FILE"

# Check ROS environment
if [ -f ~/catkin_ws1/devel/setup.bash ]; then
    echo "ROS workspace is set up." | tee -a "$LOG_FILE"
else
    echo "ROS workspace is not set up." | tee -a "$LOG_FILE"
fi

# Check Ubuntu version
UBUNTU_VERSION=$(lsb_release -rs)
echo "Ubuntu version: $UBUNTU_VERSION" | tee -a "$LOG_FILE"

# Summary
echo "Verification completed. Check $LOG_FILE for details."
