#!/bin/bash

# Log file location
LOG_FILE="verify.log"

# Function to log a step
log_step() {
    echo -e "\n========================================" | tee -a "$LOG_FILE"
    echo "Step $1: $2" | tee -a "$LOG_FILE"
    echo "========================================" | tee -a "$LOG_FILE"
}

# Function to check if a package is installed
check_package() {
    if dpkg -l | grep -q "$1"; then
        echo "$1 is installed." | tee -a "$LOG_FILE"
    else
        echo "Error: $1 is not installed!" | tee -a "$LOG_FILE"
        return 1
    fi
    return 0
}

# Function to check if a directory exists
check_directory() {
    if [ -d "$1" ]; then
        echo "$1 exists." | tee -a "$LOG_FILE"
    else
        echo "Error: $1 does not exist!" | tee -a "$LOG_FILE"
        return 1
    fi
    return 0
}

# Clear previous log and start verification
echo "Verification Log - $(date)" > "$LOG_FILE"
echo "Starting verification process..." | tee -a "$LOG_FILE"

# Step 1: Verify core development tools
log_step "1" "Verifying core development tools"
core_packages=(
    git
    python3
    python3-pip
    python3-dev
    build-essential
    cmake
    g++
    gdb
    libeigen3-dev
    libopencv-dev
    libyaml-cpp-dev
    python3-yaml
    libboost-all-dev
    libcurl4-openssl-dev
    libxml2-dev
    libbz2-dev
)

for package in "${core_packages[@]}"; do
    check_package "$package"
done

# Step 2: Verify ROS Noetic installation
log_step "2" "Verifying ROS Noetic installation"
check_package "ros-noetic-desktop-full"

# Step 3: Verify rosdep installation
log_step "3" "Verifying rosdep installation"
check_package "python3-rosdep"

# Step 4: Verify project installations
log_step "4" "Verifying PX4 and Ardupilot installations"
check_directory "$HOME/PX4-Autopilot"
check_directory "$HOME/ardupilot"

# Step 5: Verify Gazebo installation
log_step "5" "Verifying Gazebo installation"
check_package "gazebo11"
check_package "libgazebo11-dev"

# Step 6: Verify MAVROS installation
log_step "6" "Verifying MAVROS installation"
mavros_packages=(
    ros-noetic-mavros
    ros-noetic-mavros-extras
    ros-noetic-mavlink
)

for package in "${mavros_packages[@]}"; do
    check_package "$package"
done

# Step 7: Verify QGroundControl installation
log_step "7" "Verifying QGroundControl installation"
check_package "qgroundcontrol"

# Step 8: Check ROS workspace
log_step "8" "Checking ROS workspace"
if [ -d "$HOME/catkin_ws1" ]; then
    echo "ROS workspace is set up." | tee -a "$LOG_FILE"
else
    echo "Error: No valid ROS workspace found." | tee -a "$LOG_FILE"
fi

# Check Ubuntu version
UBUNTU_VERSION=$(lsb_release -rs)
echo "Ubuntu version: $UBUNTU_VERSION" | tee -a "$LOG_FILE"

# Summary
echo "Verification completed. Check $LOG_FILE for details." | tee -a "$LOG_FILE"
