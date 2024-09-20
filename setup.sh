#!/bin/bash

# Clear the log file
LOG_FILE="setup.log"
: > "$LOG_FILE"

# Exit immediately if a command exits with a non-zero status
set -e

# Function to log steps
log_step() {
    echo -e "\n========================================" | tee -a "$LOG_FILE"
    echo "Step $1: $2" | tee -a "$LOG_FILE"
    echo "========================================" | tee -a "$LOG_FILE"
}

# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error in Step $1. Exiting..." | tee -a "$LOG_FILE"
        exit 1
    fi
    echo "Step $1 completed successfully!" | tee -a "$LOG_FILE"
}

# Function to show usage
usage() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  --uninstall           Remove installed packages."
    echo "  --reinstall           Reinstall existing packages."
    echo "  --force-install       Install packages regardless of existing installations."
    echo "  --full-installation   Automatically install all tools without prompts."
    echo "  --wsl                 Special configurations for Windows Subsystem for Linux."
    echo "  --help                Show this help message."
    exit 1
}

# Determine action based on arguments
ACTION="install"
FORCE_INSTALL=""
FULL_INSTALL=""
WSL_CONFIG=""
for arg in "$@"; do
    case $arg in
        --uninstall) ACTION="uninstall" ;;
        --reinstall) ACTION="reinstall" ;;
        --force-install) FORCE_INSTALL="true" ;;
        --full-installation) FULL_INSTALL="true" ;;
        --wsl) WSL_CONFIG="true" ;;
        --help) usage ;;
        # *) echo "Unknown argument: $arg"; usage ;;  # Remove this line
    esac
done

# Check for incompatible arguments
if [[ "$ACTION" == "uninstall" && ("$FULL_INSTALL" == "true" || "$FORCE_INSTALL" == "true" || "$ACTION" == "reinstall") ]] ||
   [[ "$ACTION" == "reinstall" && ("$FULL_INSTALL" == "true" || "$ACTION" == "uninstall") ]] ||
   [[ "$FULL_INSTALL" == "true" && ("$ACTION" == "uninstall" || "$ACTION" == "reinstall") ]]; then
    echo "Error: Incompatible arguments provided."
    usage
fi

# Function to check if a package is installed
check_package() {
    dpkg -l | grep -q "$1"
}

# Function to install packages
install_packages() {
    log_step "$1" "$2"
    if [[ "$FORCE_INSTALL" == "true" || ! $(check_package "$3") ]]; then
        sudo apt install -y $3
        check_success "$1"
    else
        echo "$3 is already installed, skipping..." | tee -a "$LOG_FILE"
    fi
}

# Function to uninstall packages
uninstall_packages() {
    log_step "$1" "$2"
    sudo apt remove --purge -y $3
    check_success "$1"
}

# Function to reinstall packages
reinstall_packages() {
    log_step "$1" "$2"
    sudo apt install --reinstall -y $3
    check_success "$1"
}

# Handle actions based on arguments
if [ "$ACTION" == "uninstall" ]; then
    uninstall_packages "1" "Removing installed packages" "git python3 python3-pip python3-dev build-essential cmake g++ gdb libeigen3-dev libopencv-dev libyaml-cpp-dev python3-yaml libboost-all-dev libcurl4-openssl-dev libxml2-dev libbz2-dev ros-noetic-desktop-full python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool"
    echo "Uninstallation completed. Check the log file $LOG_FILE for details."
    exit 0
fi

# Step 1: Add necessary apt repositories
log_step "1" "Adding apt repositories"
sudo apt-add-repository -y universe
sudo apt-add-repository -y multiverse
sudo apt-add-repository -y restricted
check_success "1"

# Step 2: Update repositories and upgrade packages
log_step "2" "Updating repositories and upgrading packages"
sudo apt update && sudo apt upgrade -y
check_success "2"

# Step 3: Install core development tools and libraries
core_packages="git python3 python3-pip python3-dev build-essential cmake g++ gdb libeigen3-dev libopencv-dev libyaml-cpp-dev python3-yaml libboost-all-dev libcurl4-openssl-dev libxml2-dev libbz2-dev"
install_packages "3" "Installing core development tools and libraries" "$core_packages"

# Step 4: Set up ROS Noetic repository and install ROS
log_step "4" "Setting up ROS Noetic repository and installing ROS"
if ! check_package "ros-noetic-desktop-full"; then
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install -y curl && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update && sudo apt install -y ros-noetic-desktop-full
    source /opt/ros/noetic/setup.bash 
fi
check_success "4"

# Step 5: Initialize rosdep and install ROS tools
log_step "5" "Initializing rosdep and installing ROS tools"
if ! check_package "python3-rosdep"; then
    sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo rosdep init && rosdep update
fi
check_success "5"

# Handle special configurations for WSL
if [ "$WSL_CONFIG" == "true" ]; then
    log_step "WSL" "Applying WSL specific configurations"
    echo 'export GAZEBO_IP=127.0.0.1' >> ~/.bashrc
    echo "export DISPLAY=\$(cat /etc/resolv.conf | grep nameserver | awk '{print \$2}'):0" >> ~/.bashrc
    echo 'export LIBGL_ALWAYS_INDIRECT=0' >> ~/.bashrc
    source ~/.bashrc
    echo "WSL configurations applied." | tee -a "$LOG_FILE"
fi

# Function to install repositories or tools
install_repo_tool() {
    if [ "$ACTION" != "uninstall" ]; then
        log_step "$1" "$2"
        if [ ! -d "$3" ]; then
            cd ~ 
            eval <<"EOF"
$4
EOF
        else
            echo "$3 already exists, skipping installation." | tee -a "$LOG_FILE"
        fi
        check_success "$1"
    fi
}

# Step 6: Install PX4 SITL
install_repo_tool "6" "Installing PX4 SITL" "PX4-Autopilot" <<EOF
    # echo "export PATH=$PATH:/home/shival/.local/bin" >> ~/.bashrc 
    source ~/.bashrc
    pip install --upgrade numpy
    # Check if git is already installed before cloning
    if ! command -v git &> /dev/null; then 
        sudo apt install -y git
    fi
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive || { 
        echo "Error cloning PX4-Autopilot repository. Please check your internet connection and try again."
        # exit 1
    } 
    cd PX4-Autopilot &&
    bash ./Tools/setup/ubuntu.sh -y
    # make px4_sitl_default gazebo
EOF

# Step 7: Install Ardupilot SITL and dependencies
install_repo_tool "7" "Installing Ardupilot SITL and dependencies" "ardupilot" <<EOF
    # Check if git is already installed before cloning
    if ! command -v git &> /dev/null; then 
        sudo apt install -y git
    fi
    git clone https://github.com/ArduPilot/ardupilot.git --recursive || { 
        echo "Error cloning ArduPilot repository. Please check your internet connection and try again."
        # exit 1
    } 
    # Ensure the ardupilot directory exists
    mkdir -p ardupilot &&
    cd ardupilot &&
    git checkout Copter-4.0.4 &&
    git submodule update --init --recursive || { 
        echo "Error updating submodules. Trying with https instead of git..." 
        git config --global url."https://".insteadOf git://
        git submodule update --init --recursive || { 
            echo "Error updating submodules even after switching to https. Please check your internet connection and try again."
            # exit 1
        } 
    } &&
    Tools/environment_install/install-prereqs-ubuntu.sh -y &&
    source ~/.profile &&
    cd ~/ardupilot/ArduCopter &&
    sim_vehicle.py -w
EOF

# Step 8: Setup Gazebo and Ardupilot-Gazebo Plugin
install_repo_tool "8" "Setting up Gazebo and Ardupilot-Gazebo Plugin" "ardupilot_gazebo" <<EOF
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list' &&
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - &&
    sudo apt update &&
    sudo apt-get install -y gazebo11 libgazebo11-dev &&
    # Check if git is already installed before cloning
    if ! command -v git &> /dev/null; then 
        sudo apt install -y git
    fi
    git clone https://github.com/khancyr/ardupilot_gazebo.git || { 
        echo "Error cloning ardupilot_gazebo repository. Please check your internet connection and try again."
        # exit 1
    } &&
    cd ardupilot_gazebo &&
    mkdir build && cd build &&
    cmake .. &&
    make -j4 &&
    sudo make install &&
    echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc &&
    echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc &&
    source ~/.bashrc
EOF

# Step 9: Install MAVROS, MAVLink, and IQ Sim
install_repo_tool "9" "Installing MAVROS, MAVLink, and IQ Sim" "" <<EOF
    for i in {1..3}; do # Retry up to 3 times
        if sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavlink; then
            break  # Exit the loop if successful
        else
            echo "Attempt $i failed. Retrying MAVROS installation..."
            sleep 5  # Wait for a few seconds before retrying
        fi
    done
EOF

# Step 10: Install QGroundControl
install_repo_tool "10" "Installing QGroundControl" "" <<EOF
    if ! check_package "qgroundcontrol"; then
        for i in {1..3}; do
            if sudo add-apt-repository ppa:qgroundcontrol/ppa -y &&
               sudo apt update &&
               sudo apt install -y qgroundcontrol; then
                break
            else
                echo "Attempt $i failed. Retrying QGroundControl installation..."
                sleep 5
            fi
        done
    else
        echo "QGroundControl is already installed, skipping..." | tee -a "$LOG_FILE"
    fi
EOF