# Automated Drone Simulation Setup

This guide will help you set up a complete simulation environment for drones using PX4, Ardupilot, Gazebo, MAVROS, and IQ Sim with minimal human intervention.

## Requirements

1. **Ubuntu 20.04 LTS** (for ROS Noetic compatibility) or **Windows 10/11** with WSL2.
2. Internet connection for downloading required packages.

## Installation Steps

### For Ubuntu Users

1. **Clone the repository (if not done already)**:
   You need to get the `setup.sh` script. You can clone it using:
   ```bash
   git clone https://github.com/Shival-Gupta/drones-workshop.git
   cd drones-workshop
   ```

2. **Run the `setup.sh` Script**:
   The script will automate the installation and setup process for ROS, PX4, Ardupilot, Gazebo, MAVROS, and IQ Sim.
   ```bash
   chmod +x setup.sh
   ./setup.sh
   ```

3. **Verify the Installation**:
   After the installation, run the verification script:
   ```bash
   chmod +x verify.sh
   ./verify.sh
   ```

### For Windows Users (WSL2)

1. **Set up WSL2 with Ubuntu 20.04**:
   Follow Microsoft’s guide to install WSL2 with Ubuntu 20.04.

2. **Install XLaunch**:
   Download and install XLaunch to enable GUI applications from WSL.

3. **Clone the repository**:
   Open your WSL terminal and run:
   ```bash
   git clone https://github.com/Shival-Gupta/drones-workshop.git
   cd drones-workshop/wsl
   ```

4. **Run the `setup.sh` Script**:
   The script will automate the installation and setup process, including environment configurations for XLaunch.
   ```bash
   chmod +x setup.sh
   ./setup.sh
   ```

5. **Verify the Installation**:
   After the installation, run the verification script:
   ```bash
   chmod +x verify.sh
   ./verify.sh
   ```

## Troubleshooting
- If Gazebo doesn't launch or fails to connect with the simulation, ensure you have installed the correct version using the setup scripts.
- Verify environment variables are set correctly by running:
   ```bash
   source ~/.bashrc
   ```

## Additional Information
- PX4 Autopilot: [PX4 Official Site](https://px4.io/)
- ArduPilot: [ArduPilot Official Site](https://ardupilot.org/)
- QGroundControl: [QGroundControl Download](https://qgroundcontrol.com/)
- ROS in WSL2: [YouTube Tutorial](https://youtu.be/DW7l9LHdK5c?si=ltJt0esJWsJj9lDu)
