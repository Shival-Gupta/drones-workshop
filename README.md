# Automated Drone Simulation Setup for Ubuntu 20.04 (with/without WSL)

This guide will help you set up a complete simulation environment for drones using PX4, Ardupilot, Gazebo, MAVROS, QGroundControl, and IQ Sim with minimal human intervention.

## Requirements

1. **Ubuntu 20.04 LTS** or Windows with WSL.
2. An **internet connection** for downloading required packages.

## Software Overview

- **PX4**: A popular open-source flight control software for drones.
- **Ardupilot**: A versatile open-source autopilot system supporting many vehicle types.
- **Gazebo**: A 3D robotics simulator that integrates with PX4 and Ardupilot for SITL.
- **MAVROS**: A ROS package that serves as a communication layer between ROS and MAVLink-compatible autopilots.
- **QGroundControl**: A ground control station for drone operations.
- **IQ Sim**: A simulation environment used with PX4 and Ardupilot to simulate different flight scenarios.

## Steps to Run the Setup Script

### Step 0. (Optional for WSL)
Set the default version to WSL 2 and install Ubuntu 20.04:
```bash
wsl --set-default-version 2
wsl --install -d Ubuntu-20.04

```

### Step 1. Clone the Repository

First, clone the repository to get access to the setup script:
```bash
git clone https://github.com/Shival-Gupta/drones-workshop.git
cd drones-workshop

```

### Step 2. Run the `setup.sh` Script

The `setup.sh` script automates the installation and setup process for ROS, PX4, Ardupilot, Gazebo, MAVROS, and IQ Sim.

Make the script executable and run it:
```bash
chmod +x setup.sh
./setup.sh

```

#### Options
- **Default Installation**: `./setup.sh`
- **Uninstall Packages**: `./setup.sh --uninstall`
- **Reinstall Packages**: `./setup.sh --reinstall`
- **Force Install**: `./setup.sh --force-install`
- **Full Installation**: `./setup.sh --full-installation`
- **WSL Configuration**: `./setup.sh --wsl`

### Step 3. Run the Verification Script

After installation, it's recommended to verify that everything is set up correctly:
```bash
chmod +x verify.sh
./verify.sh

```

### Step 4. Launch PX4 SITL with Gazebo

After the installation, you can run PX4 SITL (Software In The Loop) with Gazebo:
```bash
cd ~/PX4-Autopilot
make px4_sitl_default gazebo

```

### Step 5. Launch Ardupilot SITL with Gazebo

To run Ardupilot with Gazebo:
1. Launch Gazebo with the Ardupilot-Gazebo plugin:
   ```bash
   gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world

   ```
2. In another terminal, launch Ardupilot SITL:
   ```bash
   cd ~/ardupilot/ArduCopter
   sim_vehicle.py -v ArduCopter -f gazebo-iris --console

   ```

### Step 6. Running MAVROS and IQ Sim

To launch IQ Sim and get telemetry data from the drone:
1. In one terminal, launch the runway simulation:
   ```bash
   roslaunch iq_sim runway.launch
   
   ```
2. In another terminal, run Ardupilot SITL:
   ```bash
   cd ~
   ./startsitl.sh

   ```
3. Open QGroundControl:
   ```bash
   ./QGroundControl.AppImage

   ```

### Step 7. Using MAVROS and Checking Telemetry Data

To monitor telemetry data:
1. Start MAVROS:
   ```bash
   roslaunch iq_sim apm.launch

   ```
2. View telemetry data from FCU:
   ```bash
   rostopic echo /mavros/global_position/local

   ```
3. Interact with MAVProxy to control the drone:
   ```bash
   mode guided
   arm throttle
   takeoff 10

   ```

## Troubleshooting

- If Gazebo doesn't launch or fails to connect with the simulation, ensure you have installed the Gazebo 11 version using the `setup.sh` script.
- Verify environment variables are set correctly by running:
  ```bash
  source ~/.bashrc
  
  ```
- Check the verification log (`verify.log`) for details about your installation status.

## Additional Information

- PX4 Autopilot: [PX4 Official Site](https://px4.io/)
- ArduPilot: [ArduPilot Official Site](https://ardupilot.org/)
- QGroundControl: [QGroundControl Download](https://qgroundcontrol.com/)
- MAVROS: [MAVROS Wiki](https://wiki.ros.org/mavros)
- ROS in WSL2: [YouTube Tutorial](https://youtu.be/DW7l9LHdK5c?si=ltJt0esJWsJj9lDu)
