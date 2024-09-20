# Automated Drone Simulation Setup for WSL

This guide will help you set up a complete simulation environment for drones using PX4, Ardupilot, Gazebo, MAVROS, and IQ Sim on Windows Subsystem for Linux (WSL) with minimal human intervention.

## Requirements

1. **WSL2** with Ubuntu 20.04 LTS.
2. **XLaunch** or any other X server running on Windows.
3. Internet connection for downloading required packages.

## Steps to Run the Setup Script

### 1. Install WSL2 and Ubuntu
Follow the official guide to install WSL2 and set up Ubuntu 20.04.

### 2. Clone the repository (if not done already)
You need to get the `wsl/setup.sh` script. You can clone it using:
```bash
git clone https://github.com/Shival-Gupta/drones-workshop.git
cd drones-workshop/wsl
```

### 3. Run the `setup.sh` Script
The script will automate the installation and setup process for ROS, PX4, Ardupilot, Gazebo, MAVROS, and IQ Sim.

Run the script:
```bash
chmod +x setup.sh
./setup.sh
```

### 4. Launch PX4 SITL with Gazebo
After the installation, you can run PX4 SITL (Software In The Loop) with Gazebo using the following commands:
```bash
cd ~/PX4-Autopilot
make px4_sitl_default gazebo
```

### 5. Launch Ardupilot SITL with Gazebo
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

### 6. Running MAVROS and IQ Sim
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

### 7. Using MAVROS and Checking Telemetry Data
To monitor telemetry data:
1. Start MAVROS:
   ```bash
   roslaunch iq_sim apm.launch
   ```
2. View telemetry data from FCU:
   ```bash
   rostopic echo /mavros/global_position/local
   ```
3. You can interact with MAVProxy to control the drone:
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

## Additional Information
- PX4 Autopilot: [PX4 Official Site](https://px4.io/)
- ArduPilot: [ArduPilot Official Site](https://ardupilot.org/)
- QGroundControl: [QGroundControl Download](https://qgroundcontrol.com/)
- ROS in WSL2: [YouTube Tutorial](https://youtu.be/DW7l9LHdK5c?si=ltJt0esJWsJj9lDu)
