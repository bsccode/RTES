
# ENS5204 Workshop 3 – Introduction to the ROSbot 2.0 Pro

## Overview

In this workshop, we will explore the ROSbot 2.0 Pro mobile robot platform used in the practical components of the course. This includes both hardware components and ROS software packages for controlling the robot.

### Workshop Notation
Anything starting with the `➢` symbol indicates an action you need to carry out.

---

## ROSbot 2.0 Pro Hardware

Refer to the [ROSbot Manual](https://husarion.com/manuals/rosbot/) and answer the following:

1. **What sensors does the ROSbot 2.0 Pro have in addition to the Astra depth camera and RPLiDAR?**
2. **What microcontroller board is present besides the UPBoard, and what is its purpose?**
3. **What ROS topics provide the battery status and allow speed control? Include the message types.**
4. **How are the motors controlled?**

---

## Installing ROS2 Packages for ROSbot 2 Pro

### Steps:
1. Rename `ros2_ws` to `ros2_old_ws` using the Files utility.
2. Visit the [Husarion ROSbot GitHub Page](https://github.com/husarion/rosbot_ros).
3. Clone the repository and install necessary tools:
```bash
sudo apt-get update
sudo apt-get install -y python3-pip ros-dev-tools stm32flash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone https://github.com/husarion/rosbot_ros src/
```

---

## Using Gazebo for ROSbot 2.0 Pro Simulation

Gazebo simulates the ROSbot in a 3D environment with physics and sensor integration.

### Installing Gazebo Fortress
Follow the [Gazebo Installation Guide](https://gazebosim.org/docs/fortress/install_ubuntu/#binary-installation-on-ubuntu).

Commands:
```bash
sudo apt-get update
sudo apt-get install lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress
```

### Building and Running Simulation
1. Navigate to the [Husarion GitHub Page](https://github.com/husarion/rosbot_ros) and follow the "Build and Run Gazebo Simulation" section.
2. Commands:
```bash
export GZ_VERSION=fortress
export HUSARION_ROS_BUILD=simulation
source /opt/ros/$ROS_DISTRO/setup.bash
vcs import src < src/rosbot/rosbot_hardware.repos
vcs import src < src/rosbot/rosbot_simulation.repos
cp -r src/ros2_controllers/diff_drive_controller src && cp -r src/ros2_controllers/imu_sensor_broadcaster src && rm -rf src/ros2_controllers
sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --executor sequential
```

### Running Simulation
Commands:
```bash
source install/setup.bash
ros2 launch rosbot_gazebo simulation.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## RViz2 Configuration

### Creating a New Package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --license Apache-2.0 tutorial_pkg
cd tutorial_pkg
mkdir rviz
```

### Setting Up RViz2
Refer to [Data Visualisation with Rviz2](https://husarion.com/tutorials/ros2-tutorials/4-kinematics-and-visualization/) for steps. Save `rosbot.rviz` in `~/ros2_ws/src/tutorial_pkg/rviz`. Commands:
```bash
rviz2 -d ~/ros2_ws/src/tutorial_pkg/rviz/rosbot.rviz
```

---

## SLAM Configuration

### Preparing for SLAM
Commands:
```bash
cd ~/ros2_ws/src/tutorial_pkg
mkdir config launch img_data maps
sudo apt install ros-$ROS_DISTRO-slam-toolbox
```

### Save Parameters
Copy the SLAM parameters into `config/slam.yaml`. Edit to set `scan_topic: /scan`.

### Launch SLAM
Commands:
```bash
ros2 launch tutorial_pkg slam.launch.py use_sim_time:=true
rviz2 -d ~/ros2_ws/src/tutorial_pkg/rviz/rosbot.rviz
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Saving and Loading Maps

### Save Map
Commands:
```bash
sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup
cd ~/ros2_ws/src/tutorial_pkg/maps
ros2 run nav2_map_server map_saver_cli -f map
```

### Load Map
Commands:
```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=map.yaml -p use_sim_time:=true
ros2 run nav2_util lifecycle_bringup map_server
```

---

## Connecting to ROSbot via Husarnet

### Install Husarnet
Commands:
```bash
curl https://install.husarnet.com/install.sh | sudo bash
sudo systemctl restart husarnet
husarnet join fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/RCkvypBXTbhbxEWBMAJKKZ/RTESxx
```

### Connect to ROSbot
Commands:
```bash
ssh husarion@rosbot
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

Ensure you review the [Navigation Tutorial](https://husarion.com/tutorials/ros2-tutorials/9-navigation/) before the next workshop.
