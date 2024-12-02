
# ENS5204 Workshop 6 – Controlling the Rosbot and Visualising on RViz Remotely

## Overview

In the previous workshop, code that will work with the devices on the Rosbot as well as communicating remotely via topics. In this workshop you are going to use the Rosbot’s topics to control it remotely, visualise its movements on RViz and map the environment on your local device.

### Workshop Notation
Anything starting with the `➢` symbol indicates an action that you need to carry out.

---

## SLAM and Mapping using the Rosbot

In workshop 3, you did SLAM and mapping using Gazebo. Now you are going to do it using the Rosbot while creating the map and running the SLAM and Navigation from your own device using the tools previously installed during Workshop 3.

For this to work, you need to have communications with the Rosbot topics happening, so ensure that you have done the last part of Workshop 5 and that it can view the Rosbot’s topics. If not, you will need to do that first.

Note: There is an updated version of Workshop 5 that addresses some of the issues experienced with that part (Husarnet and its DDS configuration XML).

➢ Open up a terminal window and run the alias command ROBOT_COMM to enable you to communicate with the robot.

Note: You will need to do this with EVERY terminal window, as the setting is per terminal.

➢ Check that you can view all the Rosbot’s topics using the command:
```bash
ros2 topic list
```
o You need to see the topics like `/robot_description` and `/cmd_vel` as well as the various sensor topics.

➢ Start RViz2 using the command:
```bash
rviz2
```
o Note: You will NOT be using the previously saved configuration because that was based on the simulated robot for Gazebo.

➢ Load the Robot Description:
- Add → By Display Type → Robot Model.
- Set the Description Topic to `/robot_description`.

➢ Load the Axes:
- Add → By Display Type → Axes.

➢ Load Odometry and keep reasonable history:
- Add → By Topic → `/odometry/filtered/Odometry`.
- Disable **Odometry-Covariance**.
- Change **Odometry-Keep** to 5.

➢ In order to start mapping, change the Global Options → Fixed Frame to `Odom`.

➢ Load the Lidar scan input:
- Add → By Topic → `/scan/LaserScan`.
- You should see the lidar scan output on RViz.

➢ It is recommended that you save this configuration in a file but use a different name to the Gazebo rosbot configuration.
o Example: Save as `rosbot2pro.rviz` file in the `~/ros2_ws/src/tutorial_pkg/rviz` folder.

You can then call it up quickly using an alias like in Workshop 3.

### Running SLAM
➢ Open another terminal window and run the alias command ROBOT_COMM.

➢ Launch the slam toolbox node using the command:
```bash
cd ~/ros2_ws
ros2 launch tutorial_pkg slam.launch.py use_sim_time:=False
```
o Note: the `use_sim_time` parameter determines whether it is using Gazebo (`true`) or the real robot (`false`).

➢ In RViz2:
- Add → By topic → `/map` → Map.
- You should see the start of the map being formed by the lidar output.

✓ Demonstrate this functionality to the tutor for part of the marks.

---

## Docker

Docker is an open platform for developing, shipping, and running applications. Docker provides the ability to package and run an application in a loosely isolated environment called a container. Containers are lightweight and contain everything needed to run the application, so it doesn't need to rely on what's installed on the host. The isolation and security of containers allows many containers to run simultaneously on a given host. (Docker Inc, 2024).

### Key Docker Terminology
- **Container:** Isolated processes for an app’s components. They are self-contained and have everything the app needs to function with no reliance on pre-installed dependencies on the host machine.
- **Container image:** A standardized package that includes all of the files, binaries, libraries, and configurations to run a container.
- **Registry:** A centralized location that stores and manages container images. Docker Hub is a public registry that anyone can use and is the default registry.
- **Repository:** A collection of related container images within a registry. Each repository contains one or more container images (like a folder to organize images based on projects).

Note: The Rosbot has ROS2 Foxy installed due to limitations on the UP board. However, it has a Docker container running the Humble version of Microros and drivers for the Rosbot, camera, and lidar. **Do NOT modify this setup as it may render the robot inoperable.**

### Installing Docker
➢ To install Docker, refer to the [Docker Installation Guide for Ubuntu](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository).

### Resolving Docker Permission Issues
➢ Add your user to the Docker group:
```bash
sudo usermod -aG docker ros
```

➢ Verify Docker group membership:
```bash
groups ros
```
o You should see `docker` listed among the groups associated with the user `ros`.

➢ Adjust Docker file permissions:
```bash
sudo chown -R ros:docker /var/run/docker.sock
sudo chmod 660 /var/run/docker.sock
```

➢ Restart the Docker service:
```bash
sudo service docker restart
```

➢ Test Docker:
```bash
docker --version
```
o If the command executes successfully without any errors, the permissions should be set properly!

✓ Demonstrate this functionality to the tutor for part of the marks.

---

~ End of Workshop ~
