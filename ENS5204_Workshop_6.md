
# ENS5204 Workshop 6 – Controlling the Rosbot and Visualising on RViz Remotely

## Overview

This workshop focuses on using the Rosbot's topics to control it remotely, visualising its movements on RViz, and mapping the environment on your local device.

### Workshop Notation
Anything starting with the `➢` symbol indicates an action you need to carry out.

---

## SLAM and Mapping Using the Rosbot

1. Ensure you can view the Rosbot's topics using:
```bash
ROBOT_COMM
ros2 topic list
```
   - Verify topics like `/robot_description` and `/cmd_vel` are visible.
2. Start RViz2:
```bash
rviz2
```
3. Configure RViz2:
   - Add **Robot Model** and set the Description Topic to `/robot_description`.
   - Add **Axes**.
   - Add **Odometry** from `/odometry/filtered/Odometry`.
     - Disable **Odometry-Covariance**.
     - Set **Odometry-Keep** to 5.
   - Set **Global Options → Fixed Frame** to `Odom`.
   - Add Lidar scan from `/scan/LaserScan`.

4. Save the configuration as `rosbot2pro.rviz` in `~/ros2_ws/src/tutorial_pkg/rviz`.

### Running SLAM
1. Launch SLAM Toolbox:
```bash
ROBOT_COMM
cd ~/ros2_ws
ros2 launch tutorial_pkg slam.launch.py use_sim_time:=False
```
2. In RViz2, add **Map** from `/map`.

✓ Demonstrate SLAM functionality to the tutor.

---

## Building the Map

1. Use `teleop_twist_keyboard` to control the Rosbot and build the map.
2. Save the map as per Workshop 3 instructions.

✓ Demonstrate the saved map to the tutor.

---

## Installing Docker

Docker is essential for running ROS2 applications in isolated containers.

### Installation Steps
1. Refer to the [Docker Installation Guide](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository).
2. Follow the steps to install Docker and test with the `hello-world` image.

### Resolving Permission Issues
1. Add the user to the Docker group:
```bash
sudo usermod -aG docker ros
```
2. Verify group membership:
```bash
groups ros
```
   - Ensure `docker` is listed.
3. Adjust file permissions:
```bash
sudo chown -R ros:docker /var/run/docker.sock
sudo chmod 660 /var/run/docker.sock
```
4. Restart Docker service:
```bash
sudo service docker restart
```
5. Verify Docker installation:
```bash
docker --version
```

✓ Demonstrate Docker functionality to the tutor.

---

~ End of Workshop ~
