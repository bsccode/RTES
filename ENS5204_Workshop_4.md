
# ENS5204 Workshop 4 – Navigation

## Overview

In this workshop, you will use the Nav2 stack to enable the robot to autonomously navigate a simulated environment, leveraging the map created in Workshop 3.

### Workshop Notation
Anything starting with the `➢` symbol indicates an action you need to carry out.

---

## Reloading the Simulation Environment

Ensure you have a saved map of the environment from Workshop 3. If not, complete the steps in the 'Saving Maps' section of Workshop 3.

### Define Alias for RViz with Rosbot Configuration
1. Open the `.bashrc` file and add:
```bash
alias RVIZ_ROSBOT='rviz2 -d ~/ros2_ws/src/tutorial_pkg/rviz/rosbot.rviz'
```
2. Save and close the file.

---

## Navigation Setup

1. Refer to the [Husarion Navigation Tutorial](https://husarion.com/tutorials/ros2-tutorials/9-navigation/) and read through the introduction and parameter information.
2. Download the following files from the Workshop 4 Canvas page:
   - `navigation.yaml` → `~/ros2_ws/src/tutorial_pkg/config`
   - `navigation.launch.py` → `~/ros2_ws/src/tutorial_pkg/launch`
3. Update `CMakeLists.txt` to include:
```plaintext
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
```
4. Build the `tutorial_pkg` repository:
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select tutorial_pkg
```

---

## Running Navigation

### Steps:
1. Start Gazebo simulation:
```bash
ROSBOT_SIM
```
2. Launch navigation tools:
```bash
ros2 launch tutorial_pkg navigation.launch.py use_sim_time:=True
```
3. Open RViz with the Rosbot configuration:
```bash
RVIZ_ROSBOT
```
4. In RViz:
   - Set **Display → Global Options → Fixed Frame** to `Map`.
   - Add costmaps:
     - **Add → By Topic → /global_costmap → costmap-Map**
     - **Add → By Topic → /local_costmap → costmap-Map**
   - Change local costmap **Colour Scheme** to `costmap`.

5. Set the robot’s position using **2D Pose Estimate**.
6. Set a navigation goal using **2D Goal Pose**.

---

## Adjusting Costmap Parameters

### Steps:
1. Modify `inflation_radius` in `navigation.yaml`:
```yaml
local_costmap:
  inflation_radius: 0.1
```
2. Relaunch:
```bash
ROSBOT_SIM
ros2 launch tutorial_pkg navigation.launch.py use_sim_time:=True
RVIZ_ROSBOT
```

### Observe and Test:
- Add costmaps and verify changes.
- Set navigation goals in RViz.

---

## Understanding Costmap Variations

1. Use `teleop_twist_keyboard` to move the robot to a different location.
2. Relaunch navigation and RViz.
3. Observe discrepancies between the costmap and the Gazebo world.

### Questions:
- Why does the costmap not match the actual world?
- Why does the costmap vary as the robot moves? How does this relate to real-world robots?

---

## Final Task: Node for Proximity Sensor Echo

### Requirements:
1. Create a node that echoes the front left proximity sensor's range while moving the robot.
2. Bonus: Add a warning if the range is below 0.3.

### Steps:
1. Write and integrate the node into the `tutorial_pkg` package.
2. Test the node manually.

---

Ensure you demonstrate the navigation, costmap variations, and node functionality to the tutor for workshop marks.
