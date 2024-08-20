# ENS5204 Workshop 3: Introduction to the ROSbot 2.0 Pro

## Overview

In this workshop, we will explore the ROSbot 2.0 Pro mobile robot platform that we will be using for the practical components of the course in more detail. We will explore the hardware components of the robot and also the ROS software packages that have been made available to control it.

**Workshop Notation**: Anything starting with an arrow bullet (`➢`) indicates an action that you need to carry out.

## ROSbot 2.0 Pro Hardware

➢ Start by reading through the manual here: [ROSbot Manual](https://husarion.com/manuals/rosbot/), and answer the following questions.  

**Note**: Ensure that you have selected the tab for the PRO model.

1. In addition to the Astra depth camera and RPLiDAR, what other sensors does the ROSbot 2.0 Pro have? 

_______________________________________________________________________________ 

2. In addition to the UPBoard micro PC running Ubuntu Linux, what other microcontroller board is present on the platform, and what purpose does it serve? 

_______________________________________________________________________________ 

3. What ROS topic can be accessed to obtain the current internal battery status, and what topic can messages be published to in order to control the speed of the robot? What message type is used for each of these messages? 

_______________________________________________________________________________ 

4. How are the motors on the robot controlled? 

_______________________________________________________________________________ 

## Installing the ROS 2 Packages for ROSbot 2.0 Pro

➢ First, rename the `ros2_ws` folder we created in the previous workshop to `ros2_old_ws`; otherwise, the following steps will not work. This can easily be done using the Files utility (the icon below FireFox).

➢ Go to the Husarion Rosbot GitHub page: [Husarion Rosbot GitHub](https://github.com/husarion/rosbot_ros).

➢ Scroll down to the section ‘Source Build’ and copy the following commands from the webpage. Make sure you are in the home (`~`) folder when doing so.

➢ Install necessary tools by running the following commands:

```bash
sudo apt-get update
sudo apt-get install -y python3-pip ros-dev-tools stm32flash
```

➢ Create a workspace folder and clone the `rosbot_ros` repository:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone https://github.com/husarion/rosbot_ros src/
```

## Using Gazebo to Simulate the ROSbot 2.0 Pro

To accelerate development and testing, and to enable multiple people to work with the platform at the same time, we will be making use of a simulated version of the ROSbot 2.0 Pro robot within the ROS 2 Gazebo simulator. Gazebo is an open-source 3D robotics simulator with an integrated physics engine, OpenGL rendering, and support for sensor simulation and actuator control with full ROS 2 integration.

➢ Go to the Gazebo page for installing Gazebo Fortress (the correct version of Gazebo for our setup) at [Gazebo Fortress Installation](https://gazebosim.org/docs/fortress/install_ubuntu/#binary-installation-on-ubuntu) and carry out the following sections ONLY (the instructions below can just be copied from the webpage to minimize errors):

1. **First install some necessary tools**:

```bash
sudo apt-get update
sudo apt-get install lsb-release gnupg
```

2. **Then install Ignition Fortress**:

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress
```

➢ Go back to the Husarion Rosbot GitHub page [Husarion Rosbot GitHub](https://github.com/husarion/rosbot_ros), scroll down to the ‘Build and run Gazebo simulation’ section, and carry out the following steps (copy instructions from the webpage):

1. **Add the GZ_VERSION environment variable appropriate to your version**:

```bash
export GZ_VERSION=fortress
```

2. **Building**:

```bash
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

**Note**: Add the `--executor sequential` parameter at the end of the last command to avoid the issue of the system ‘hanging’.

3. **Running Gazebo**:

```bash
source install/setup.bash
ros2 launch rosbot_gazebo simulation.launch.py
```

You should now see The Gazebo application come up with the Gazebo ‘world’ with the Rosbot in it.

➢ To ‘drive’ the Rosbot in the simulation, we can use the teleop via keyboard function (like was done for the turtlesim in Workshop 2). Open a new terminal window and run the command:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

You should see Rosbot move around the simulation world based on the keys that you press.

### Setting Up Aliases in .bashrc

Let’s set things up so that it will be easier to run every time by setting some things up in our `.bashrc` file.

➢ Open up the `.bashrc` file and add the following line at the end:

```bash
source ~/ros2_ws/install/setup.bash
```

➢ In the middle part of the `.bashrc` file, there is a section marked `# Alias definitions`. Just below that, add the following lines:

```bash
alias ROSBOT_SIM='ros2 launch rosbot_gazebo simulation.launch.py'
alias TELE_KEY='ros2 run teleop_twist_keyboard teleop_twist_keyboard'
```

**Note**: If copying from the worksheet, note that Word has changed the apostrophe character used – and that makes a difference! Use the same apostrophe as in the other alias commands in the `.bashrc` file.

The `alias` essentially provides a shorthand that replaces a longer command (or even a sequence of commands) so it is easier to use. From now on, you can just launch the Gazebo simulation using the shortcut `ROSBOT_SIM` and the `teleop_twist_keyboard` node using the shortcut `TELE_KEY`.

➢ Stop Gazebo and the teleop and close both terminal windows.

➢ Open a new terminal window and type in the command `ROSBOT_SIM`. This should start the Gazebo simulation.

➢ Run the `teleop_twist_keyboard` (using the shortcut command `TELE_KEY`) and move the Rosbot around the simulation.

**Note**: You can zoom in to get a better view of how the Rosbot is moving by right-clicking on the Gazebo window, selecting the `Move To` option, and using your mouse to move the view around.

➢ Run the command `ros2 topic echo /odometry/filtered` in a new terminal window, and you should get information about the current location in the window.

This is related to the type of message used to determine the state of the robot of the `nav_msgs/Odometry` type containing information about speeds (`twist`) and positions (`pose`). To get data only for a specific field, you can add the `--field` flag and names of these fields.

➢ Stop the previous command and run the command `ros2 topic echo /odometry/filtered --field pose.pose`, and you should see something like below:

➢ Move the robot around and see how the information changes.

✓ Demonstrate to the tutor that you are able to control the Rosbot in Gazebo with the keyboard for part of the workshop marks.

## RViz2

RViz2 is a tool that allows visualization of robot position, traveled path, planned trajectory, sensor state, or obstacles surrounding the robot.

Before we move on to the next part, we are going to create a new package called `tutorial_pkg` and also create a folder to save our Rviz config. Reminder: Creating packages was covered in Workshop 2.

➢ In a new terminal window, run the following commands:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --license Apache-2.0 tutorial_pkg
cd tutorial_pkg
mkdir rviz
```

➢ Go to the Husarion tutorial page on Kinematics and Visualization: [Kinematics and Visualization](https://husarion.com/tutorials/ros2-tutorials/4-kinematics-and-visualization/).

➢ Skip down to the part `Data Visualization with Rviz2

` and follow the instructions until the Task 1 section.

**Hint**: To stop the Odometry arrows from ‘overcrowding’ the view, reduce the ‘Keep’ parameter value to 3.

**Note**: At the step for saving the configuration, save the `rosbot.rviz` file in the `~/ros2_ws/src/tutorial_pkg/rviz` folder.

➢ Now add the camera image to the Rviz2 configuration.

**Hint**: You need to choose the right topic to see the image. In a terminal window, use the command `ros2 topic list` to see all the topics available.

If done correctly, you should be able to see the camera image at the bottom left of the Rviz2 window.

➢ Now add the lidar output to the visualization.

➢ Move the robot around using your keyboard, compare the camera and lidar outputs, and note how they change. Compare what is in Rviz2 (based on the robot’s sensors) with the ‘world view’ in Gazebo.

**Note**: If there is an issue with the responsiveness of the robot, remove the camera feed (close the camera feed tile).

➢ Save the current configuration file for future use.

Provided you have saved the configuration as noted above, you can recall Rviz with the saved setting using the command:

```bash
rviz2 -d ~/ros2_ws/src/tutorial_pkg/rviz/rosbot.rviz
```

**Note**: This is a long command – so to save time, you may want to create your own alias for it.

✓ Demonstrate to the tutor the odometry, camera, and lidar outputs in Rviz2 for part of the marks.

## SLAM Configuration

SLAM (simultaneous localization and mapping) is a technique for creating a map of the environment and determining robot position at the same time.

➢ Close Rviz2 and Gazebo for now.

➢ Create some new folders for use later on in the exercise by carrying out the following commands:

```bash
cd ~/ros2_ws/src/tutorial_pkg
mkdir config
mkdir launch
mkdir img_data
mkdir maps
```

➢ Go to the Husarion tutorial web page on SLAM: [Husarion SLAM Tutorial](https://husarion.com/tutorials/ros2-tutorials/8-slam/) and read through the description WITHOUT doing any of the instructions until you get to the section on SLAM configuration. You should refer to the parts referring to the Gazebo simulation.

➢ Install the SLAM toolbox:

```bash
sudo apt install ros-$ROS_DISTRO-slam-toolbox
```

You can also copy this command from the webpage.

➢ Below this command, you will find a box with the SLAM parameter information. Copy these parameters and save them in a file called `slam.yaml` in the folder `config` that we created earlier.

➢ Edit the `slam.yaml` file and change the `scan_topic:` parameter from `/scan_filtered` to `/scan`.

**Note**: The `/scan_filtered` parameter is used by a different model of robot; the Rosbot 2 Pro uses `/scan`.

➢ Move further down the page, and you will find another box with the code for the file `slam.launch.py`, which will run the `async_slam_toolbox_node` with the configuration above. Copy this code and save it in a file called `slam.launch.py` in the folder `tutorial_pkg/launch`.

➢ Copy the `CMakeLists.txt` file provided with this workshop into the `tutorial_pkg` folder (overwrite the existing file).

➢ Open a terminal window and go to the `ros2_ws` folder:

```bash
cd ~/ros2_ws
```

➢ Build the `tutorial_pkg` repository by running the command:

```bash
colcon build --symlink-install --packages-select tutorial_pkg
```

### Running SLAM and Testing

Now to actually run SLAM and test it out.

➢ Open another terminal window and launch the SLAM toolbox node using the command:

```bash
ros2 launch tutorial_pkg slam.launch.py use_sim_time:=true
```

➢ Launch Gazebo using the previously defined alias command: `ROSBOT_SIM`.

➢ Launch Rviz2 with the previously saved configuration file using the following command:

```bash
rviz2 -d ~/ros2_ws/src/tutorial_pkg/rviz/rosbot.rviz
```

**Note**: If you have created an alias for the above, you can use that.

➢ In Rviz2, now add the topic `/map`.

- Click on the Add button, select By Topic, and add `Map`.

➢ Run the `teleop_keyboard` node using the shortcut `TELE_KEY`.

➢ Use the keys to move the simulated Rosbot around.

You should now see the map of the area that is being built up by the Rosbot’s lidar – white points indicate free space, black points are a wall or other obstacle, and transparently marked places not yet visited.

➢ Keep moving the Rosbot around until you have built up a map of the whole ‘world’.

**Note**: You can see where the Rosbot is in the ‘world’ in Gazebo.

**Note**: You can change your view of the map using the Views window on the right of the Rviz2 window and changing the type of view.

You can stop when you have a map of most of the world as shown below.

✓ Demonstrate to the tutor the completed map in Rviz2 for part of the marks.

## Saving Maps

➢ The map created will be lost once you shut down the node, so we need to save it. To save the map (and for the next parts), we need the Nav2 (Navigation 2) packages, which you need to install by opening a new terminal window and running the following commands:

```bash
sudo apt install ros-$ROS_DISTRO-navigation2
sudo apt install ros-$ROS_DISTRO-nav2-bringup
```

**Note**: You can find and copy these commands from the Husarion SLAM tutorial page in the section called Maps.

➢ Now go to the `maps` folder we created earlier:

```bash
cd ~/ros2_ws/src/tutorial_pkg/maps
```

➢ Now save the map using the `nav2_map_server` package using the command:

```bash
ros2 run nav2_map_server map_saver_cli -f map
```

The command above should save 2 files in the current (`maps`) folder – `map.pgm` and `map.yaml`.

➢ Check that the 2 files above have been saved.

## Loading Saved Maps

➢ Close RViz2 without saving the configuration file so you can start afresh.

➢ Reopen Rviz2 using the same command as before:

```bash
rviz2 -d ~/ros2_ws/src/tutorial_pkg/rviz/rosbot.rviz
```

or use the alias.

You should see the robot but no map (as it was initially).

➢ To load the map, you need to use the map_server. Make sure you are in the folder with the map (`~/ros2_ws/src/tutorial_pkg/maps`) and then run the command:

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=map.yaml -p use_sim_time:=true
```

➢ We now need to run another utility (`life_cycle_bringup`). Open a new terminal window and run the command:

```bash
ros2 run nav2_util lifecycle_bringup map_server
```

➢ In your Rviz2 window, go to Add - By Topic - Map.

You should see the previously saved map appear.

For now, we shall stop here with regards to the simulation and look at Navigation in the next workshop. For now, we will move on to trying to connect to Rosbot via Husarnet.

## Joining Husarnet VPN

You now need to see if you can connect to the physical Rosbot. Due to the issues caused by ECU’s network security (and to enable easy access to it from anywhere), we shall be connecting to the Rosbot using Husarion’s special VPN called Husarnet.

➢ To install the VPN client on your Linux drive, run the following command:

```bash
curl https://install.husarnet.com/install.sh | sudo bash
```

➢ Then run this command:

```bash
sudo systemctl restart husarnet
```

➢ Join the ENS5204 ‘network’ using the following command:

```bash
husarnet join \fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/RCkvypBXTbhbxEWBMAJKKZ \RTESxx
```

**Note**: Replace the `xx` in the command above with the number on your USB drive, e.g., `RTES01`.

➢ Reboot your system, then open a terminal window. You should see your prompt change to show `ros@RTESxx`.

## Connecting to the Rosbot

You will now be able to access the Rosbot using its hostname (`rosbot`) instead of having to know its IP address.

➢ To test this connection, you should now try to connect to the Rosbot using `ssh` (Secure Shell protocol), using the command below as the superuser `husarion`:

```bash
ssh husarion@rosbot
```

**Note**: The password is also `husarion`.

You should see a screen similar to below:

If you see the above, you are essentially viewing a terminal window on the Rosbot.

➢ Now try to control the Rosbot using the `teleop_twist_keyboard` function. In

 the ssh shell, run the command:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

➢ Control the Rosbot using the `teleop_twist_keyboard` function.

✓ Demonstrate to the tutor your ability to control the Rosbot for part of the marks.

That’s it for this week.

➢ Ahead of next week, have a look through the “Navigation” tutorial: [Navigation Tutorial](https://husarion.com/tutorials/ros2-tutorials/9-navigation).

---

~ End of Workshop ~

---