---

# ENS5204 Workshop 4: Navigation

## Overview

In the previous workshop, you would have covered using Gazebo and RViz2 to simulate an environment and create a map of that environment. In this workshop, you will use the Nav2 stack (which would have been installed in Workshop 3) to get the robot to autonomously navigate around that environment.

**Workshop Notation**: Anything starting with an arrow bullet (`➢`) indicates an action that you need to carry out.

## Get Simulation Environment Reloaded

➢ You need to have created a map of the whole environment and saved it in Workshop 3 as we will need that for this workshop. If that has not been done yet, then you need to go back and complete that (at least till the end of the ‘Saving Maps’ section).

If you have the map saved from previously, then you can continue.

First, let’s define a new alias for loading Rviz with the saved Rosbot configuration to save time.

➢ Open up the `.bashrc` file and add the following lines below the previous alias definitions:

```bash
alias RVIZ_ROSBOT='rviz2 -d ~/ros2_ws/src/tutorial_pkg/rviz/rosbot.rviz'
```

**Note**: If copying from the worksheet, note that Word has changed the apostrophe character used – and that makes a difference! Use the same apostrophe as in the other alias commands in the `.bashrc` file.

To understand what is in a map file, let’s have a look at it.

➢ Use your File utility to navigate to the `maps` folder as given previously, and you should see the two files `map.pgm` and `map.yaml`.

- The `map.pgm` file is just an image that shows the map.

➢ If you open the `map.yaml` file in an editor, it just has some parameters, including one that specifies the image for the map and also the resolution which should be 0.04. This means that one pixel in the image corresponds to 0.04m in the ‘real’ world.

➢ Now close the file.

### Localisation and Navigation

Localisation is the process by which a robot determines where it is located with respect to its environment. The robot uses algorithms like AMCL to estimate its position by comparing the environment that it senses to the given map of the environment.

Navigation for an autonomous robot is the ability to direct itself (safely) from its current position to a desired destination.

➢ Go to the Husarion tutorial web page on Navigation: [Navigation Tutorial](https://husarion.com/tutorials/ros2-tutorials/9-navigation/). Read through the information from Introduction until the end of the Parameters (just before the Launching navigation section) WITHOUT doing any steps.

As noted on the Husarion tutorial webpage, there are lots of parameters to configure for navigation, so you shall use a pre-prepared configuration file as well as a launch file that will run the required services.

➢ Go to the Workshop 4 page on the Canvas site for this unit.

➢ Download the file `navigation.yaml` and copy it to the folder:

```bash
~/ros2_ws/src/tutorial_pkg/config
```

➢ Download the file `navigation.launch.py` and copy it to the folder:

```bash
~/ros2_ws/src/tutorial_pkg/launch
```

➢ Open the file `CMakeLists.txt` in `~/ros2_ws/src/tutorial_pkg` and in the list of `install(DIRECTORY` add a new entry `rviz` (below the `maps` entry).

➢ Open a terminal window and go to the `ros2_ws` folder:

```bash
cd ~/ros2_ws
```

➢ Build the `tutorial_pkg` repository by running the command:

```bash
colcon build --symlink-install --packages-select tutorial_pkg
```

➢ Open a new terminal window and type in the command `ROSBOT_SIM` to start the Gazebo simulation.

➢ Next, launch the required navigation tools using the command:

```bash
ros2 launch tutorial_pkg navigation.launch.py use_sim_time:=True
```

➢ Open another terminal window and type in the command `RVIZ_ROSBOT` to start the Rviz2 with the Rosbot configured from the last workshop.

➢ In Rviz2, change the `Display – Global Options - Fixed Frame` setting from `Odom` to `Map`.

**Note**: To improve performance, you can remove the camera feed window (bottom left) as well.

➢ Save this configuration by using the menu `File – Save Config` (or `Ctrl-S`).

You should see the robot but no map. Now let's add the previously saved map as costmaps (Refer to the Husarion tutorial page for an explanation of what a costmap is).

➢ In your Rviz2 window, go to `Add - By Topic – /global_costmap – costmap- Map`.

You should see a ‘Costmap’ based on the previously saved map appear.

➢ Next, go to `Add - By Topic – /local_costmap – costmap- Map`.

You should see a smaller square highlighted in the larger map. This is the costmap immediately around the robot.

➢ At the bottom of the left-hand window in RViz2, using the drop-down list, change the `local_costmap` parameter `Colour Scheme` from `map` to `costmap`.

You should see a colorful costmap appear in the rectangle around the robot. This represents what it understands as the costmap in its vicinity.

You might notice some variation between what the robot detects with its lidar (red lines) and the costmap. Think about why this might be the case.

➢ In your Rviz2 window, go to `Add - By Topic – /goal_pose – pose`.

When you set a Goal later on, you can see it as a red arrow on the map.

➢ You can use the green `2D Pose Estimate` arrow to give the system an indication of where it is.

- Click and hold down at the position of the robot, move the arrow around till it is pointing in the direction of the robot, then release it.

➢ Now use the `2D Goal Pose` arrow in the same way to set a goal position and see if the robot can find its way there.

It may struggle because it is in a tight position.

➢ Open a new terminal window and run the shortcut command `TELE_KEY`, which should bring up the `teleop_twist_keyboard` function. Use this to move the robot out to a more ‘open’ area.

➢ Now try setting a new `2D Goal Pose` away from the robot and see if it can find its way there.

✓ Demonstrate to the tutor that you are able to get the Rosbot in Gazebo navigating to a goal (or at least trying to) for part of the workshop marks.

You might notice that the costmap shaded areas between the leaf-shaped obstacles almost touch each other. The shaded areas represent what the navigation algorithm works out as possible obstacles and a ‘clearance’ around them. A large clearance can lead to the robot easily getting ‘stuck’.

➢ Close Gazebo, RViz2, and also kill the navigation process using `Ctrl-C` (or just close the terminal window).

➢ Open the `navigation.yaml` file (located in the `~/ros2_ws/src/tutorial_pkg/config` folder), scroll down to the `local_costmap:` section, and find the parameter `inflation_radius:` and change the value from `0.75` to `0.1`, then save the file.

➢ Now open up Gazebo, the navigation process again, and then start RViz2 (in that order) as previously.

- `ROSBOT_SIM`
- `ros2 launch tutorial_pkg navigation.launch.py use_sim_time:=True`
- `RVIZ_ROSBOT`

➢ Now add both the global and local costmaps as previously and change the `local_costmap` `Colour Scheme` to `costmap`.

Can you see any difference in the local costmap?

➢ Try setting a goal and see if it makes a difference in the ability of the robot to find its way out of ‘crowded’ areas.

✓ Demonstrate to the tutor navigating with the reduced costmap for part of the workshop marks.

➢ Kill the navigation process and RViz2 again.

➢ Use the `teleop_twist_keyboard` function to move the robot to some point far away from the normal starting point (the ‘origin’) in Gazebo.

➢ THEN reload the navigation and RViz2 as previously.

You might notice that the costmap does not match what you see in Gazebo and what the robot’s lidar is picking up.

### Questions

**Q1**: Why does the costmap not match the actual ‘world’?

_________________________________________________________________________________

➢ Use the `teleop_twist_keyboard` to drive the robot around the world and see how the costmap varies. Hint: Head towards corners and other distinctive features.

**Q2**: Why is the costmap varying? How does this relate to what a real robot would do?

_________________________________________________________________________________

_________________________________________________________________________________

✓ Demonstrate to the tutor the ‘varying’ costmap and show your answers to the questions for part of the workshop marks.

## TASK

Now it is time to put together all that you have learned so far in a meaningful way.

➢ You need to write code for a node that echoes the range output of the robot's front left proximity sensor as you move it around the Gazebo world using the teleop function and put it as a node in the `tutorial_pkg` package that you can call. You can use the

 code you created in the previous workshops as a base.

**Hint**: You may want to test this manually first.

➢ For bonus marks: Get your node to display a warning message if the proximity reading is below `0.3`.

✓ Demonstrate to the tutor your node functionality for part of the workshop marks.

---

~ End of Workshop ~

---

