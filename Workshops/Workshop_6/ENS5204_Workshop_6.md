# ENS5204 Real-Time Embedded Systems

## Workshop 6 – Controlling the ROSbot and Visualizing on RViz Remotely

### Overview

In the previous workshop, you worked with code that interfaces with the devices on the ROSbot as well as communicating remotely via topics. In this workshop, you are going to use the ROSbot’s topics to control it remotely, visualize its movements on RViz, and map the environment on your local device.

➢ **Workshop Notation**: Anything starting with the ‘arrow bullet’ (like this line) indicates an action that you need to carry out.

### SLAM and Mapping Using the ROSbot

In Workshop 3, you performed SLAM and mapping using Gazebo. Now you are going to do it using the ROSbot while creating the map and running SLAM and Navigation from your own device using the tools previously installed during Workshop 3.

For this to work, you need to have communications with the ROSbot topics happening, so ensure that you have completed the last part of Workshop 5 and that you can view the ROSbot’s topics. If not, you will need to do that first.

**Note**: There is an updated version of Workshop 5 that addresses some of the issues experienced with that part (Husarnet and its DDS configuration XML).

➢ Open up a terminal window and run the alias command `ROBOT_COMM` to enable you to communicate with the robot.

  **Note**: You will need to do this with **EVERY** terminal window, as the setting is per terminal.

➢ Check that you can view all the ROSbot’s topics using the command:

  ```bash
  ros2 topic list
  ```

  - You need to see topics like `/robot_description` and `/cmd_vel` as well as the various sensor topics.

➢ Start RViz2 using the command:

  ```bash
  rviz2
  ```

  **Note**: You will **NOT** be using the previously saved configuration because that was based on the simulated robot for Gazebo.

➢ **Load the Robot Description**

  - **Add** → **By Display Type** → **RobotModel**
  - Set the **Description Topic** to `/robot_description`

➢ **Load the Axes**

  - **Add** → **By Display Type** → **Axes**

➢ **Load Odometry and Keep Reasonable History**

  - **Add** → **By Topic** → `/odometry/filtered` → **Odometry**
  - Disable **Odometry** → **Covariance**
  - Change **Odometry** → **Keep** to `5`

➢ In order to start mapping, change the **Global Options** → **Fixed Frame** to `Odom`.

➢ **Load the Lidar Scan Input**

  - **Add** → **By Topic** → `/scan` → **LaserScan**
  - You should see the lidar scan output on RViz.

➢ It is recommended that you save this configuration in a file but use a different name from the Gazebo ROSbot configuration.

  - For example, save as `rosbot2pro.rviz` file in the `~/ros2_ws/src/tutorial_pkg/rviz` folder.

- You can then call it up quickly using an alias like in Workshop 3.

Now to actually run SLAM and test it out.

➢ Open another terminal window and run the alias command `ROBOT_COMM`.
➢ Launch the SLAM toolbox node using the command:

  ```bash
  cd ~/ros2_ws
  ros2 launch tutorial_pkg slam.launch.py use_sim_time:=False
  ```

  **Note**: The `use_sim_time` parameter determines whether it is using Gazebo (`True`) or the real robot (`False`).

➢ In RViz2, **Add** → **By Topic** → `/map` → **Map**

  - You should see the start of a map being formed by the lidar output.

✅ **Demonstrate this functionality to the tutor for part of the marks.**

**Note**: The next part needs to be coordinated with others in the class so that we don’t send the ROSbot crashing around due to multiple persons controlling it at the same time!

➢ Run the `TELE_KEY` and see if you can start building up a map of the room.

  - **Note**: All running RViz should be able to see the changes as the ROSbot moves.
  - Consult your tutor if this does not happen.

➢ Try saving the map (even if it is a partial map).

  - Refer to Workshop 3 – **Saving Maps** section to check what you need to do to save the maps.

✅ **Demonstrate the saved (full or partial) map to the tutor for part of the marks.**

### Docker

Docker is an open platform for developing, shipping, and running applications. Docker provides the ability to package and run an application in a loosely isolated environment called a **container**. Containers are lightweight and contain everything needed to run the application, so it doesn't need to rely on what's installed on the host. The isolation and security of containers allow many containers to run simultaneously on a given host. (Docker Inc, 2024)

You will be installing Docker on your drive because it has become the de facto standard for distributing and installing software/tools in the ROS2 environment.

Here is some terminology related to Docker:

- **Container**: Isolated processes for an app’s components. They are self-contained and have everything the app needs to function with no reliance on pre-installed dependencies on the host machine.
- **Container Image**: A standardized package that includes all of the files, binaries, libraries, and configurations to run a container.
- **Registry**: A centralized location that stores and manages container images. Docker Hub is a public registry that anyone can use and is the default registry.
- **Repository**: A collection of related container images within a registry. Each repository contains one or more container images (like a folder to organize images based on projects).

There are additional resources on Canvas under **Week 6 Readings and Resources**—especially a good YouTube playlist that you should probably go through if you are planning to use Docker.

**Note**: The ROSbot has ROS2 Foxy installed due to limitations on the UP board; however, it has a Docker container running Humble version of Micro-ROS, and drivers for the ROSbot, camera, and Lidar. **Do NOT mess this up as it may render the robot inoperable!**

You will now install Docker on your local system, so you have it ready in case you need to use it for your projects.

### Installing Docker

➢ To install Docker, go to the Docker installation page for Ubuntu and scroll down to the section **Install using the repository**:

  [Install Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)

➢ Follow the 3 steps listed there by copying the commands (up until running the hello-world image).

#### Step 1: Set up the repository

➢ **Update the apt package index and install packages to allow apt to use a repository over HTTPS:**

  ```bash
  sudo apt-get update
  sudo apt-get install ca-certificates curl gnupg
  ```

➢ **Add Docker’s official GPG key:**

  ```bash
  sudo install -m 0755 -d /etc/apt/keyrings
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
  sudo chmod a+r /etc/apt/keyrings/docker.gpg
  ```

➢ **Use the following command to set up the repository:**

  ```bash
  echo \
    "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
    "$(lsb_release -cs)" stable" | \
    sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
  ```

#### Step 2: Install Docker Engine

➢ **Update the apt package index:**

  ```bash
  sudo apt-get update
  ```

➢ **Install Docker Engine, containerd, and Docker Compose:**

  ```bash
  sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
  ```

#### Step 3: Verify Installation

➢ **Run the hello-world image to verify that Docker Engine is installed correctly:**

  ```bash
  sudo docker run hello-world
  ```

### Sorting Docker Permission Issues

Next, we need to run some additional steps to sort out some permission issues (Singh, 2023).

➢ **Add your user to Docker Group**

  ```bash
  sudo usermod -aG docker ros
  ```

➢ **Verify Docker group membership**

  ```bash
  groups ros
  ```

  - You should see `docker` listed among the groups associated with the user `ros`.

➢ **Adjust Docker file permissions**

  ```bash
  sudo chown -R ros:docker /var/run/docker.sock
  sudo chmod 660 /var/run/docker.sock
  ```

➢ **Restart the Docker Service**

  ```bash
  sudo service docker restart
  ```

➢ **Test Docker**

  ```bash
  docker --version
  ```

  - If the command executes successfully without any errors, the permissions should be set properly!

✅ **Demonstrate this functionality to the tutor for part of the marks.**

~ End of Workshop ~
