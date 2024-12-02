
# ENS5204 Workshop 5 – Writing Code for the Rosbot and Communication via Topics

## Overview

This workshop focuses on writing code to interact with the devices on the Rosbot and communicate remotely via ROS topics.

### Workshop Notation
Anything starting with the `➢` symbol indicates an action you need to carry out.

---

## Creating Your Own Workspace and Code

### Steps:
1. Refer to the [Creating Nodes and Messages Tutorial](https://husarion.com/tutorials/ros2-tutorials/2-creating-nodes-messages/).
2. Create a package named `rtes_xx_pkg` (replace `xx` with your USB number).
3. Update `package.xml` with `Apache-2.0` license if errors occur.
4. Follow the tutorial steps to:
   - Create a subscriber node (use `camera/color/image_raw` topic).
   - Test the node using Gazebo and `teleop_twist_keyboard`.
5. Continue to the publisher node creation.

✓ Demonstrate functionality to the tutor (both subscribing and publishing).

---

## Using GitLab

### Set Up GitLab
1. Ensure your GitLab account is active and set up.
2. Generate an SSH key:
```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
cat ~/.ssh/id_ed25519.pub
```
3. Add the SSH key to your GitLab account under **Profile Settings → SSH Keys**.
4. Test the connection:
```bash
ssh -T git@gitlab.com
```

### Upload Code to GitLab
1. Install Git:
```bash
sudo apt-add-repository ppa:git-core/ppa
sudo apt-get update
sudo apt-get install git
```
2. From `~/ros2_ws/src/rtes_xx_pkg` directory, run:
```bash
git init
git config --global user.name "<Your name>"
git config --global user.email "<Your email>"
git add CMakeLists.txt package.xml src/ include/
git commit -m "<Your name> Workshop 5 commit"
git remote add origin https://gitlab.com/ecueng/ens5204-2024.git
git branch -M <Your name>
git push -uf origin <Your name>
```

✓ Demonstrate your node running on the robot to the tutor.

---

## Communicating with the Rosbot

### Ensure Setup
1. Connect to Husarnet (see Workshop 3).
2. SSH into the Rosbot:
```bash
ssh husarion@rosbot
```
3. Clone your GitLab repository onto the Rosbot:
```bash
cd ~/ros_ws/src
git clone -b <Your_name> https://gitlab.com/ecueng/ens5204-2024.git rtes_xx_pkg
```
4. Build the package:
```bash
cd ~/ros_ws
colcon build --packages-select rtes_xx_pkg
```
5. Run the node:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run rtes_xx_pkg my_first_node
```

### Testing Camera Node
1. Monitor the `/brightness` topic output while changing the light level.
2. Make changes to the code and push updates using Git:
```bash
git commit -a -m "<Your name> Workshop 5 commit"
git push
```
3. Pull updates on the Rosbot:
```bash
git pull
```

✓ Demonstrate to the tutor.

---

## Configuring FastDDS Middleware

### Install and Configure
1. Install FastDDS:
```bash
sudo apt install ros-humble-rmw-fastrtps-cpp
```
2. Install Husarnet DDS utility:
```bash
RELEASE="v1.3.5"
ARCH="amd64"
sudo curl -L https://github.com/husarnet/husarnet-dds/releases/download/$RELEASE/husarnet-dds-linux-$ARCH -o /usr/local/bin/husarnet-dds
sudo chmod +x /usr/local/bin/husarnet-dds
```
3. Generate XML configuration:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/var/tmp/husarnet-fastdds-simple.xml
husarnet-dds singleshot
```

4. Add to `.bashrc`:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/var/tmp/husarnet-fastdds-simple.xml
```

---

## Controlling Robot Access with LOCALHOST

### Configure `.bashrc`
Add these aliases:
```bash
alias ROBOT_COMM='export ROS_LOCALHOST_ONLY=0'
alias LOCAL_COMM='export ROS_LOCALHOST_ONLY=1'
```

### Test Communication
1. Switch to global communication:
```bash
ROBOT_COMM
ros2 topic echo /testmsg
```
2. Test local communication:
```bash
LOCAL_COMM
ros2 topic echo /testmsg
```

✓ Demonstrate message monitoring to the tutor.

---

## Testing from Your Own Device

1. Connect to Husarnet.
2. Ensure Gazebo is not running.
3. Run:
```bash
ROBOT_COMM
ros2 topic list
```
4. Run your camera node to monitor brightness.

✓ Demonstrate functionality from your own device.

~ End of Workshop ~
