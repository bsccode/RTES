
# ENS5204 Workshop 2 – Getting Started with ROS2: Tools, Publishers and Subscribers

## Overview

In this workshop, we will start to explore the capabilities of ROS2 and how we can use it to construct our own programs. Last week we set up a ROS2 environment within our Linux OS install; this week, we will start to explore how to use this environment. The first step for this is to gain an understanding of some of the most fundamental tools and program structures in ROS2.

### Workshop Notation
Anything starting with the `➢` symbol indicates an action you need to carry out.

---

## Basic ROS2 Tools

### Updating Packages
To ensure all Ubuntu packages are up to date and installed properly:
1. Open a new terminal window.
2. Run:
```bash
sudo apt update
sudo apt upgrade
```
3. Respond `Y` when prompted (password is `ros`).
4. Restart the computer.

### Verify ROS Environment
1. Open a terminal and type:
```bash
printenv | grep -i ROS
```

### Using Turtlesim
1. Refer to the [Introducing Turtlesim Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html).
2. Follow tasks:
   - Install turtlesim.
   - Start turtlesim.
   - Use turtlesim.

✓ Demonstrate control of the turtle with the keyboard for workshop marks.

---

## Installing and Using `rqt`

1. Follow tasks in the [Introducing Turtlesim Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html):
   - Install `rqt`.
   - Use `rqt`.
   - Remapping.

2. Set the second turtle’s pen to yellow, width to 8, and control both turtles in turn.

✓ Demonstrate turtle control with specified pen properties.

---

## ROS2 Communication Concepts

Refer to the [Understanding ROS2 Nodes Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) for background information.

### Tasks
1. Run the following commands:
```bash
ros2 run <package_name> <executable_name>
ros2 node list
ros2 node info /my_turtle
```

---

## Understanding Topics

Refer to the [Understanding Topics Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html).

### Tasks
- Run:
```bash
rqt_graph
ros2 topic list
ros2 topic echo <topic_name>
ros2 topic info <topic_name>
ros2 interface show <interface_type>
ros2 topic pub <topic_name> <msg_type> "{<msg_data>}"
```

---

## Understanding Services

Refer to the [Understanding Services Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html).

### Tasks
- Run:
```bash
ros2 service list
ros2 service type <service_name>
ros2 service call <service_name> <service_type> "{<service_arguments>}"
```

---

## Creating a Package

Refer to the [Creating a Workspace Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

### Steps
1. Clone a sample repository:
```bash
cd ~/ros2_ws/src
git clone <repo_url>
```
2. Resolve dependencies:
```bash
rosdep install -i --from-path src --rosdistro humble -y
```
3. Build the workspace:
```bash
colcon build --packages-select <package_name>
```

---

## Writing a Simple Publisher and Subscriber

Refer to the [Writing a Simple Publisher and Subscriber Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html).

### Tasks
1. Create a package.
2. Write and modify publisher and subscriber nodes:
   - Publisher: `my_publisher.cpp`.
   - Subscriber: `my_subscriber.cpp`.
3. Amend `CMakeLists.txt` for `talker2` and `listener2` executables.
4. Build and test:
```bash
ros2 run <package_name> talker2
ros2 run <package_name> listener2
```

---

## Additional Tasks

1. Install `terminator` for managing multiple terminal instances:
```bash
sudo apt install terminator
```

2. Run:
```bash
terminator
```

3. Split windows horizontally and vertically for multiple terminals.

✓ Demonstrate modified talker and listener nodes with `rqt_graph`.

---

Ensure you have reviewed the [ROS2 Service and Client Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html) before the next workshop.
