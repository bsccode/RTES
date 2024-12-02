
# ENS5204 Workshop 2 – Getting Started with ROS2: Tools, Publishers and Subscribers

## Overview

In this workshop, we will start to explore the capabilities of ROS2 and how we can use it to construct our own programs. Last week we set up a ROS2 environment within our Linux OS install; this week, we will start to explore how to use this environment. The first step for this is to gain an understanding of some of the most fundamental tools and program structures in ROS2.

### Workshop Notation
Anything starting with the `➢` symbol indicates an action that you need to carry out.

---

## Basic ROS2 Tools

With ROS2 installed, and the ROS2 `setup.bash` file sourced (which you should have set up to be done automatically through the `.bashrc` file whenever a new terminal is launched), you will have access to a number of ROS2 tools through the command line. You should have tested the most fundamental of these commands last week.

Note: Since we have configured the environment so that the `setup.bash` file sources automatically, you can ignore the instructions to ‘source ROS2’ in the following exercises.

To check that all the Ubuntu packages are up to date and installed properly:

➢ Open a new terminal window.  
➢ Type `sudo apt update` and press Enter.  
➢ Then type `sudo apt upgrade` and press Enter.  
  - Respond ‘Y’ when asked. Remember that the password for superuser `ros` is `ros`.  
  - Remember that the prompt responses are case-sensitive.  
➢ Restart the computer so the upgraded version of Ubuntu is running.

To check that the environment has been set up properly, do the following:

➢ Open a new terminal window.  
➢ Type `printenv | grep -i ROS` and press Enter.  
➢ Check the environment variables, particularly DOMAIN_ID and LOCALHOST_ONLY values (refer to Workshop 1 for the correct values).

---

## Using Turtlesim

We will now investigate a simple ROS example program: Turtlesim, a basic 2D robot simulator.

To start this, we will use the ROS2 command `ros2 run`, which is used to start ROS nodes.  
  - Nodes are essentially our ROS2 programs that carry out the required functions within the system.  
  - The `ros2 run` command takes the arguments `[package name] [executable name]`.  

➢ Go to the [Introducing Turtlesim Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html).  
➢ Read through the Background information.  
➢ Skip ahead to the Tasks section and carry out the following tasks:  
  1. Install turtlesim.  
  2. Start turtlesim.  
  3. Use turtlesim.

✓ Demonstrate to the tutor that you are able to control the turtle with your keyboard for part of the workshop marks.

---

## Installing and Using rqt

ROS also includes some visualisation tools. We will start with `rqt`, a framework of GUI tools for ROS.

➢ Move ahead in the Tasks section of the Turtlesim guide and carry out the following tasks:  
  4. Install `rqt`.  
  5. Use `rqt`.  
  6. Remapping.

➢ Set the second turtle’s pen colour to yellow and pen width to 8 using `rqt`, and move both turtles alternately.

✓ Demonstrate to the tutor that you are able to control both turtles with the given pen properties for part of the workshop marks.

➢ Stop the simulation using the commands listed in the "Close turtlesim" section.

---

## Understanding Nodes, Topics, Services, Actions, and Parameters

ROS2 software control is achieved via a distributed system described as the ROS graph. This consists of many nodes, where each node has a single modular purpose. Nodes communicate via:

- **Topics:** Bus-like exchanges where publishers post messages and subscribers read them.  
- **Services:** Call-and-response communication, triggered by a client request.  
- **Actions:** For long-running tasks, consisting of a goal, feedback, and result, using a client-server model.  
- **Parameters:** Configuration values for nodes.  

### Tasks

➢ Use the [Understanding Nodes Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html).  
➢ Read the Background information.  
➢ Skip to the Tasks section and perform these tasks:  
  1. Run `ros2 run`.  
  2. List nodes with `ros2 node list`.  
  3. Display node info with `ros2 node info /my_turtle`.

---

Continue through similar tasks to explore:

1. **Topics**: [Understanding Topics Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html).  
2. **Services**: [Understanding Services Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html).  
3. **Parameters**: [Understanding Parameters Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html).  
4. **Actions**: [Understanding Actions Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html).

✓ Demonstrate tasks and observations to the tutor for workshop marks.

---

## Creating a ROS2 Package

Follow the guide: [Creating a Workspace Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).  

### Tasks

1. Clone a sample repository.
2. Resolve dependencies:
```bash
sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
```
3. Build and test the workspace.

---

Follow the [Creating Your First ROS2 Package Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) to:

1. Create a package.  
2. Write publisher and subscriber nodes.  
3. Modify and test new nodes.  
4. Visualise the communication using `rqt_graph`.

✓ Demonstrate modified nodes and `rqt_graph` to the tutor.

~ End of Workshop ~
