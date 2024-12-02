# ENS5204 Real-Time Embedded Systems

## Workshop 2 – Getting Started with ROS2: Tools, Publishers and Subscribers

### Overview

In this workshop we will start to explore the capabilities of ROS2 and how we can use it to construct our own programs. Last week we set up a ROS2 environment within our Linux OS install; this week we will start to explore how to use this environment. The first step for this is to gain an understanding of some of the most fundamental tools and program structures in ROS2.

➢ **Workshop Notation**: Anything starting with the ‘arrow bullet’ (like this line) indicates an action that you need to carry out.

### Basic ROS2 Tools

With ROS2 installed, and the ROS2 `setup.bash` file sourced (which you should have set up to be done automatically through the `.bashrc` file whenever a new terminal is launched), you will have access to a number of ROS2 tools through the command line. You should have tested the most fundamental of these commands last week.

**Note**: Since we have configured the environment so that the `setup.bash` file is sourced automatically, you can ignore the instructions to ‘source ROS2’ in the following exercises.

To check that all the Ubuntu packages are up to date and installed properly, we shall use the `apt update` and `apt upgrade` commands as follows:

➢ Open a new terminal window
➢ Type:

  ```bash
  sudo apt update
  ```

  and press Enter.
  
➢ Then type:

  ```bash
  sudo apt upgrade
  ```

  and press Enter.
  
  - Respond `Y` when asked and remember that the password for superuser `ros` is `ros`.
  - Remember that the prompt responses are case-sensitive.

➢ Restart the computer so the upgraded version of Ubuntu is running.

To check that the environment has been set up properly, do the following:

➢ Open a new terminal window
➢ Type:

  ```bash
  printenv | grep -i ROS
  ```

  and press Enter.
  
➢ Check the environment variables, particularly `DOMAIN_ID` and `LOCALHOST_ONLY` values (refer to Tutorial 1 for what the values should be).

We will now investigate a simple ROS example program that we also installed last week. This is the **turtlesim**, a basic 2D robot simulator.

To start this, we will use the ROS2 command `ros2 run`, which is used to start ROS nodes.

- Remember nodes are essentially our ROS2 programs that carry out the required functions within the system.
- The `ros2 run` command takes the arguments `[package name] [executable name]`.

➢ Go to: [Introducing Turtlesim](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)
➢ Read through the Background information
➢ Skip ahead to the Tasks section and carry out the following tasks by copying and pasting the commands (ensure that you are using the commands in the Linux tab).
  - **1 Install turtlesim**
  - **2 Start turtlesim**
  - **3 Use turtlesim**

✅ **Demonstrate to the tutor that you are able to control the turtle with your keyboard for part of the workshop marks.**

### Installing and using rqt

ROS also includes some visualization tools that enable us to get a picture of what is going on in the system. We will start with **rqt**, which is a framework of GUI tools for ROS.

➢ Move ahead in the Tasks section and carry out the following tasks by copying and pasting the commands (ensure that you are using the commands in the Linux tab).
  - **4 Install rqt**
  - **5 Use rqt**
  - **6 Remapping**

➢ Set the second turtle’s pen color to yellow and pen width to 8 using rqt and move both turtles in turn by alternating between the teleop terminal windows.

✅ **Demonstrate to the tutor that you are able to control both turtles with your keyboard with the pen properties as given for part of the workshop marks.**

➢ Stop the simulation using the commands listed in the section **7 Close turtlesim**

**Note**: We will be using turtlesim again in the following exercises, but it is better to restart it ‘clean’.

### Understanding nodes, topics, services, actions, and parameters

The overall software control of a robot in ROS2 is achieved via a distributed system described by what is called a **Computational Graph** or simply a **ROS graph**. This ‘graph’ consists of many nodes, where a node is an object that is responsible for a single modular purpose. Each node can send and receive data from other nodes via topics, services, actions, or parameters. (Open Robotics, 2024).

- **Topics**: Act as a bus for nodes to exchange messages. Publishers put messages on a topic, and subscribers read those messages. A topic can have multiple publishers and subscribers.
- **Services**: Another method of communication, but they work on a call-and-response model (only provide data when specifically called by a client).
- **Actions**: Another method of communication intended for long-running tasks, consisting of 3 parts: a goal, feedback, and result. It is based on a client-server model where the ‘action client’ node sends a goal to an ‘action server’ node that acknowledges it, returns a stream of feedback (via feedback topic), and a result.
- **Parameters**: Configuration values of a node (node settings) that can be of various types.

The following exercises are designed to build your understanding of these concepts.

➢ Go to **Understanding Nodes** exercise: [Understanding ROS2 Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
➢ Read through the Background information
➢ Skip ahead to the Tasks section and carry out the following tasks by copying and pasting the commands given on the webpage:
  - **1 ros2 run**
  - **2 ros2 node list**
  - **3 ros2 node info**

The command `ros2 node info /my_turtle` should return output like this:

```
Topic name                         Topic data type
/my_turtle
Subscribers:
/parameter_events                  rcl_interfaces/msg/ParameterEvent
/turtle1/cmd_vel                   geometry_msgs/msg/Twist
Publishers:
/parameter_events                  rcl_interfaces/msg/ParameterEvent
/rosout                            rcl_interfaces/msg/Log
/turtle1/color_sensor              turtlesim/msg/Color
/turtle1/pose                      turtlesim/msg/Pose
Service Servers:
/clear                             std_srvs/srv/Empty
/kill                              turtlesim/srv/Kill
/my_turtle/describe_parameters     rcl_interfaces/srv/DescribeParameters
/my_turtle/get_parameter_types     rcl_interfaces/srv/GetParameterTypes
/my_turtle/get_parameters          rcl_interfaces/srv/GetParameters
/my_turtle/list_parameters         rcl_interfaces/srv/ListParameters
/my_turtle/set_parameters          rcl_interfaces/srv/SetParameters
/my_turtle/set_parameters_atomically rcl_interfaces/srv/SetParametersAtomically
/reset                             std_srvs/srv/Empty
/spawn                             turtlesim/srv/Spawn
/turtle1/set_pen                   turtlesim/srv/SetPen
/turtle1/teleport_absolute         turtlesim/srv/TeleportAbsolute
/turtle1/teleport_relative         turtlesim/srv/TeleportRelative
Service Clients:
Action Servers:
/turtle1/rotate_absolute           turtlesim/action/RotateAbsolute
Action Clients:
```

➢ Compare the connections on the `teleop_turtle` and `my_turtle` nodes and try to figure out the **TWO** connections that enable the `teleop_turtle` node to control the movements of the turtle.

**Answers**:

1. ___________________________________________________________
2. ___________________________________________________________

➢ Use the arrows in the `turtle_teleop_key` window and observe what happens to with the turtles in **BOTH** windows. Why do you think that happens?

To better understand the reason for the last observation, let’s move to the next exercise.

➢ Move on to the **Understanding Topics** exercise: [Understanding ROS2 Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
➢ Read through the Background information then carry out the following task:
  - **2 rqt_graph**

**Note**: Do not do the Setup task before the `rqt_graph` task; instead, leave the two `turtlesim` windows and `turtle_teleop_key` window from the previous exercise open. The output may look different from the exercise but should better explain the last observation.

➢ To set things back to match the exercise, let’s close both `turtlesim` windows (using `Ctrl-C`) and restart one `turtlesim` using the command:

  ```bash
  ros2 run turtlesim turtlesim_node
  ```

  (as per the Setup task)

➢ Go to the `rqt_graph` and hit the refresh button to check if it now matches what is shown in the exercise.

➢ Continue with the following tasks:
  - **3 ros2 topic list**
  - **4 ros2 topic echo**
  - **5 ros2 topic info**
  - **6 ros2 interface show**
  - **7 ros2 topic pub**
  - **8 ros2 topic hz**

**Note**: *Pose* in robotics means the position and orientation of a robot (or robot element).

✅ **Demonstrate to the tutor that you are able to control the turtle with the command line publishing to the topic regularly as given for part of the workshop marks.**

➢ Terminate all the active nodes by using `Ctrl-C` before moving to the next exercise.

➢ Move on to the **Understanding Services** exercise: [Understanding ROS2 Services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)

➢ Read through the Background information then carry out the following tasks:
  - **1 Setup**
  - **2 ros2 service list**
  - **3 ros2 service type**
  - **4 ros2 service find**
  - **5 ros2 interface show**
  - **6 ros2 service call**

➢ Move on to the **Understanding Parameters** exercise: [Understanding ROS2 Parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)

➢ Read through the Background information then carry out the following tasks:
  - **2 ros2 param list**
  - **3 ros2 param get**
  - **4 ros2 param set**
  - **5 ros2 param dump**
  - **6 ros2 param load**
  - **7 Load parameter file on node startup**

➢ Move on to the **Understanding Actions** exercise: [Understanding ROS2 Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

➢ Read through the Background information then carry out the following tasks:
  - **2 Use actions**
  - **3 ros2 node info**
  - **4 ros2 action list**
  - **5 ros2 action info**
  - **6 ros2 interface show**
  - **7 ros2 action send_goal**

To find out what is going on in your system (particularly when trying to debug code), it is useful to be able to monitor log messages. **rqt_console** is a very useful tool for this.

➢ Move on to the **Using rqt_console to view logs** exercise: [Using Rqt Console](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Using-Rqt-Console/Using-Rqt-Console.html)

➢ Read through the Background information then carry out the following tasks:
  - **1 Setup**
  - **2 Messages on rqt_console**
  - **3 Logger levels**

➢ Close all nodes and terminal windows from previous exercises.

### Launch files

Any kind of meaningful project will require a number of nodes running simultaneously. Launch files provide a mechanism that allows you to start up and configure a number of ROS2 nodes simultaneously.

➢ Go to the **Launching nodes** exercise: [Launching Multiple Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html)

➢ Read through the Background information then carry out the following tasks:
  - **Running a Launch File**
  - **Control the Turtlesim Nodes**

➢ Modify the commands above to have the two turtles moving at different speeds and circle sizes.

✅ **Demonstrate to the tutor both turtles moving as per the last step above exercise**

### Creating a package

In order to install your ROS2 code or share it with others, you will need to organize it in a package (Open Robotics, 2024). We will now go through some examples of how this is done, building on some preliminary work done in Workshop 1.

In the last part of Workshop 1, in addition to configuring the ROS2 environment, you would also have configured a new ROS2 workspace. You would have already created a workspace folder `~/ros2_ws` that contains the required folders within it (`src`, `build`, `install`, and `logs`).

The following exercise is to further understand how this workspace is to be used, but we can skip a couple of steps.

➢ Close all open terminal windows, and open a fresh terminal window
➢ Type in the command:

  ```bash
  cd ~/ros2_ws/src
  ```

➢ Go to the **Creating a Workspace** exercise: [Creating a Workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

➢ Read through the Background information, then skip the first 2 tasks and carry out the following task:
  - **3 Clone a sample repo**

➢ Open the Files utility (should be the second icon, below Firefox, in your left-hand menu bar).
  - This is the Ubuntu equivalent of File Explorer in Windows

➢ Browse to the folder `~/ros2_ws/src`. You should now see the folder `ros_tutorials` in there.

➢ In the terminal window type in the command:

  ```bash
  ls
  ```

  to view the contents.

➢ Copy the commands from the **4 Resolve dependencies** task and run it
  - Commands:

    ```bash
    cd ..
    rosdep install -i --from-path src --rosdistro humble -y
    ```

You will probably see an error message saying that the `rosdep` installation has not been initialized yet. We will now run the two commands it suggests.

➢ Run the command:

  ```bash
  sudo rosdep init
  ```

  **Note**: If it asks for a password, it is `ros`

➢ Then run the command:

  ```bash
  rosdep update
  ```

➢ Now check the dependencies again by running the earlier `rosdep install` command again
  - Remember that you can ‘replay’ commands in a terminal window by using the up arrow, finding the command, and then hitting `Enter`.
  - It should now show the message `#All required rosdeps installed successfully`

➢ Continue the **Creating a Workspace** exercise and carry out the following tasks:
  - **5 Build the workspace with colcon**
  - **6 Source the overlay**
    - **Note** the instruction that this MUST be done in a new terminal window
  - **7 Modify the overlay**

**Note**: If you face an issue with the system hanging during the build, use the fix as in Workshop 1.

✅ **Show your tutor the TurtleSim window with the new title**

➢ Close all open terminal windows, and open a fresh terminal window

➢ Go to the **Creating a package** exercise: [Creating Your First ROS2 Package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

➢ Read through the Background information, then carry out the following tasks:
  - **1 Create a package**
  - **2 Build a package**
  - **3 Source the setup file**
  - **4 Use the package**
  - **5 Examine package contents**
  - **6 Customize package.xml**

As you would have seen in previous tasks, whenever we want to use a workspace in a new terminal, or if we have run a `colcon build`, we need to source a setup file in the workspace. This setup file is automatically created as `local_setup.bash` in the workspace `install` folder.

You will need to run this after every `colcon build`, but if you want to avoid needing to run it every time you open a new terminal, you should add it to your `.bashrc` startup script.

➢ To do this, use the Ubuntu file manager, and go to the Home folder
➢ Press `Ctrl-h` to show hidden files.
➢ Double-click `.bashrc` to open it in a text editor and scroll to the end. You should see the `source /opt/ros/humble/setup.bash` line that you added previously and the export settings.
➢ Add the new source line below this, then save and close the file.
  - Line to add:

    ```bash
    source ~/ros2_ws/install/local_setup.bash
    ```

### Creating your own Publisher and Subscriber

We will now create our own publisher and subscriber, which will give you an idea of how ROS code is structured.

➢ Close all open terminal windows, and open a fresh terminal window
➢ Go to the **Writing a simple publisher and subscriber (C++)**: [Writing a Simple Cpp Publisher And Subscriber](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)

➢ Read through the Background information, then carry out the following tasks:
  - **1 Create a package**
  - **2 Write the publisher node**
  - **3 Write the subscriber node**
  - **4 Build and run**

Now let’s apply what has been covered by modifying the Publisher-Subscriber code and get it working.

➢ In a new terminal window, go to the `~/ros2_ws/src/cpp_pubsub/src` folder
  - Command is:

    ```bash
    cd ~/ros2_ws/src/cpp_pubsub/src
    ```

➢ Run the command:

  ```bash
  cp publisher_member_function.cpp my_publisher.cpp
  ```

➢ Next, the command:

  ```bash
  cp subscriber_member_function.cpp my_subscriber.cpp
  ```

You should now see copies of the publisher and subscriber code files in the `ros2_ws/src/cpp_pubsub/src` folder with these new names.

➢ Modify the code in the `my_publisher.cpp` file so that it has:
  - Node name: `MyPublisher`
  - Topic: `my_topic`
  - Message: “Message from `<yourname>` number:” + the count
  - Timing: 1 second

➢ Modify the code in the `my_subscriber.cpp` file so that it:
  - Has node name `MySubscriber`
  - Listens to the messages sent by the `MyPublisher` node above

➢ Amend the `CMakeLists.txt` file so that the package will be created with these new publisher and subscriber nodes and name the executable as `talker2` and `listener2` respectively

➢ Go to the root of your workspace (`ros2_ws` folder) using the command:

  ```bash
  cd ~/ros2_ws
  ```

➢ Check for dependencies:

  ```bash
  rosdep install -i --from-path src --rosdistro humble -y
  ```

➢ Build your new package:

  ```bash
  colcon build --packages-select cpp_pubsub
  ```

To make testing a little easier we will install one more program that makes switching between multiple terminals a little easier.

➢ In a new terminal type:

  ```bash
  sudo apt install terminator
  ```

  and press Enter.

This will install another terminal application that allows multiple terminal instances in one window.

➢ Once it has installed, type:

  ```bash
  terminator
  ```

  to run it.

➢ You may want to right-click on the **Terminator** icon that appears on the left side menu and select **“Add to favorites”** to pin it there.

At the moment it just looks like a regular terminal window.

➢ Right-click and you can select **Split Horizontally**, which will give you two terminals
➢ Then right-click separately in the top and then the bottom and select **Split Vertically** in each to end up with four terminals in one window.

Now let us test your modified code:

➢ In one of the 4 windows, change to the root of your workspace (`~/ros2_ws`) then run the command:

  ```bash
  ros2 run cpp_pubsub talker2
  ```

➢ In another window, change to the root of your workspace (`~/ros2_ws`) then run the command:

  ```bash
  ros2 run cpp_pubsub listener2
  ```

You should now see both the talker and listener with a message including your name.

You can also try posting a message to your topic directly from the command line.

➢ In the 3rd window, run the command:

  ```bash
  ros2 topic pub --once /my_topic std_msgs/msg/String "{data: 'Hello there from terminal 3'}"
  ```

➢ Run `rqt_graph` from another window to see the nodes and topic involved

✅ **Demonstrate to the tutor your modified talker and listener interaction above as well the `rqt_graph` showing the names to get the remainder of the workshop marks.**

Before next week, also have a look through the ROS2 tutorial on writing a simple service and client:

[Writing a Simple Service and Client (C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html)
