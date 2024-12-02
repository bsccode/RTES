
# ENS5204 Workshop 1 – Linux Familiarisation and ROS 2 Installation

## Overview

In this workshop, you will be introduced to the Linux desktop environment and guided through the process to install and configure ROS 2 (Robot Operating System 2), which we will use for project work in this unit.

### Workshop Notation
Anything starting with the `➢` symbol indicates an action you need to carry out.

---

## Linux Environment

Linux is a free open-source operating system with features similar to Windows but includes significant differences. It is the primary OS for ROS.

**Important:** The bootable USB provided should be kept safe as losing it will result in the loss of your personal environment.

### Starting the Terminal
To start the Terminal, click the icon resembling this on the left-side menu:

```plaintext
<terminal_icon_placeholder>
```

---

### Viewing and Navigating Directories

#### List Files
```bash
ls
```

#### Change Directory
To change to the `Downloads` directory:
```bash
cd Downloads
```

**Note:** Linux commands are case-sensitive.

#### Autocompletion Shortcut
Press `Tab` to autocomplete commands. For example:
1. Type `cd D` and press `Tab` → suggestions will include `Desktop`, `Documents`, and `Downloads`.
2. Add `ow` to type `cd Dow` and press `Tab` → completes to `cd Downloads`.

#### Print Current Directory
To print the current directory:
```bash
pwd
```

#### Move to Parent Directory
```bash
cd ..
```

#### Home Directory Shortcut
```bash
cd ~
```

#### Create Directories
```bash
mkdir ros2_ws
mkdir ros2_ws/src
```

---

### Useful Commands
- Move files or folders: `mv`
- Copy files or folders: `cp`
- Delete files or folders: `rm` (irreversible)
- Create an empty file: `touch`

For additional help, use:
```bash
man <command>
```

---

## ROS 2 Installation

### Updating Package Index
Run this command to update the package index:
```bash
sudo apt update
```

### Installing ROS 2
Follow the [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) and complete the following sections:
1. **Setup Sources**
2. **Install ROS 2 Packages**: Install the Desktop Install (Recommended) and Development tools.
3. **Environment Setup**
4. **Try Examples**: Use the `Talker` and `Listener` examples to verify the installation.

### Stopping Processes
To stop a running process:
```bash
Ctrl-C
```

---

## Configuring the ROS 2 Environment

Follow the [ROS 2 Environment Configuration Guide](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html).

1. **Source the Setup Files**
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. **Add Sourcing to Shell Startup Script**
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```
3. **Check Environment Variables**
   ```bash
   printenv | grep -i ROS
   ```

---

## Configuring a New ROS Workspace

### Navigate to Workspace Directory
```bash
cd ~/ros2_ws
```

### Build the Workspace
Follow the steps in the [Colcon Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html).

#### If Build Hangs
Use the sequential build option:
```bash
colcon build --symlink-install --executor sequential --event-handlers console_direct+
```

---

## Additional Resources

- [Linux for Robotics Course](https://www.theconstruct.ai/robotigniteacademy_learnros/ros-courses-library/linux-for-robotics/)
- [ROS 2 Concepts](https://docs.ros.org/en/humble/Concepts/Basic.html)

---

Ensure you have shown your tutor the completed examples before leaving!
