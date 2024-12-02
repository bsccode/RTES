
# ENS5204 Workshop 1 – Linux Familiarisation and ROS 2 Installation

## Overview

In this first workshop, you will be introduced to the Linux desktop environment and guided through the process to install and perform initial configuration for the ROS2 (Robot Operating System 2) that we will be using as the basis for the project work in this unit. You will be provided with a bootable USB drive that will start an Ubuntu Linux 22.04 environment on the lab PCs when you boot from it (just plug it in and start, or restart, the PC).

### Workshop Notation
Anything starting with the `➢` symbol indicates an action that you need to carry out.

---

## Linux Environment

Linux is a free open-source operating system with similar features and capabilities to Windows but also some significant differences. This is the OS that best supports ROS, so we will be using it as our main development environment for the semester. The bootable USB you will be provided with should enable you to run it on any computer you can configure to boot from USB. Note, however, if you lose the drive, you will be required to replace it at your cost and will also lose your personal environment, so look after it!

The Linux Desktop environment has a somewhat different layout to the Windows environment you might be used to, but you should be able to get to grips with the basic functionality reasonably easily as the controls are fairly similar. One key difference, however, particularly when using ROS, is that a lot of interactions with the Linux system are carried out through a Command Shell – or Terminal Window.

➢ You can start the Terminal by clicking the icon that looks like this on the menu on the left side of the screen:

### Viewing and Navigating Directories (Folders)

➢ Type `ls` and press Enter.

This will execute the “List” command and will list the files and folders (directories) in the current directory (the home directory that is the default file path when the shell opens).

➢ Try `cd Downloads` to change to the “Downloads” directory.

Note that Linux commands are case-sensitive, so typing `cd downloads` will respond ‘No such file or directory’.

➢ Type `ls` in this new folder.

You should see one file, an installer for Visual Studio Code.

➢ Type `man ls` to see manual options for the `ls` command. Exit with `q`.

➢ Type `pwd` to print the current directory.

➢ Type `cd ..` to move up one directory.

---

## Making Workspace Directories

➢ Create directories:
```bash
mkdir ros2_ws
mkdir ros2_ws/src
```

---

## ROS2 Installation

Use the guide: [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

➢ Follow instructions to:
- Setup Sources
- Install ROS2 Packages (Desktop Install and Development Tools)
- Environment setup
- Try examples

---

## Configuring the ROS2 Environment

➢ Use the guide: [Configuring ROS2 Environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html).

Follow tasks:
1. Source Setup Files
2. Add sourcing to your shell startup script
3. Check Environment Variables:
```bash
printenv | grep -i ROS
```

---

## Configuring a New ROS Workspace

Use the [Colcon Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html).

➢ Follow instructions to:
- Add sources
- Build workspace
- Run tests
- Source environment
- Try demo

---

Ensure you show your tutor you can successfully run the two examples before you leave.

~ End of Workshop ~
