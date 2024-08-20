
# ENS5204 Workshop 1: Linux Familiarisation and ROS 2 Installation

## Overview

In this first workshop, you will be introduced to the Linux desktop environment and guided through the process of installing and performing initial configuration for ROS 2 (Robot Operating System 2), which we will be using as the basis for the project work in this unit.

You will be provided with a bootable USB drive that will start an Ubuntu Linux 22.04 environment on the lab PCs when you boot from it (just plug it in and start or restart the PC).

**Workshop Notation**: Anything starting with an arrow bullet (`➢`) indicates an action that you need to carry out.

## Linux Environment

Linux is a free, open-source operating system with similar features and capabilities to Windows but also some significant differences. This is the OS that best supports ROS, so we will be using it as our main development environment for the semester.

The bootable USB you will be provided with should enable you to run it on any computer you can configure to boot from USB. Note, however, if you lose the drive, you will be required to replace it at your cost and will also lose your personal environment, so look after it!

The Linux desktop environment has a somewhat different layout from the Windows environment you might be used to, but you should be able to get to grips with the basic functionality reasonably easily as the controls are fairly similar. One key difference, however, particularly when using ROS, is that a lot of interactions with the Linux system are carried out through a Command Shell – or Terminal Window.

### Starting the Terminal

➢ You can start the Terminal by clicking the icon that looks like this on the menu on the left side of the screen: `[Icon Image]`.

You can enter text commands into this Shell to execute them on the system.

### Viewing and Navigating Directories

➢ Type the following command and press Enter:

```bash
ls
```

This will execute the “List” command and will list the files and folders (directories) in the current directory (the home directory that is the default file path when the shell opens).

To change to a different directory, use the `cd` command:

➢ Type the following command to change to the "Downloads" directory:

```bash
cd Downloads
```

**Note**: The commands in Linux are case-sensitive, so if you type in `cd downloads`, it will respond with `No such file or directory`.

The Linux shell has a shortcut to speed up command entry. If you start typing a parameter and press the `Tab` key, the system will attempt to autocomplete this for you.

For example:

- `cd D` and press `Tab` will show “Desktop” “Documents” and “Downloads” as options.
- Add an `o`, and you'll just see “Documents” and “Downloads”.
- Add a subsequent `w` so you have `cd Dow`, then press `Tab` again, and the system will autocomplete this to `cd Downloads/` for you.

➢ Type `ls` in this new folder.

You should see one file, which is an installer for Visual Studio Code that was previously downloaded and installed into the environment for you.

➢ You can run this program simply by typing `code` into the shell and pressing Enter, or by clicking on the corresponding icon on the left side menu.

➢ Close it for now though, as we will not be using it today.

### Additional Useful Commands

If you want more information about any Linux shell command, you can type `man` (for “Manual”) followed by the name of the command:

➢ Try typing:

```bash
man ls
```

This will show you all of the options you can use with the `ls` command. You can scroll with the cursor keys or the spacebar.

➢ Press `q` to exit the man page.

Another useful command is `pwd`, which shows the directory you are currently in:

➢ Type:

```bash
pwd
```

This should produce `/home/ros/Downloads`.

- “/home” is where the user home directories are stored by default in Linux.
- “ros” is the default user that has been set up for these bootable drives.

Note: While Windows uses the “\” symbol to separate folders in a path, Linux uses the “/” symbol.

To return to a directory one level up, type:

```bash
cd ..
```

➢ Type the following:

```bash
pwd
```

You should now see `/home/ros`.

The `~` symbol can be used as an abbreviation for `/home/ros`, the default home directory for the current user.

For example, you can type `cd ~` from anywhere within the file system to return to `/home/ros`.

You can also type `cd ~/Downloads` to go straight to the user Downloads directory from anywhere.

### Making Workspace Directories

To create a new directory, use the `mkdir` command:

➢ Type the following command (ensure that you are in the `/home/ros` folder before you do this):

```bash
mkdir ros2_ws
```

This is short for “ROS2 workspace” and is where we will be creating our ROS 2 packages later on.

➢ Type `ls`. You should now see this new directory listed.

➢ Create one more directory by typing:

```bash
mkdir ros2_ws/src
```

This will create a “src” directory within the ROS workspace directory we created previously.

**Note**: A shortcut to create both of these at once is to just type:

```bash
mkdir -p ros2_ws/src
```

The `-p` option creates parent directories as needed, allowing a longer path to be created in one go.

### Some Other Useful Commands

- `mv` for “Move”, which allows files or folders to be relocated.
- `cp` for “Copy”, which allows files or folders to be copied (i.e., replicated at a different location).
- `rm` for “Remove”, which allows files or folders to be deleted (note this is irreversible!).
- `touch` to create a new empty file.

**Info**: If you want to go into more depth on features of Linux and its command shell, you can complete the following short online course outside of class: [The Construct - Linux for Robotics](https://www.theconstruct.ai/robotigniteacademy_learnros/ros-courses-library/linux-for-robotics/). Note that you will need to create an account with “The Construct” to access the course, but they have a wealth of excellent ROS and programming resources, so I would highly recommend this.

## ROS2 Installation

We will now use the Linux shell to install the ROS 2 system. Note that there are a number of different versions of ROS available that have been released over time, including a “ROS 2” version, which introduces some more significant changes. We will be using the latest “Long Term Support” release version of ROS 2, which is ROS 2 Humble (or Humble Hawksbill, which is the full name of the release).

**Note**: ROS normally follows an annual release cycle for updates, with a “Long Term Support” or LTS version released every second year. The LTS versions are the ones that are used for most development activities due to the longer-term support offered (generally 5 years), unless a particular feature is needed that is only available in a newer non-LTS version.

To carry out the installation, we will be using the command line Linux “Advanced Package Tool”, which is a very useful tool we will use to install a whole range of different software as we progress through the course. To run this, we use `sudo apt` followed by suitable parameters, e.g., `sudo apt update` to update the package index (which is generally run before any other apt commands in a particular session).

**Note**: `sudo`, which means “Super User DO”, is running the subsequent command with elevated privileges. The first time you do this, it will ask for a password; the password for this USB Linux install is just `ros`.

➢ Type the following:

```bash
sudo apt update
```

To complete the ROS 2 installation, use the guide provided on the main ROS 2 documentation at: [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

➢ Go to the webpage above from a browser within Linux. You can copy and paste the commands directly into the command line.

The Firefox browser is available to run from the left side menu, and you can access the site through this.

You can copy text by either (i) using `Ctrl-C` like in Windows or (ii) by moving the mouse over the green window on the site, and when the Copy icon appears, click on it.

To paste it into the terminal, you will either need to right-click and select paste, or just press the middle mouse button when over the terminal window.

➢ Carefully follow the instructions in the `Setup Sources` section.

➢ Follow the instructions in the `Install ROS2 Packages` section to install the Desktop Install (Recommended) and Development tools. Let your tutor know if you have any problems.

**Note**: The `Y/n` prompts are case-sensitive.

➢ Follow the instructions in the `Environment setup` section.

➢ Test out the installation by following the commands in the `Try some examples` – `Talker-listener` section.

Show your tutor that you can successfully run the talker and listener.

➢ You can stop the talker and listener processes by typing `Ctrl-C` in the appropriate terminal window.

## Configuring the ROS 2 Environment

➢ Go to: [Configuring ROS 2 Environment

](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html).

➢ Read through the Background information.

➢ Skip ahead to the Tasks section and carry out the following tasks by copying and pasting the commands (ensure that you are using the commands in the Linux tab):

1. **Source the Setup files**
2. **Add sourcing to your shell startup script**

The above will ensure that the ROS 2 environment is automatically set up properly in every terminal window you open.

➢ Carry out the following tasks by copying and pasting the commands (ensure that you are using the commands in the Linux tab):

3. **Check environment variables**
   1. **The ROS_DOMAIN_ID variable** – Replace `<your_domain_id>` with the number `0`.
   2. **The ROS_LOCALHOST_ONLY variable**

Let’s check that everything has been set properly:

➢ Type:

```bash
printenv | grep -i ROS
```

Check the environment variables, particularly DOMAIN_ID and LOCALHOST_ONLY values:

- `ROS_DOMAIN_ID=0`
- `ROS_LOCALHOST_ONLY=1`

The `echo “….” >> ~/.bashrc` commands used in this section insert the text in the quotes into the `.bashrc` file in the root folder. The `.bashrc` file is a configuration file for the Bash shell used to set up the user’s shell environment according to their preferences. We have used it to save us from having to remember to run certain configuration commands every time we open a new terminal window.

### To Check the .bashrc File

➢ Click on the Files icon on the left-hand menu bar (second from top).

➢ Click on the top left button with 3 lines, and tick the option “Show Hidden Files”.

➢ Browse to the ‘Home’ folder where you should now see the `.bashrc` file.

➢ Double click on the file to open it and scroll down to the bottom. You should see the commands to source and set the domain variables from last week and above.

## Configuring a New ROS Workspace

We will now configure a new ROS workspace in the folder we created earlier. To do this, we will be using the colcon tools that we will install, which are the official automated build tools for ROS 2 (based on CMake).

➢ Go into your new ROS 2 workspace directory by executing the following at the command line:

```bash
cd ~/ros2_ws
```

➢ In your browser, go to: [Colcon Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html).

➢ Skip to and read through the Basics information.

➢ Skip ahead to the Add some sources section and carry out the following tasks (ensure that you are using the commands in the Linux tab and that you are in the `~/ros2_ws` folder):

- **Add some sources**
- **Build the workspace**
- **Run tests**
- **Source the environment**
- **Try a demo**

**Note**: If the system ‘hangs’ while doing the build, it is probably because the system is trying to process multiple jobs at once and is unable to handle the load. You can overcome this issue by forcing colcon to carry out the jobs sequentially by using the following command:

```bash
colcon build --symlink-install --executor sequential --event-handlers console_direct+
```

Show your tutor that you can successfully run the demo publisher and subscriber.

The `Build the Workspace` task will create three additional directories in the workspace folder: “build”, “install”, and “logs”. The “src” directory is the only one you should do anything with (this is where you will create your packages and code). Logs are stored in the “logs” directory, and the “build” and “install” directories are used by the build system and should not be touched.

The workspace is now ready for the creation of ROS 2 packages, but we will look into how to do more of that in the next tutorial.

We will stop there for this week, but you may want to start exploring some of the other ROS 2 information at: [ROS 2 Basic Concepts](https://docs.ros.org/en/humble/Concepts/Basic.html) in your own time. In particular, you might want to look at the concepts from Nodes to Parameters ahead of next week.

**Make sure you have shown your tutor that you can successfully run the two examples before you leave.**

