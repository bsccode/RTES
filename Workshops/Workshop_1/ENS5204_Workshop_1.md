# ENS5204 Real-Time Embedded Systems

## Workshop 1 – Linux Familiarisation and ROS 2 Installation

### Overview

In this first workshop you will be introduced to the Linux desktop environment, and guided through the process to install and perform initial configuration for the ROS2 (Robot Operating System 2) that we will be using as the basis for the project work in this unit. You will be provided with a bootable USB drive that will start an Ubuntu Linux 22.04 environment on the lab PCs when you boot from it (just plug it in and start, or restart, the PC).

➢ **Workshop Notation**: Anything starting with the ‘arrow bullet’ (like this line) indicates an action that you need to carry out.

### Linux Environment

Linux is a free open-source operating system with similar features and capabilities to Windows, but also some significant differences. This is the OS that best supports ROS, so we will be using it as our main development environment for the semester. The bootable USB you will be provided with should enable you to run it on any computer you can configure to boot from USB. Note, however, if you lose the drive you will be required to replace it at your cost, and will also lose your personal environment, so look after it!

The Linux Desktop environment has a somewhat different layout to the Windows environment you might be used to, but you should be able to get to grips with the basic functionality reasonably easily as the controls are fairly similar. One key difference, however, particularly when using ROS, is that a lot of interactions with the Linux system are carried out through a Command Shell – or Terminal Window.

➢ You can start the Terminal by clicking the icon that looks like this on the menu on the left side of the screen:

![Terminal](terminal.png "terminal-emulator-icon")


You can enter text commands into this Shell to execute them on the system.

#### Viewing and navigating directories (folders)

➢  **Type**:

  ```bash
  ls
  ```

  and press Enter.

This will execute the “List” command and will list the files and folders (directories) in the current directory (the home directory that is the default file path when the shell opens).

If you want to change to a different directory, you can use the `cd` command for “Change Directory”, but this time you will need to provide a parameter to tell the system which directory to change to.

➢  **Try**:

  ```bash
  cd Downloads
  ```

  to change to the “Downloads” directory.

Note that the commands in Linux are case-sensitive, so if you type in `cd downloads` it will respond `No such file or directory`.

Also note that the Linux shell has a shortcut to speed up command entry, if you start typing a parameter and press the Tab key, the system will attempt to autocomplete this for you, if there is more than one option, the set of options will be displayed below the command line and more characters will be needed, e.g. if you type `cd D` and press Tab you will see “Desktop” “Documents” and “Downloads” as options, if you add an `o` you will just see “Documents” and “Downloads” and if you add a subsequent `w` so you have `cd Dow` and then press Tab again, then the system will autocomplete this to `cd Downloads/` for you.

➢  **Type**:

  ```bash
  ls
  ```

  in this new folder.

You should just see one file, which is an installer for Visual Studio Code that was previously downloaded and installed into the environment for you.

➢  You can run this program simply by typing:

  ```bash
  code
  ```

  into the shell and pressing Enter, or by clicking on the corresponding icon on the left side menu.

➢  Close it for now though, we will not be using it today.

**Note**: if you want more information about any Linux shell command you can just type `man` (for “Manual”) followed by the name of the command.

➢ **Try typing**:

  ```bash
  man ls
  ```

  now.

This will show you all of the options you can use with the `ls` command, e.g. add `-d *` to list only directories. You can scroll with the cursor keys or the spacebar.

➢  **Press** `q` to exit the man page.

Another useful command is `pwd`, which will show you which directory you are currently in.

➢  **Type in**:

  ```bash
  pwd
  ```

  now.

This should produce `/home/ros/Downloads`.

`/home` is where the user home directories are stored by default in Linux and `ros` is the default user that has been setup for these bootable drives. Note that while Windows uses the `\` symbol to separate folders in a path, Linux uses the `/` symbol, just one of the minor differences to get used to!

If you want to return to a directory one level up, you can just type `cd ..`.

➢  **Type in**:

  ```bash
  cd ..
  ```

  now.

➢  Then type in:

  ```bash
  pwd
  ```

You should now see `/home/ros`.

Note that the `~` symbol can be used as an abbreviation for `/home/ros`, i.e. the default home directory for the current user, so if you type `cd ~` from anywhere within the file system, it will return you to `/home/ros`. You can also type `cd ~/Downloads` to go straight to the user Downloads directory from anywhere, for example.

#### Making workspace directories

To create a new directory you can use the `mkdir` command.

➢  **Type**:

  ```bash
  mkdir ros2_ws
  ```

  (ensure that you are in the `/home/ros` folder before you do this).

This is short for “ROS2 workspace” and is where we will be creating our ROS2 packages later on.

➢  **Type**:

  ```bash
  ls
  ```

You should now see this new directory listed.

➢  **Create one more directory by typing**:

  ```bash
  mkdir ros2_ws/src
  ```

This will create a `src` directory within the ROS workspace directory we created previously.

Note, a shortcut to create both of these at once is to just type:

```bash
mkdir -p ros2_ws/src
```

the `-p` option creates parent directories as needed allowing a longer path to be created in one go.

Some other useful commands are:

- `mv` for “Move”, which allows files or folders to be relocated;
- `cp` for “Copy”, which allows files or folders to be copied (i.e. replicated at a different location);
- `rm` for “Remove”, which allows files or folders to be deleted (note this is irreversible!); and
- `touch` to create a new empty file.

**Info**: If you want to go into more depth on features of Linux and its command shell, you can complete the following short online course outside of class: [The Construct - Linux for Robotics](https://app.theconstructsim.com/#/Course/59). Note that you will need to create an account with “The Construct” to access the course, but they have a wealth of excellent ROS and programming resources, so I would highly recommend this.

### ROS2 Installation

We will now use the Linux shell to install the ROS2 system. Note that there are a number of different versions of ROS available that have been released over time, including a “ROS2” version, which introduces some more significant changes. We will be using the latest “Long Term Support” release version of ROS2, which is ROS2 Humble (or Humble Hawksbill, which is the full name of the release).

**Note**: ROS normally follows an annual release cycle for updates, with a “Long Term Support” or LTS version released every second year, and the LTS versions are the ones that are used for most development activities due to the longer term support offered (generally 5 years), unless a particular feature is needed that is only available in a newer non-LTS version.

To carry out the installation we will be using the command line Linux “Advanced Package Tool”, which is a very useful tool we will use to install a whole range of different software as we progress through the course. To run this, we use `sudo apt` followed by suitable parameters, e.g. `sudo apt update` to update the package index (which is generally run before any other apt commands in a particular session).

**Note**: `sudo`, which means “Super User DO”, is running the subsequent command with elevated privileges – the first time you do this it will ask for a password, the password for this USB Linux install is just `ros`.

➢  **Type**:

  ```bash
  sudo apt update
  ```

  now.

To complete the ROS2 installation, use the guide provided on the main ROS2 documentation at: [Installing ROS 2 via Debian Packages](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

➢  Go to the webpage above from a browser within Linux. You can copy and paste the commands directly into the command line.

The Firefox browser is available to run from the left side menu, and you can access the site through this.

You can copy text by either (i) using `Ctrl-C` like in Windows or (ii) by moving mouse over the green window on the site, and when the Copy icon appears, click on it.

To paste it into the terminal you will either need to right click and select paste, or just press the middle mouse button when over the terminal window.

➢  Carefully follow the instructions in the **Setup Sources** section.

➢  Follow the instructions in the **Install ROS2 Packages** section to install the Desktop Install (Recommended) and Development tools. Let your tutor know if you have any problems.

  - **Note**: The ‘Y/n’ prompts are case-sensitive

➢  Follow the instructions in the **Environment setup** section

➢  Test out the installation by following the commands in the **Try some examples** – **Talker-listener** section.

Show your tutor that you can successfully run the talker and listener.

➢  You can stop the talker and listener processes by typing in `Ctrl-C` in the appropriate terminal window.

### Configuring the ROS2 Environment

➢  Go to: [Configuring your ROS 2 environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)

➢  Read through the Background information

➢  Skip ahead to the **Tasks** section and carry out the following tasks by copying and pasting the commands (ensure that you are using the commands in the Linux tab).

  - **1 Source the Setup files**

  - **2 Add sourcing to your shell startup script**

The above will ensure that the ROS2 environment is automatically setup properly in every terminal window you open.

➢  Carry out the following tasks by copying and pasting the commands (ensure that you are using the commands in the Linux tab).

  - **3 Check environment variables**

  - **3.1 The ROS_DOMAIN_ID variable**

    - **Note**: Replace `<your_domain_id>` with the number `0`

  - **3.2 The ROS_LOCALHOST_ONLY variable**

Let’s check that everything has been set properly:

➢  **Type**:

  ```bash
  printenv | grep -i ROS
  ```

  and press Enter.

➢  Check the environment variables, particularly `DOMAIN_ID` and `LOCALHOST_ONLY` values

  - `ROS_DOMAIN_ID=0`

  - `ROS_LOCALHOST_ONLY=1`

The `echo "…." >> ~/.bashrc` commands used in this section inserts the text in the quotes into the `.bashrc` file in the root folder. The `.bashrc` file is a configuration file for the Bash shell that used to set up the user’s shell environment according to their preferences. We have used it to save us from having to remember to run certain configuration command every time we open a new terminal window.

- To check the `.bashrc` file

  - Click on the **Files** icon on the left-hand menu bar (second from top)

  - Click on the top left button with 3 lines, and tick the option **“Show Hidden Files”**

  - Browse to the ‘Home’ folder where you should now see the `.bashrc` file.

  - Double click on the file to open it and scroll down to the bottom. You should see the commands to source and set the domain variables from last week and above.

### Configuring a new ROS workspace

We will now configure a new ROS workspace in the folder we created earlier. To do this we will be using the `colcon` tools that we will install, which are the official automated build tools for ROS2 (based on CMake).

➢  Go into your new ROS2 workspace directory by executing:

  ```bash
  cd ~/ros2_ws
  ```

  at the command line

➢  In your browser, go to: [Colcon Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

➢  Skip to and read through the **Basics** information

➢  Skip ahead to the **Add some sources** section and carry out the following tasks (ensure that you are using the commands in the Linux tab and that you are in the `~/ros2_ws` folder):

  - **Add some sources**

  - **Build the workspace**

  - **Run tests**

  - **Source the environment**

  - **Try a demo**

**Note**: If the system ‘hangs’ while doing the build, it is probably because the system is trying to process multiple jobs at once and is unable to handle the load. You can overcome this issue by forcing `colcon` to carry out the jobs sequentially by using the following command:

```bash
colcon build --symlink-install --executor sequential --event-handlers console_direct+
```

Show your tutor that you can successfully run the demo publisher and subscriber.

The ‘Build the Workspace’ task will create three additional directories in the workspace folder, `build`, `install` and `logs`. The `src` directory is the only one you should do anything with (this is where you will create your packages and code), logs are stored in the `logs` directory, and the `build` and `install` directories are used by the build system and should not be touched.

The workspace is now ready for the creation of ROS2 packages, but we will look into how to do more of that in the next tutorial.

We will stop there for this week, but you may want to start exploring some of the other ROS2 information [Basic ROS 2 concepts](https://docs.ros.org/en/humble/Concepts/Basic.html) in your own time. In particular you might want to look at the concepts from Nodes to Parameters ahead of next week.

Make sure you have shown your tutor that you can successfully run the two examples before you leave.
