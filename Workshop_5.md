# ENS5204 Real-Time Embedded Systems

## Workshop 5 – Writing Code for the ROSbot and Communication via Topics

### Overview

In the previous workshop, you covered using the Nav2 stack to get the robot to autonomously navigate around the simulated environment. In this workshop, you will look at writing code that will work with the devices on the ROSbot as well as communicating remotely via topics.

➢ **Workshop Notation**: Anything starting with the ‘arrow bullet’ (like this line) indicates an action that you need to carry out.

### Creating Your Own Workspace and Code

➢ Go to the Husarion tutorial web page on **Creating Nodes - Messages**: [Creating Nodes - Messages](https://husarion.com/tutorials/ros2-tutorials/2-creating-nodes-messages/)
- Read through the information until the point of creating a new package.
➢ Now create a new package but name it `rtes_xx_pkg` where `xx` is the number of your given USB (instead of `tutorial_pkg` as we already have that).

  ```bash
  cd ~/ros2_ws/src
  ros2 pkg create --build-type ament_cmake --node-name my_first_node --dependencies rclcpp rclcpp_components std_msgs rclpy rclpy rclpy
  rtes_xx_pkg
  ```

➢ You may get an error message about the License file. Open the `package.xml` file and change the license setting to `Apache-2.0`. You could also copy it from the `package.xml` in the previously used `tutorial_pkg`.
➢ Follow the steps in the tutorial to create a subscriber and testing it using Gazebo.

  - **Note**: Remember to always use `rtes_xx_pkg` instead of `tutorial_pkg`.
  - **Note 2**: Make sure you read the expandable sections on **Hints** and **Code Explained** to ensure you understand what is being done.
  - **Note 3**: The robot (simulation and real) uses a different topic to what is on the website code. Use the topic `/camera/color/image_raw`.

➢ Once Gazebo is open, use the `TELE_KEY` (shortcut for `teleop_twist_keyboard` function) to move the robot around the world (particularly into shadow areas) and see how the node output varies.

✅ **Demonstrate this functionality to the tutor once completed for part of the workshop marks.**

➢ Continue following the steps till the end of the tutorial (creating a publisher, etc.)

  - **Note**: Remember to always use `rtes_xx_pkg` instead of `tutorial_pkg`.

✅ **Demonstrate this functionality (publishing to topic) to the tutor once completed for part of the workshop marks.**

### Using GitLab

Now that you know your code works using the Gazebo simulation, it’s time to try it out on the real robot. To do this, you will need to transfer your code over – we will use GitLab to manage this process.

To transfer the code, we will first upload it to a Git repository using GitLab.

You should have received an email inviting you to become part of the `ENS5204-2024` repository. Ensure that you have set up this account and use that username and email in the steps below.

#### How to Create an SSH Key for GitLab

➢ Follow these steps to generate an SSH key pair and add it to your GitLab account.

**Step 1: Check for Existing SSH Keys**

Before creating a new SSH key, check if you already have one:

1. Open a terminal.
2. Run the following command to check if an SSH key already exists:

   ```bash
   ls -al ~/.ssh
   ```

If you see files named `id_rsa.pub` or `id_ed25519.pub`, you already have an SSH key.

**Step 2: Generate a New SSH Key**

If you don’t have an existing SSH key or want to create a new one, follow these steps:

1. Run the following command in the terminal, replacing `your_email@example.com` with your email address:

   For ED25519 algorithm:

   ```bash
   ssh-keygen -t ed25519 -C "your_email@example.com"
   ```

   If your system doesn't support the ed25519 algorithm, you can use RSA instead:

   ```bash
   ssh-keygen -t rsa -b 4096 -C "your_email@example.com"
   ```

2. When prompted to enter a file in which to save the key, press **Enter** to accept the default location.
3. You will then be asked to enter a passphrase. You can enter a secure passphrase or leave it empty for no passphrase.

**Step 3: Add Your SSH Key to the SSH Agent**

To ensure your SSH key is automatically used when interacting with GitLab, you need to add it to the SSH agent:

1. Start the SSH agent in the background:

   ```bash
   eval "$(ssh-agent -s)"
   ```

2. Add your SSH private key to the SSH agent:

   For ED25519:

   ```bash
   ssh-add ~/.ssh/id_ed25519
   ```

   For RSA:

   ```bash
   ssh-add ~/.ssh/id_rsa
   ```

**Step 4: Add the SSH Key to Your GitLab Account**

1. Copy your SSH public key to your clipboard:

   For ED25519:

   ```bash
   cat ~/.ssh/id_ed25519.pub
   ```

   For RSA:

   ```bash
   cat ~/.ssh/id_rsa.pub
   ```

   Then, highlight the key and copy it to your clipboard.

2. Log in to your GitLab account.
3. Go to **Profile Settings > SSH Keys**.
4. In the **Key** field, paste your SSH key.
5. Optionally, you can give the key a title for easy identification.
6. Click **Add key** to save your SSH key.

**Step 5: Test Your Connection**

Finally, verify that your SSH key is working:

1. Run the following command to test the connection:

   ```bash
   ssh -T git@gitlab.com
   ```

2. If successful, you should see a message like:

   ```
   Welcome to GitLab, @your_username!
   ```

   - **Note**: This step may not work due to ECU network security if done on campus!

Your SSH key is now set up and ready to use with GitLab!

### Using GitLab to Transfer Code

➢ Firstly, make sure you have the latest version of Git installed:

  ```bash
  sudo apt-add-repository ppa:git-core/ppa
  sudo apt-get update
  sudo apt-get install git
  ```

Now, we shall upload the `rtes_xx_pkg` you created to a GitLab repository.

➢ To do this, make sure you are in the base directory of your `rtes_xx_pkg` (`~/ros2_ws/src/rtes_xx_pkg`) in a Terminal window, and then execute the following set of commands (in all cases replace `xx` with your USB number):

  ```bash
  git init
  git config --global user.name "<Your name>"
  git config --global user.email johndoe@example.com
  git add CMakeLists.txt package.xml src/ include/
  git commit -m "<Your name> Workshop 5 commit"
  git remote add origin https://gitlab.com/ecueng/ens5204-2024.git
  git branch -M <Your name>
  git push -uf origin <Your name>
  ```

**Note**: This next part needs to be coordinated by the tutor so not everyone is trying to load and run things on the robot at the same time. If you are waiting for your turn, you can move on to the next section.

To connect to the robot, we can use `ssh` (as we did in Workshop 3).

➢ If you have not connected to Husarnet, then you need to do it now (see Workshop 3).
➢ Power on the robot and wait until the second red LED on the back has lit up, then type `ssh husarion@rosbot` into a Terminal window. The password for the robot is also `husarion`.
➢ Once you are connected to the robot, you will need to clone your code from the Git repository you have uploaded it to:

  ```bash
  cd ~/ros2_ws/src
  git clone -b <Your_name> https://gitlab.com/ecueng/ENS5204-2024.git rtes_xx_pkg
  ```

➢ Next, build/compile your package on the robot:

  ```bash
  cd ~/ros2_ws
  colcon build –packages-select rtes_xx_pkg
  ```

➢ If it builds successfully, source the workspace setup file:

  ```bash
  source ~/ros2_ws/install/setup.bash
  ```

➢ Then run the node:

  ```bash
  ros2 run rtes_xx_pkg my_first_node
  ```

➢ Monitor the topic `/brightness` as before to see the output. You can try changing the light level by covering the camera or shining a light into it.

**Note**: If you need to make any changes to your code:

1. Just make the changes on your main development system and save them.
2. Then execute:

   ```bash
   git commit -a -m "<Your name> Workshop 5 commit"
   git push
   ```

   - Make sure you are in the base directory of your package when running these commands.

3. Then on the robot, also change into the base directory of your package and enter `git pull`, and this should synchronize the changes.

✅ **Demonstrate your code working on the robot to the tutor once completed for part of the workshop marks.**

### Communicating with the ROSbot Using Topics

➢ If you haven’t already done so, ensure that your device (USB / laptop) is connected to Husarnet (as shown in the last section of Workshop 3) and you can `ssh` into the ROSbot and control it from a terminal in the ROSbot.

We are now going to test your ability to connect to the ROSbot and control it from your own device over the network. However, before we do that, we need to ensure that there are some ‘safety’ measures to ensure that multiple users are not accidentally sending commands to the ROSbot.

The following measures are not foolproof (i.e., can’t protect against human carelessness) but do provide a mechanism to enable you to work on your devices and not accidentally send commands to the ROSbot. For this, we are going to utilize the `ROS_LOCALHOST_ONLY` parameter we set in Workshop 1 and use some aliases (which we covered in Workshop 3).

There is also one part of the configuration that needs to be added for the topics to become visible – configuring the right RMW (ROS2 Middleware) setting for the correct DDS implementation. For now, we shall be using FastDDS as that should have been installed by default when you connected to Husarnet.

However, to ensure that all is up to date and to ensure the address lists have been set properly, we shall re-do the following:

➢ **Install/upgrade the right version of the FastDDS middleware** using the command:

  ```bash
  sudo apt install ros-humble-rmw-fastrtps-cpp
  ```

Next, we shall install and use the Husarnet-DDS utility to generate the correct DDS XML configuration file. The instructions below can be found on the Husarnet website page: [Husarnet-DDS](https://husarnet.com/docs/ros2/husarnet-dds/) and copied from there.

➢ **Install the Husarnet DDS utility**

  ```bash
  RELEASE="v1.3.5"
  ARCH="amd64"
  sudo curl -L https://github.com/husarnet/husarnet-dds/releases/download/$RELEASE/husarnet-dds-linux-$ARCH -o /usr/local/bin/husarnet-dds
  sudo chmod +x /usr/local/bin/husarnet-dds
  ```

➢ **Next, we need to set up the environment and generate the XML file**:

  ```bash
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE=/var/tmp/husarnet-fastdds-simple.xml
  husarnet-dds singleshot
  ```

➢ You also need to add the following lines to set the environment variables at the end of the `.bashrc` file to ensure the environment is set in new terminal windows:

  ```bash
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE=/var/tmp/husarnet-fastdds-simple.xml
  ```

➢ You should try to view the XML file in a text editor (without changing anything!). **Note**: `/var` is a hidden folder in the root of the drive. You will see the first address is the IPv6 address of your device. You will also see in the `<initialPeersList>` part all the other device addresses. The ROSbot address is the one ending with `:738b` (in this current Husarnet network). Also note that UDP is the broadcast transport protocol (equivalent to TCP).

#### Controlling Robot Access Using the `ROS_LOCALHOST_ONLY` Environment Variable

We had previously set the `ROS_LOCALHOST_ONLY` environment variable to `1` (True) to ensure that you would not interfere with one another in the lab. This would make the various topics only visible on that particular device. However, to communicate with the robot, we need to have topics visible.

➢ Open up the `.bashrc` file in the root (`~`) folder and add the following two aliases in the alias section:

  ```bash
  alias ROBOT_COMM='export ROS_LOCALHOST_ONLY=0'
  alias LOCAL_COMM='export ROS_LOCALHOST_ONLY=1'
  ```

  - **Note**: If copying from the worksheet, note that Word has changed the apostrophe character used – and that makes a difference! Use the same apostrophe as in the other alias commands in the `.bashrc` file.

The alias `ROBOT_COMM` will allow your device to communicate with the ROSbot via Husarnet, while the alias `LOCAL_COMM` will stop all communication with other devices.

Now to test this concept:

➢ Get the tutor to run the test message command on the ROSbot.
➢ Open a new terminal window and run the following commands:

  ```bash
  LOCAL_COMM
  ROBOT_COMM
  ros2 topic echo /testmsg
  ```

➢ You should now see a ‘greeting’ from the ROSbot.
➢ Stop the topic echo (`Ctrl-C`), run the command `LOCAL_COMM` then re-run the topic echo to see if the message comes through.
➢ Open a second terminal window and test to see what happens when one has run `LOCAL_COMM` and another has `ROBOT_COMM` setting (or both the same) and see if you can control the robot from your device.

✅ **Show the monitoring of the test message from the robot to the tutor once completed for part of the workshop marks.**

➢ To test that it works both ways, run `TELE_KEY` and see if you can control the robot from your device.

**Note**: The communication is **NOT** exclusive. You will need to coordinate with the rest of the class to see who is controlling the ROSbot.

Now let’s see if you can monitor the ROSbot’s camera topic using the node you created **FROM YOUR OWN DEVICE**.

➢ Ensure that Gazebo is **NOT** running (as the simulation uses the same topic names).
➢ Run the alias `ROBOT_COMM`.
➢ Run the command `ros2 topic list` and check if you can see the camera topic on your device terminal.
➢ If you can see it, then run your node to see if it can display the brightness of the camera feed.

✅ **Demonstrate the above to the tutor once completed for part of the workshop marks.**

~ End of Workshop ~
