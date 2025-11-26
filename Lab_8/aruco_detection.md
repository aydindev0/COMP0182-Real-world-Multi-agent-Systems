# COMP0182 (Multi-Agent Systems): Lab Sheet 3

##NB: You can find goal_pose.py in: COMP0182-Multi-Agent-Systems/turtlebot3_burger_auto_navigation/auto_navigation/scripts

https://github.com/narsimlukemsaram/COMP0182-Multi-Agent-Systems/
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

## Task 1 (Mandatory): Install and connect the Remote PC (your Laptop) and Target Robot (TurtleBot3 Burger)
Follow **Chapter 3: Quick Start Guide** from official tutorial from
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup

**Important**: Make sure you select the correct version of ROS: **Noetic**

![TurtleBot3](/Week_03/img/noetic.png)

### Key points 
These are the points that you need to pay attention to while following the offcial tutorial. For the steps that are not mentioned here, just follow the content from the tutorial.

**3. 1. PC Setup**

You can skip the introduction video first, it talks about setting up a TurtleBot3 in the Gazebo simulation world. You can go back and watch this after finishing this lab's work.

**1. 1. 1. Download and Install Ubuntu on PC**: 

You should have finished these steps in previous lab sessions. If yes, skip this step and go to the "1. 1. 2 Install Dependent ROS Packages" sub-section.
If not please do it.

**1. 1. 2. Install ROS on Remote PC**: 

You should have finished these steps in previous lab sessions. If yes, skip this step and go to the "1. 1. 3 Install Dependent ROS Packages" sub-section.
If not please do it.

**3. 2. SBC Setup**

**0. 2. 2. Download TurtleBot3 SBC Image**

Download the correct image file for your hardware and ROS version. Noetic version images are created based on Ubuntu 20.04. 

Please Download the "Raspberry Pi 4B (2GB or 4GB)" ROS Noetic image.

**0.2.5. Resize the Partition**

[Optional] Be aware of selecting an incorrect disk or a partition, partitioning a system disk of your PC may cause a serious system malfunction. You can skip this step. 

**0.2.6. Configure the WiFi Network Setting**

Boot Up the Raspberry Pi:

- Connect the HDMI cable of the monitor to the HDMI port of Raspberry Pi.

- Connect input devices to the USB port of Raspberry Pi.

- Insert the microSD card.

- Connect the power (either with USB or OpenCR) to turn on the Raspberry Pi.

- Login with ID **ubuntu** and PASSWORD **turtlebot**.

Just follow the instructions. 

If you are unable to go to this folder on your command prompt: "cd /media/$USER/writable/etc/netplan", you can go to this folder: "cd /etc/netplan" and follow the instructions. 

**3. 3. OpenCR Setup**

Just follow the instructions.

**3. 4. Hardware Assembly**

This can be skipped as the turtlebots have already been built. 

**3. 5. Bringup**

Just follow the instructions.

**3. 6. Basic Operation**

Just follow the instructions.

Expected output:

Teleoperating of TurtleBot3 from your laptop.

Hints: 3 terminals from your PC, one for SSH, one for ROS (roscore), and one for teleoperation.

## Task 2: Finding single and multiple ArUco Markers using the camera

In this task, we are gonna show you how to track ArUco markers and estimate their pose using a USB camera in Python 3 on Ubuntu 20.04 on ROS Noetic. **Use your own PC for this part, not the turtlebot**

**Installing the required package**:

So first let's install the required packages:

Open a terminal on your Ubuntu 20.04. Type this command:

```bash
sudo apt-get install ros-noetic-usb-cam ros-noetic-aruco-ros
```

**Quick Note**: If you need to install aruco-ros separately later, you can use:
```bash
sudo apt install ros-noetic-aruco-ros
```

**Setting up the workspace**:

Before launching the files, you need to set up your catkin workspace properly:

Download this package. You may find it simpler to clone the entire repository and move the directory once it is on your local machine.

https://github.com/narsimlukemsaram/COMP0182-Multi-Agent-Systems/tree/main/turtlebot3_burger_auto_navigation/

Create your workspace and add a src directory in which packages are to be placed. 

```bash
mkdir catkin_ws
cd catkin_ws
mkdir src
```

1. Move the package into `catkin_ws/src`
2. Build the workspace. Make sure you are not in the /src directory.

```bash
cd ~/catkin_ws
catkin_make
```
3. Source the workspace:
```bash
source devel/setup.bash
```
**Understanding the launch files**

1. ``usb_cam_stream_publisher.launch``: USB image publisher
   
2. ``aruco_marker_finder.launch``: single ArUco Marker finder
   
3. ``multiple_aruco_marker_finder.launch``: multiple ArUco Markers finder

**Run ROS Master**:

Open the first terminal, and run this command to execute the ROS master:

```bash
roscore
```

Leave this terminal open at all times. If rosmaster is not active then you will not successfully launch any other packages including the turltebot3 bringup. 

**Build or compile your workspace**:

Open the second terminal, and run this command to compile your workspace:

**Launch Files**:
```bash
catkin_ws 
catkin_make
```

**Camera Setup (Important)**:

Before publishing images, you may need to disable autofocus on your camera for better marker detection. If using `/dev/video0`, run:

```bash
sudo v4l2-ctl -d /dev/video0 --set-ctrl=focus_automatic_continuous=0
```

**Publishing Images from Camera and Estimating the Pose**:

Open the third terminal, and go to this folder:

```bash
cd catkin_ws/src/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder/launch/
```

and run this command to publish USB camera images and estimate the pose of the ArUco Marker:

```bash
catkin_ws/src/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder/launch $ roslaunch usb_cam_stream_publisher.launch
```

**Finding the ArUco Marker**:

Open the fourth terminal, and go to this folder:

```bash
cd catkin_ws/src/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder/launch/
```

and run this command to find the ArUco Marker:

```bash
catkin_ws/src/turtlebot3_burger_auto_navigation/auto_aruco_marker_finder/launch $ roslaunch aruco_marker_finder.launch markerId:=701 markerSize:=0.05
```

**Note**: You can easily change the marker ID and size by passing arguments to the launch command instead of modifying the YAML file. For example, if you want to detect marker ID 502 with size 0.15m using `/dev/video0`:

```bash
roslaunch aruco_marker_finder.launch markerId:=502 markerSize:=0.15 video_device:=/dev/video0
```

**Open Rqt GUI**

Open the fifth terminal, and run this command to see the results:

```bash
rosrun rqt_gui rqt_gui
```

To visualize the camera feed with detected markers:
1. Go to **Plugins -> Visualization -> Image View**
2. Select the topic `/usb_cam/image_raw`

**Pose of ArUco Marker**

Open the sixth terminal, and run this command to see the pose of the marker:

```bash
rostopic echo /aruco_single/pose
```
The rqt_gui window will allow you to visualise the arena so that you can ensure the aruco markers are in view.

Move on to the next lab to explore single and multi agent navigation. 

Reference:
https://github.com/narsimlukemsaram/COMP0182-Multi-Agent-Systems