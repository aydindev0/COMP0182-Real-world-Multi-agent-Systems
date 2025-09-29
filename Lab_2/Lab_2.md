# COMP0182 Real-world Multi-agent Systems: Lab Sheet One

In this exercise you will edit and build a ROS package. It is adapted from the ROS 2 Humble tutorials.


## Configure environment

In this section you will configure the ROS workspace on the Turtlebot. 

Source the ROS setup by restarting your shell:

```
source ~/.bashrc
```

Set your domain ID to your group number to avoid conflicts over the network:

```
export ROS_DOMAIN_ID=<your_group_number>
echo "export ROS_DOMAIN_ID=<your_group_number>" >> ~/.bashrc
```

## Understand the workspace

A workspace is a directory containing ROS 2 packages. Remember to source your ROS 2 installation in every terminal you plan to work in. This step will be omitted going forward.

**1. Navigate to the src directory within your Turtlebot3_ws**

This is the src directory within your Turltebot's ROS 2 workspace. Packages live inside the src directory. 

```
cd ~/turtlebot3_ws/src
```

## Create a package

Within your workspace you will now create a package.

**1. Create a package in the src directory**

This will create a package named py_pubsub. 
```
ros2 pkg create --build-type ament_python --license Apache-2.0 py_pubsub
```

**2. Download the example talker code**

```
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
```

There will now be a file named publisher_member_function.py in your directory. 

**3. Open and examine the file**

Go through each line and determine what the code will do. You can read a breakdown of the code from the ROS 2 tutorial linked on this page. 


```
nano publisher_member_function.py
```


**4. Change the counter and timer**

Instead of the current output, the message data should be changed to say hello from your group (remember to include your group number).

**5. Add dependencies**

Open package.xml in the same way as the python file. The description, maintainer and license may have been autocompleted. This will be important if you decide to share a package in the future. 

After the line with the license, add the dependencies:

```
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

Save the file and close.

```
ctrl + S
ctrl + X
```

### Add an entry point

Open setup.py. If you did not make any edits to the maintainer, description or license in the package.xml it should be identical. These must match!

In setup.py add the following line to entry_points:

```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
```

## Build and run 

From the root of your ros workspace enter the following to ensure the dependencies are installed:

```
rosdep install -i --from-path src --rosdistro humble -y
```

Now build the package from the root of the workspace:

```
colcon build --packages-select py_pubsub
```

Open a new terminal and source the installation

```
source install/setup.bash
```

## Talk over the local network

You will be given an IP address to complete the following in the lab. When provided, enter this inter your terminal:

```
export ROS_DISCOVERY_SERVER=<provided_IP_address>
ros2 daemon stop # reload ROS 2 daemon
ros2 run talker
```

You should receive a response in your terminal, once your message is delivered. 

Feel free to explore the ROS 2 Humble tutorials here https://docs.ros.org/en/humble/Tutorials.html


This lab was adapted from 
Adapted from https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html and https://husarion.com/tutorials/ros2-tutorials/6-robot-network/