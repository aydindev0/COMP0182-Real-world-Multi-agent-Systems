# COMP0182 Real-world Multi-agent Systems: Lab Sheet One

In this exercise you will edit and build a ROS package. It is adapted from the ROS 2 Humble tutorials.

## Talker and listener exercise

### Configure environment

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

### Understand the workspace

A workspace is a directory containing ROS 2 packages. Remember to source your ROS 2 installation in every terminal you plan to work in. This step will be omitted going forward.

**1. Navigate to the src directory within your Turtlebot3_ws**

This is the src directory within your Turltebot's ROS 2 workspace. Packages live inside the src directory. 

```
cd ~/turtlebot3_ws/src
```

### Create a package

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

### Build and run 

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

### Talk over the local network

You will be given an IP address to complete the following in the lab. When provided, enter this inter your terminal:

```
export ROS_DISCOVERY_SERVER=<provided_IP_address>
ros2 daemon stop # reload ROS 2 daemon
ros2 run talker
```

You should receive a response in your terminal, once your message is delivered. 

Feel free to explore the ROS 2 Humble tutorials here https://docs.ros.org/en/humble/Tutorials.html



## Unit tests

**1. Open a new terminal**

Open a new terminal on your laptop and connect to the Raspberry Pi of your Turtlebot via SSH using its IP address.

If you do not know the IP address of the Rasperry Pi, you will need to connect a monitor and keyboard and type ```ip a``` in the terminal while connected to the MSc_IoT network via Wi-Fi. The IP address will be listed next to ```inet```. If you changed the username on the Raspberry Pi, use that in place of ```ubuntu``` below:

```
ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
```

Now specify the model of the Turtlebot3 (burger) and launch the bringup in the same terminal:

```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

**2. Operate your Turtlebot**

Open a new terminal and run the following on your laptop:

```
ros2 run turtlebot3_teleop teleop_keyboard
```

You will be able to move the Turtlebot using your keyboard. Instructions will be displayed in the terminal window.

**3. Note the position, orientation and velocity**

To retrieve more data, you can use the topic monitor while using teleoperation to move the robot. In your terminal on the remote PC, run:

```
rqt
```

You will see a list of topics you can subscribe to. If you cannot see the topics, click on ```plugins > Topics > Topic monitor```. Select each topic to see what data is collected, and find the topic(s) that allow you to retreieve the position, orientation and velocity of the Turtlebot. 

Once you have found the topic of interest, you can run:

```
ros2 topic echo {topic_name}
```


This lab was adapted from 
Adapted from https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html and https://husarion.com/tutorials/ros2-tutorials/6-robot-network/
and https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#basic-operation