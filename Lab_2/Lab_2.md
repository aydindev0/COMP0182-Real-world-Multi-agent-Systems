# COMP0182 Real-world Multi-agent Systems: Lab Sheet Two

In this exercise you will edit and build a ROS package. It is adapted from the ROS 2 Humble tutorials.

## Talker and listener exercise

### Configure the environment

```
export ROS_DOMAIN_ID=30
ros2 topic pub -r 1 /msg_{your group number} std_msgs/msg/String data:\ 'Hello, from {your group number}'
```

### Listener 
Check if the topic is published using a second laptop

```
export ROS_DOMAIN_ID=30
ros2 topic list
```

You should be able to see the new topic you just published, with the name ```msg_{your group number}```.

If you see the topic, run:
```
ros2 topic echo {group topic name}
```
Your terminal should receive the message from the talker.

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
