# COMP0182 Real-world Multi-agent Systems: Lab Sheet Four

In this week's lab you will be introduced to the Phase Space motion capture system. 

As many of you have realised, there has been some issues with the domain when trying to control your robots. The first step this week will be to resolve this by adding a namespace to each robot. 

**1. Add a namespace to the bringup file.**

SSH into the turtlebot and change directory to:

```
/turtlebot3_ws/src/turtlebot3/turtlebot3_bringup/launch
```

From here you can edit the ```robot.launch.py``` file using

```
sudo nano robot.launch.py
```

On line 6 of the ```generate_launch_description()``` function, you will see a variable named "namespace". Change the empty default to the identity of your group's turtlebot, like so:

```
namespace = LaunchConfiguration('namespace', default='turtle8')
```

using your groups number instead.

Now, after running bringup

```
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30
ros2 launch turtlebot3_bringup robot.launch.py
```

On the remote PC, you should now see the namespace prefix on each topic when running ```ros2 topic list```. You will also see the other topics published by your colleagues'.


```
export ROS_DOMAIN_ID=30
ros2 topic list
```

**2. Understanding Phase Space**

Your robot has 4 LEDs on top, each with a unique ID  that has been grouped into a rigid body. Each group will be able to subscribe to a topic named ```/rigid_body_turtlebot{group_number}``` published on the lab PC. 

You can echo any topic in the terminal by using

```
ros2 topic echo {topic_name}
```

When viewing the rigid body data, you will be able to see the position and orientation of your robot in the 2D plane.  

**3. Today's task**

Today the task is to move your robot to a desired goal  position and orientation. This means that your robot should not only finish at the desired location in the arena, but also should face the correct direction.

