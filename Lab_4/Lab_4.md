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

**2. Repository Setup and Dependencies**

Before proceeding with today's tasks, you'll need to set up the required repository and dependencies. The lab uses a custom ROS2 workspace that integrates TurtleBot navigation with PhaseSpace motion capture.

**2.1 Conda Environment Setup**

If you don't have conda installed, you'll need to install it first:

```bash
# Download and install Miniforge
wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
# Follow the installation prompts and restart your terminal
```

**2.2 Repository Setup**

Clone the required repository from GitHub:

```bash
# Clone the repository
git clone https://github.com/aydindev0/turtlebot_and_phasespace.git
cd turtlebot_and_phasespace

# Create the conda environment
conda env create -f envros2.yaml
conda activate envros2

# Navigate to the ROS2 workspace
cd ros_envs/multi_agent_systems_ws

# Build the required packages
colcon build --packages-select phasespace_msgs_2
colcon build --packages-select auto_nav

# Source the workspace
source install/setup.bash
```

**3. Understanding PhaseSpace Integration**

Now that you have the workspace set up, you can work with PhaseSpace data. Your robot has 4 LEDs on top, each with a unique ID that has been grouped into a rigid body. Each group will be able to subscribe to a topic named `/rigid_body_turtlebot{group_number}` published on the lab PC.

**3.1 Monitoring PhaseSpace Data**

You can echo any topic in the terminal by using:

```
ros2 topic echo {topic_name}
```

**Test PhaseSpace Connection:**
```bash
# Monitor your robot's rigid body data
ros2 topic echo /rigid_body_turtlebot{your_group_number}

# Check available PhaseSpace topics
ros2 topic list | grep rigid_body
```

When viewing the rigid body data, you will be able to see the position and orientation of your robot in the 2D plane. You should observe:
- **Position data**: X, Y coordinates in the arena
- **Orientation data**: Quaternion representation of robot heading
- **Timestamp information**: For synchronisation with other systems

**4. Understanding PhaseSpace Integration**

The repository includes enhanced navigation capabilities that integrate with PhaseSpace:

- **`phasespace_msgs_2`**: Custom message definitions for PhaseSpace rigid body data
- **`auto_nav`**: Enhanced TurtleBot navigation package with PhaseSpace integration
- **Quadrant-based heading detection**: Converts PhaseSpace orientation data to robot heading
- **Dual control modes**: Point-to-point and pose-to-pose navigation

**5. Today's Main Tasks**

**5.1 Task 1: Understanding PhaseSpace and Navigation**

**Primary Objectives**: Monitor pose information and understand orientation data conversion.

**Critical Investigation**: PhaseSpace provides orientation data as quaternions, but robots typically work with heading angles. You need to understand how this conversion works.

**Code Exploration Challenge**: 
- Examine the navigation code to find where quaternion data is processed
- Look for functions that convert orientation data to robot headings
- Understand how the system determines which direction the robot is facing

**Hint**: Search for mathematical operations involving quaternions, angles, or trigonometric functions in the navigation code. The conversion likely involves extracting the yaw angle from the quaternion representation.

**5.2 Task 2: Point-to-Point Navigation with Orientation Control**

Using the PhaseSpace data, implement navigation from point A to point B, matching the simulation from Lab 3.

**Hint**: Start by exploring the launch files to understand the available parameters and how they differ from the simulation setup.

**Key Requirements:**
- Robot must reach the goal position within the hysteresis zone (safe tolerance)
- Robot must face the correct orientation
- Navigation should be smooth and efficient
- Use PhaseSpace data for position feedback

**Advanced Challenge**: Extend your navigation to ensure the robot not only reaches the desired position but also faces the correct orientation.

**Investigation Required**:
- How does the current navigation system handle orientation at the goal?
- What parameters control the final orientation behaviour?
- Can you modify the system to specify both position AND orientation targets?

**Code Analysis Questions**:
1. Does the navigation system support pose-to-pose control (position + orientation)?
2. How are orientation errors calculated and corrected?
3. What happens when the robot reaches the position but faces the wrong direction?

**Implementation Challenge**: Modify your navigation setup to ensure the robot ends up facing a specific direction at the goal position. You'll need to understand how the system processes both position and orientation targets.

**6. Advanced Features and Code Understanding**

**Essential Code Reading**: Examine the navigation code structure and implement advanced features:

```bash
# Explore the auto_nav package structure
cd turtlebot_and_phasespace/ros_envs/multi_agent_systems_ws/src/auto_nav
ls -la
```

**Critical Analysis Required**: Study these key files and answer:
- **`launch/simple_navigation.launch.py`**: What parameters are configurable? How do they differ from the enhanced version?
- **`launch/enhanced_navigation.launch.py`**: What additional features does this provide?
- **`src/auto_nav/navigation_node.py`**: How does the navigation algorithm work? What control strategy is implemented?

**Code Investigation Questions**:
1. How does the system convert PhaseSpace quaternion data to robot heading? (Focus on the mathematical conversion)
2. What control algorithm is used for reaching the goal position?
3. How are velocity limits enforced?
4. How does the system handle orientation control versus position control?
5. What's the difference between point-to-point and pose-to-pose navigation modes?

**Advanced Features to Explore**:
- **Quadrant-based heading detection**: Understand how PhaseSpace quaternions are converted to robot headings
- **Dual control modes**: Implement both point-to-point and pose-to-pose navigation
- **Velocity limits**: Experiment with different `vmax` and `wmax` parameters
- **Custom goal updates**: How does the system handle goal updates? What message types are expected?

**Monitoring and Testing**:
```bash
# Monitor robot position and orientation
ros2 topic echo /rigid_body_turtlebot{your_group_number} --field position
ros2 topic echo /rigid_body_turtlebot{your_group_number} --field orientation
ros2 topic echo /cmd_vel
```

**Hint**: Look for topic publishers and subscribers in the navigation code, and examine the message types being used. The launch files might give you clues about expected topic names and parameters.

**7. Lab Objectives Summary**

By the end of this lab, you should be able to:

1. **Monitor pose information**: Successfully observe robot position and orientation via PhaseSpace
2. **Point-to-point navigation**: Navigate from point A to point B using PhaseSpace feedback
3. **Match simulation behaviour**: Replicate the navigation performance from Lab 3 simulation
4. **Understand code structure**: Comprehend the navigation implementation and PhaseSpace integration
5. **Update goal positions**: Dynamically modify navigation targets during operation

**Extension Objectives:**
- **Pose control deep dive**: Implement advanced pose control features
- **Custom parameter tuning**: Optimise navigation parameters for your specific robot
- **Real-time monitoring**: Develop custom monitoring and logging capabilities

**8. Troubleshooting**

**Common Issues:**
- **Conda environment not found**: Ensure you've activated the `envros2` environment
- **Package build failures**: Check that all dependencies are installed correctly
- **PhaseSpace data not received**: Verify your robot's LED configuration and topic names
- **Navigation not working**: Check that both TurtleBot and PhaseSpace systems are running

**Getting Help:**
- Check the repository documentation: `turtlebot_and_phasespace/README.md`
- Review package-specific documentation in the `auto_nav` package
- Consult with lab instructors for PhaseSpace-specific issues

