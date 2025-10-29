# COMP0182 Real-world Multi-agent Systems: Lab Sheet Five

In this week's lab you will work with Simultaneous Localisation and Mapping (SLAM) to create a map of the physical arena, understand the mapping process, and explore path planning with your generated maps.

**Prerequisites**: Ensure that your robot is properly connected and configured.

## 0. Namespace Configuration for This Lab

**Important**: For this SLAM mapping task, you will need to **remove the namespace** you added in Lab 4. 

**Why namespaces matter**: Namespaces resolve domain problems by prefixing all topics with a unique identifier (e.g., `/turtle8/cmd_vel`). This prevents topic name conflicts when multiple robots share the same ROS domain. When multiple robots are in the scene, especially with PhaseSpace, namespaces are essential to distinguish between each robot's topics.

**For this task**: You should temporarily revert to the default namespace configuration. To do this, edit the `robot.launch.py` file on the TurtleBot3 SBC and set the namespace default back to an empty string:

```bash
namespace = LaunchConfiguration('namespace', default='')
```

**When to use namespaces**: As soon as you have more than one robot in your scene, you must use namespaces, especially when working with PhaseSpace, where rigid body topics must be clearly distinguished between robots.

## 1. SLAM Mapping the Arena

Your first task is to create a map of the physical arena using SLAM. This map will be used for navigation and path planning in future sessions.

### 1.1 Robot Bringup

Start by launching the robot on the TurtleBot3 SBC:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```


### 1.2 Launch SLAM Node

Open a new terminal on the Remote PC and launch the Cartographer SLAM node:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

**Investigation Required**: While the SLAM node is running, explore what topics and nodes are active:

```bash
ros2 node list
ros2 topic list
```

**Questions to Consider**:
- What data is the SLAM algorithm subscribing to?
- What map-related topics are being published?

### 1.3 Teleoperation and Mapping

In another terminal on the Remote PC, launch the teleoperation node:

```bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

**Mapping Strategy**:
- Drive the robot slowly and methodically around the arena
- Cover all accessible areas, including corners and boundaries
- Drive close to walls to improve mapping accuracy
- Make multiple passes through open areas for better loop closure

**Critical Understanding**: While mapping, think about what's happening behind the scenes:
- How is the robot estimating its position without a prior map?
- What sensor data is being used for mapping?
- How does the system handle uncertainty in both localisation and mapping?


### 1.4 Save Your Map

Once you've mapped the arena to your satisfaction, save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

This will create two files: `~/map.pgm` (the map image) and `~/map.yaml` (the map metadata). The map will be saved in your home directory. 

**Map Quality Check**:
- Examine the generated map image
- Verify that walls, obstacles, and boundaries are clearly defined
- Check for any missed areas or mapping errors
- Compare maps with other groups to identify differences

## 2. Understanding SLAM: Behind the Scenes

### 2.1 What is SLAM Actually Doing?

**Code Exploration**: Examine the Cartographer launch file to understand the configuration. You will have to find the correct directory for the package in your turltebot3_ws 

```bash
cd turtlebot3_cartographer
cat launch/cartographer.launch.py
```

**Investigation Tasks**:
1. What sensor inputs does Cartographer require?
2. What coordinate frames are being used?
3. How are occupancy grid maps represented?
4. What parameters control the mapping quality and computational cost?

### 2.2 SLAM Challenges in Real-World Scenarios

**Discussion Points** (think about these while mapping):

- **Localisation Uncertainty**: How does the robot know where it is if the map isn't complete?
- **Mapping Uncertainty**: How accurate is the map if the robot's position is uncertain?
- **Sensor Fusion**: What happens when sensor data is noisy or conflicting?
- **Loop Closure**: How does the system recognise previously visited locations?
- **Computational Cost**: What makes SLAM computationally intensive?

**Practical Observation**: 
- Notice how the map updates in real-time as you drive
- Observe any corrections that happen during loop closure
- Watch for map inconsistencies or drift issues

## 3. Simulation with Control Parameters

Generate plots under simulation using different k1 and k2 values from Lab 3. Test at least 3 different parameter combinations and create comparison plots showing trajectory paths, control effort, convergence behaviour, and time to reach goal.

**Hints**: 
- Use the simulation environment from Lab 3
- Examine the navigation code to understand what k1 and k2 control in the control law
- Consider how different k1/k2 ratios affect trajectory smoothness and the trade-off between speed and accuracy

## 4. Waypoint Planning with Generated Maps

### 4.1 Using Your Map for Path Planning

**Main Task**: Adapt the waypoint planning logic from the simulation (Lab 3) to work with your generated map instead of the PyBullet planner.

**Investigation Required**:
- How can you load and use your generated map in the simulation?
- What path planning algorithms are available in the navigation stack?
- How does the PyBullet planner work, and how can you replace it with map-based planning?

**Code Exploration**:
1. Examine the map file structure (`map.yaml` and `map.pgm`)
2. Understand how occupancy grids are represented
3. Look for path planning nodes in the navigation stack
4. Investigate how to integrate map-based planning with your navigation code

**Implementation Focus**: The goal is to use the same waypoint logic from Lab 3's simulation, but generate paths using your actual arena map rather than the simulation environment. This means understanding how to load occupancy grids and translate waypoint goals into map-based paths.

## 5. Lab Objectives Summary

By the end of this lab, you should be able to:

1. **Perform SLAM mapping**: Successfully create maps of the physical arena using Cartographer
2. **Understand SLAM principles**: Explain what's happening behind the scenes during mapping
3. **Analyse control parameters**: Generate and analyse plots for different k1/k2 parameter combinations
4. **Integrate maps with planning**: Use generated maps for path planning by adapting simulation waypoint logic to map-based planning

## 6. Troubleshooting

**Common Issues**:

- **Map not updating**: Check that both robot and SLAM nodes are running and communicating
- **Poor map quality**: Drive more slowly, get closer to walls, make multiple passes
- **Teleoperation not working**: Verify the correct terminal has focus for keyboard input
- **Map save fails**: Check file permissions and ensure the directory exists
- **Simulation parameters not working**: Verify you're using the correct parameter names and units

**SLAM-Specific Issues**:
- **Drift or incorrect loop closure**: The map may need more consistent sensor data or recalibration
- **Missing areas**: Ensure comprehensive coverage and adequate sensor range
- **Distorted geometry**: May indicate sensor synchronisation or calibration issues

**Getting Help**:
- Consult Cartographer documentation for parameter tuning
- Review Nav2 documentation for path planning integration
- Discuss with other groups about effective mapping strategies
- Consult with lab instructors for sensor or calibration issues

