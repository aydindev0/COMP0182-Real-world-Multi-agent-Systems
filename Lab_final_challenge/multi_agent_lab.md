# Multi-Robot ArUco-Based Navigation

## Prerequisites

### Hardware
- Two TurtleBot3 Burger robots
- Logitech C920 HD Pro camera
- ArUco markers:
  - **Corner markers**: id500, id501, id502, id503 (define the navigation area)
  - **Robot markers**: id505, id101 (attached to robots)
  - **Additional markers** as needed for fetch points

### Software
- Ubuntu 20.04
- ROS Noetic
- Python 3.7+
- Required ROS packages:
  ```bash
  sudo apt install ros-noetic-usb-cam
  sudo apt install ros-noetic-rqt-image-view
  pip install opencv-python numpy pyyaml
  ```

---

## System Architecture

### Coordinate Transformation
The system converts simulation coordinates to real-world coordinates using a perspective transformation:
- **Simulation space**: 10×10 unit square
- **Real-world space**: Physical arena measured by ArUco corner markers
- **Method**: OpenCV `getPerspectiveTransform()` using the four corner markers

### Navigation Components
1. **ArUco Detection**: Camera detects markers and publishes poses
2. **Coordinate Transform**: Converts waypoints from sim to real-world
3. **Proportional Controller**: Moves robots to waypoints sequentially
4. **Multi-threading**: Each robot runs in a separate thread

## Setup Instructions

### 1. Camera Setup

**Identify your camera:**
```bash
ls /dev/video*
```
Plug/unplug the camera to identify the correct device (typically `/dev/video2`).

**Disable autofocus:**
```bash
sudo apt-get install uvcdynctrl
uvcdynctrl --device=/dev/video2 --set='Focus, Auto' 0
```

**Verify autofocus is disabled:**
```bash
uvcdynctrl --device=/dev/video2 --get='Focus, Auto'
# Should return 0
```

### 2. ArUco Marker Placement

**Corner Markers** (define the navigation area):
- id503: bottom-left corner (sim: 0, 0)
- id502: bottom-right corner (sim: 10, 0)
- id500: top-left corner (sim: 0, 10)
- id501: top-right corner (sim: 10, 10)

**Robot Markers**:
- id505: TurtleBot3 #1 (tb3_0)
- id101: TurtleBot3 #2 (tb3_1)

Place markers flat on the ground, ensuring that they're visible to the overhead camera.

### 3. Waypoint File Format

Create a YAML file (e.g., `waypoints.yaml`) with waypoints in simulation coordinates:

```yaml
schedule:
  agent1:
    - x: 1.0
      y: 1.0
    - x: 5.0
      y: 5.0
    - x: 9.0
      y: 9.0
  agent2:
    - x: 2.0
      y: 2.0
    - x: 6.0
      y: 6.0
    - x: 8.0
      y: 8.0
```

The script will automatically convert these to real-world coordinates.

---

## Running the System

### Terminal 1: ROS Master
```bash
roscore
```
Keep this running throughout the session.

### Terminal 2: TurtleBot3 #1 Bringup
```bash
ssh ubuntu@{IP_ADDRESS_TB3_0}
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

### Terminal 3: TurtleBot3 #2 Bringup
```bash
ssh ubuntu@{IP_ADDRESS_TB3_1}
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

### Terminal 4: ArUco Detection
```bash
export TURTLEBOT3_MODEL=burger
roslaunch auto_aruco_marker_finder multiple_aruco_marker_finder.launch
```

**Important**: Update the camera device in the launch file:
- (you can do this via command line arguments)
- File: `catkin_ws/src/.../auto_aruco_marker_finder/launch/multiple_aruco_marker_finder.launch`
- Change the ArUco id numbers to match your environment 

Before publishing images, you may need to disable autofocus on your camera for better marker detection. If using `/dev/video0`, run:

```bash
sudo v4l2-ctl -d /dev/video0 --set-ctrl=focus_automatic_continuous=0
```


### Terminal 5: Verify Detection (Optional)
```bash
rosrun rqt_gui rqt_gui
```
View topics: `/id505/aruco_single/result`, `/id101/aruco_single/result`, etc.

### Terminal 6: Run Navigation Script
```bash
cd catkin_ws/src/COMP0182-Multi-Agent-Systems/turtlebot3_burger_auto_navigation/auto_navigation/scripts
chmod +x multi_goal_navigation.py
rosrun auto_navigation multi_goal_navigation.py
```

You can use the script with these parameters:
```bash
rosrun auto_navigation multi_agent_navigation1.py \
  _waypoint_file:=./waypoints.yaml \
  _robot_names:="['tb3_0', 'tb3_1']" \
  _aruco_ids:="['id505', 'id101']"
```

---

## Troubleshooting

### Camera Not Detected
```bash
# Check video devices
v4l2-ctl --list-devices

```

### ArUco Markers Not Detected
1. Ensure markers are within camera field of view
2. Check lighting conditions (avoid glare)
3. Verify marker IDs match configuration
4. Check that autofocus is disabled
5. Verify camera launch file has correct device path

### Robot Not Moving
1. Verify ArUco marker detection in rqt
2. Check that robot namespaces match in launch files
3. Ensure `cmd_vel` topic is being published:
   ```bash
   rostopic echo /tb3_0/cmd_vel
   ```
4. Verify robot markers are correctly oriented (ArUco pattern facing up)

### Transformation Matrix Errors
- Ensure all four corner markers (id500-503) are detected
- Check that markers form a proper quadrilateral
- Verify markers are placed at correct corners

### Launch File Configuration
If camera feed fails, update these parameters in launch files:

```xml
<arg name="video_device" default="/dev/video2" />
<param name="focus_auto" value="0" />  <!-- Disable autofocus -->
<param name="pixel_format" value="mjpeg" />  <!-- Or try "yuyv" -->
```

---

## Control Parameters

Edit these constants in the script to tune robot behavior:

```python
# Control gains
K_LINEAR = 0.5          # Linear velocity gain
K_ANGULAR = 2.0         # Angular velocity gain

# Speed limits
MAX_LINEAR_SPEED = 0.2  # m/s
MAX_ANGULAR_SPEED = 1.0 # rad/s

# Navigation
POSITION_TOLERANCE = 0.1  # meters (waypoint reached threshold)
CONTROL_LOOP_RATE = 0.1   # seconds (control loop frequency)
```

---

## Key Topics

### Subscribed Topics
- `/id500/aruco_single/pose` - Corner marker (top-left)
- `/id501/aruco_single/pose` - Corner marker (top-right)
- `/id502/aruco_single/pose` - Corner marker (bottom-right)
- `/id503/aruco_single/pose` - Corner marker (bottom-left)
- `/id505/aruco_single/pose` - Robot 1 pose
- `/id101/aruco_single/pose` - Robot 2 pose

### Published Topics
- `/tb3_0/cmd_vel` - Robot 1 velocity commands
- `/tb3_1/cmd_vel` - Robot 2 velocity commands

---

## Performance Tips

### For the Final Challenge
1. **Pre-compute trajectories**: Use simulation (PyBullet) to generate optimal waypoint files
2. **Use path planning**: Apply algorithms of your choice 
3. **Collision avoidance**: Ensure waypoints maintain safe distance between robots
4. **Calibration**: Test and refine coordinate transformation in the actual arena
5. **Timing**: Minimize waypoint count while ensuring coverage of fetch points

## Quick Reference

### Stop Robots (Emergency)
```bash
rostopic pub /tb3_0/cmd_vel geometry_msgs/Twist "linear: {x: 0.0} angular: {z: 0.0}"
rostopic pub /tb3_1/cmd_vel geometry_msgs/Twist "linear: {x: 0.0} angular: {z: 0.0}"
```

---

## Additional Resources

- TurtleBot3 e-Manual: https://emanual.robotis.com/docs/en/platform/turtlebot3/
- ROS Navigation: https://wiki.ros.org/navigation
- ArUco Markers: https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html
- Course Repository: https://github.com/narsimlukemsaram/COMP0182-Multi-Agent-Systems

