#!/usr/bin/env python3
"""
ROS Node for Autonomous Navigation of TurtleBots Using ArUco Markers

This script provides multi-robot autonomous navigation by:
- Reading waypoints from a YAML file (simulation coordinates)
- Converting them to real-world coordinates using ArUco corner markers
- Using a proportional controller to move one or more robots
- Running each robot's navigation loop in a separate thread

Author: Original implementation with tidiness improvements
License: MIT (or your preferred license)
"""

# ======================================================================
# Imports
# ======================================================================

import math
import threading
from typing import List, Tuple, Dict

import cv2
import numpy as np
import rospy
import yaml
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion


# ======================================================================
# Constants
# ======================================================================

# Control parameters
K_LINEAR = 0.5
K_ANGULAR = 2.0
MAX_LINEAR_SPEED = 0.2
MAX_ANGULAR_SPEED = 1.0

# Navigation parameters
POSITION_TOLERANCE = 0.1  # meters
CONTROL_LOOP_RATE = 0.1  # seconds

# ArUco marker configuration
CORNER_MARKERS = ['id503', 'id502', 'id500', 'id501']

# Simulation coordinate bounds (for perspective transform)
SIM_CORNER_COORDS = np.float32([
    [0, 0],    # bottom-left
    [10, 0],   # bottom-right
    [0, 10],   # top-left
    [10, 10],  # top-right
])


# ======================================================================
# Coordinate Transformation
# ======================================================================

def convert_sim_to_real_pose(x: float, y: float, matrix: np.ndarray) -> Tuple[float, float]:
    """
    Apply a 3x3 perspective transformation to convert simulation coordinates to real-world.

    Args:
        x: Simulation-space x coordinate
        y: Simulation-space y coordinate
        matrix: 3x3 perspective transform matrix

    Returns:
        Tuple of (real_x, real_y) coordinates
    """
    point = np.array([x, y, 1])
    transformed_point = np.dot(matrix, point)
    # Normalize by homogeneous coordinate
    transformed_point = transformed_point / transformed_point[2]
    return transformed_point[0], transformed_point[1]


def get_transformation_matrix(aruco_markers: List[str]) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute perspective transform matrix from ArUco corner markers.

    Reads poses for four corner ArUco markers and computes both forward
    (sim->real) and reverse (real->sim) perspective transformations.

    Args:
        aruco_markers: List of marker IDs defining the real-world square corners
                      Order: [bottom-left, bottom-right, top-left, top-right]

    Returns:
        Tuple of (forward_matrix, reverse_matrix) for coordinate transformation

    Raises:
        rospy.ROSException: If timeout occurs while waiting for marker poses
    """
    marker_poses = {}

    # Collect poses for each corner marker
    for marker_id in aruco_markers:
        try:
            pose = rospy.wait_for_message(
                f'/{marker_id}/aruco_single/pose',
                PoseStamped,
                timeout=5
            )
            marker_poses[marker_id] = (pose.pose.position.x, pose.pose.position.y)
            rospy.loginfo(f"Received pose for marker {marker_id}: "
                         f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
        except rospy.ROSException as e:
            rospy.logerr(f"Timeout while waiting for marker {marker_id}: {e}")
            raise

    # Define real-world corner positions (matches aruco_markers order)
    real_points = np.float32([
        marker_poses['id503'],  # bottom-left
        marker_poses['id502'],  # bottom-right
        marker_poses['id500'],  # top-left
        marker_poses['id501'],  # top-right
    ])

    # Compute bidirectional transformation matrices
    forward_matrix = cv2.getPerspectiveTransform(SIM_CORNER_COORDS, real_points)
    reverse_matrix = cv2.getPerspectiveTransform(real_points, SIM_CORNER_COORDS)

    rospy.loginfo("Perspective transformation matrix calculated successfully.")
    return forward_matrix, reverse_matrix


# ======================================================================
# Navigation Helper Functions
# ======================================================================

def check_goal_reached(current_pose: PoseStamped, 
                      goal_x: float, 
                      goal_y: float, 
                      tolerance: float = POSITION_TOLERANCE) -> bool:
    """
    Check if the robot has reached the goal within a tolerance box.

    Args:
        current_pose: Current robot pose
        goal_x: Target x coordinate
        goal_y: Target y coordinate
        tolerance: Position tolerance in meters

    Returns:
        True if robot is within tolerance of goal
    """
    current_x = current_pose.pose.position.x
    current_y = current_pose.pose.position.y
    return (abs(current_x - goal_x) <= tolerance and 
            abs(current_y - goal_y) <= tolerance)


def normalize_angle(angle: float) -> float:
    """
    Normalize angle to [0, 2π) range.

    Args:
        angle: Angle in radians

    Returns:
        Normalized angle in [0, 2π)
    """
    return (angle + 2 * math.pi) % (2 * math.pi)


def compute_angle_error(current: float, target: float) -> float:
    """
    Compute shortest angular distance from current to target angle.

    Args:
        current: Current angle in radians
        target: Target angle in radians

    Returns:
        Angle error in range [-π, π]
    """
    error = target - current
    if error > math.pi:
        error -= 2 * math.pi
    elif error < -math.pi:
        error += 2 * math.pi
    return error


def compute_control_velocities(distance: float, 
                               theta_error: float) -> Tuple[float, float]:
    """
    Compute control velocities using proportional control law.

    Args:
        distance: Distance to goal in meters
        theta_error: Angular error in radians

    Returns:
        Tuple of (linear_velocity, angular_velocity)
    """
    # Proportional control
    linear_velocity = K_LINEAR * distance * math.cos(theta_error)
    angular_velocity = -K_ANGULAR * theta_error

    # Apply velocity limits
    linear_velocity = np.clip(linear_velocity, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
    angular_velocity = np.clip(angular_velocity, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)

    return linear_velocity, angular_velocity


# ======================================================================
# Core Navigation Loop
# ======================================================================

def navigation(turtlebot_name: str, aruco_id: str, goal_list: List[Tuple[float, float]]):
    """
    Move a TurtleBot through a sequence of waypoints in order.

    This function implements a simple proportional controller to navigate
    the robot through each waypoint sequentially.

    Args:
        turtlebot_name: Robot's namespace prefix (e.g., 'tb3_0')
        aruco_id: ArUco marker ID used for robot pose estimation
        goal_list: List of (x, y) waypoint coordinates in real-world frame
    """
    current_waypoint_idx = 0
    cmd_pub = rospy.Publisher(f'/{turtlebot_name}/cmd_vel', Twist, queue_size=1)

    rospy.loginfo(f"Starting navigation for {turtlebot_name} with {len(goal_list)} waypoints")

    # Get initial pose
    current_pose = rospy.wait_for_message(f'/{aruco_id}/aruco_single/pose', PoseStamped)
    twist = Twist()

    # Main control loop
    while current_waypoint_idx < len(goal_list) and not rospy.is_shutdown():
        goal_x, goal_y = goal_list[current_waypoint_idx]

        # Check if current waypoint is reached
        if check_goal_reached(current_pose, goal_x, goal_y):
            rospy.loginfo(f"{turtlebot_name}: Waypoint {current_waypoint_idx + 1}/{len(goal_list)} reached")
            current_waypoint_idx += 1
            
            if current_waypoint_idx >= len(goal_list):
                rospy.loginfo(f"{turtlebot_name}: All waypoints reached!")
                # Stop the robot
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                cmd_pub.publish(twist)
                break
            continue

        # Update current pose
        current_pose = rospy.wait_for_message(f'/{aruco_id}/aruco_single/pose', PoseStamped)

        # Extract orientation (yaw angle)
        orientation_q = current_pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        # Compute distance and heading to goal
        dx = goal_x - current_pose.pose.position.x
        dy = goal_y - current_pose.pose.position.y
        distance = math.hypot(dx, dy)
        goal_direction = math.atan2(dy, dx)

        # Normalize angles
        current_orientation = normalize_angle(yaw)
        goal_direction = normalize_angle(goal_direction)

        # Compute angle error
        theta_error = compute_angle_error(current_orientation, goal_direction)

        # Debug logging
        if rospy.get_param('/debug', False):
            rospy.logdebug(
                f"{turtlebot_name}: Pos=({current_pose.pose.position.x:.2f}, "
                f"{current_pose.pose.position.y:.2f}), Goal=({goal_x:.2f}, {goal_y:.2f}), "
                f"Dist={distance:.2f}m, Theta={theta_error:.2f}rad"
            )

        # Compute control velocities
        linear_vel, angular_vel = compute_control_velocities(distance, theta_error)

        # Publish velocity command
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        cmd_pub.publish(twist)

        rospy.sleep(CONTROL_LOOP_RATE)

    rospy.loginfo(f"{turtlebot_name}: Navigation complete")


# ======================================================================
# YAML Waypoint Loader
# ======================================================================

def read_yaml_file(file_path: str) -> Dict:
    """
    Load and parse YAML file.

    Args:
        file_path: Path to YAML file

    Returns:
        Dictionary containing schedule data
    """
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data['schedule']


def read_and_transform_waypoints(file_path: str, 
                                 matrix: np.ndarray) -> List[List[Tuple[float, float]]]:
    """
    Load waypoint dictionary from YAML and convert sim coordinates to real-world.

    Args:
        file_path: Path to YAML file containing waypoints
        matrix: Perspective transformation matrix (sim->real)

    Returns:
        List of waypoint lists, one per agent
    """
    # Load YAML structure
    schedule_data = read_yaml_file(file_path)
    coordinates_list = []

    # Process each agent's waypoints
    for agent_id, steps in schedule_data.items():
        rospy.loginfo(f"Processing agent {agent_id} with {len(steps)} waypoints")
        coordinates = []

        for step in steps:
            sim_x = step['x']
            sim_y = step['y']
            real_x, real_y = convert_sim_to_real_pose(sim_x, sim_y, matrix)
            rospy.logdebug(
                f"Agent {agent_id}: Sim ({sim_x}, {sim_y}) -> "
                f"Real ({real_x:.2f}, {real_y:.2f})"
            )
            coordinates.append((real_x, real_y))

        coordinates_list.append(coordinates)

    return coordinates_list


# ======================================================================
# Multi-Robot Coordination
# ======================================================================

def run_multi_robot_navigation(turtlebot_ids: List[str],
                               aruco_ids: List[str],
                               waypoint_lists: List[List[Tuple[float, float]]]):
    """
    Launch navigation threads for multiple robots.

    Args:
        turtlebot_ids: List of robot namespace prefixes
        aruco_ids: List of ArUco marker IDs for pose estimation
        waypoint_lists: List of waypoint lists (one per robot)
    """
    if len(turtlebot_ids) != len(aruco_ids) != len(waypoint_lists):
        rospy.logerr("Mismatch in number of robots, ArUco IDs, and waypoint lists")
        return

    threads = []

    # Start navigation thread for each robot
    for i, (robot_id, aruco_id, waypoints) in enumerate(
        zip(turtlebot_ids, aruco_ids, waypoint_lists)
    ):
        rospy.loginfo(f"Starting thread for robot {robot_id} (ArUco: {aruco_id})")
        thread = threading.Thread(
            target=navigation,
            args=(robot_id, aruco_id, waypoints),
            name=f"nav_{robot_id}"
        )
        threads.append(thread)
        thread.start()

    # Wait for all navigation threads to complete
    for thread in threads:
        thread.join()

    rospy.loginfo("All robots completed navigation")


# ======================================================================
# Debug Utilities
# ======================================================================

def convert_real_to_sim_debug():
    """
    Debug helper: Convert real-world ArUco detections to sim coordinates.

    This function demonstrates the reverse transformation and can be used
    for debugging and validation of the coordinate transformation.
    """
    rospy.loginfo("Running real-to-sim debug conversion...")

    # Get transformation matrices
    _, reverse_matrix = get_transformation_matrix(CORNER_MARKERS)

    # Fetch poses for two test markers
    fetch_1 = rospy.wait_for_message('/id102/aruco_single/pose', PoseStamped)
    fetch_2 = rospy.wait_for_message('/id103/aruco_single/pose', PoseStamped)

    # Extract real-world positions
    real_x1 = fetch_1.pose.position.x
    real_y1 = fetch_1.pose.position.y
    real_x2 = fetch_2.pose.position.x
    real_y2 = fetch_2.pose.position.y

    # Convert to simulation coordinates
    sim_pos1 = convert_sim_to_real_pose(real_x1, real_y1, reverse_matrix)
    sim_pos2 = convert_sim_to_real_pose(real_x2, real_y2, reverse_matrix)

    rospy.loginfo(f"Marker 1 - Real: ({real_x1:.2f}, {real_y1:.2f}) -> Sim: {sim_pos1}")
    rospy.loginfo(f"Marker 2 - Real: ({real_x2:.2f}, {real_y2:.2f}) -> Sim: {sim_pos2}")


# ======================================================================
# Main Entry Point
# ======================================================================

def main():
    """
    Initialize ROS node and coordinate multi-robot navigation.

    This function:
    1. Initializes the ROS node
    2. Computes perspective transformation from ArUco corners
    3. Loads and transforms waypoints from YAML
    4. Launches navigation threads for all robots
    """
    rospy.init_node('multi_goal_pose_navigation', log_level=rospy.INFO)
    rospy.loginfo("Multi-robot navigation node started")

    # Get parameters (with defaults)
    yaml_file = rospy.get_param('~waypoint_file', './waypoints.yaml')
    turtlebot_names = rospy.get_param('~robot_names', ["tb3_0", "tb3_1"])
    aruco_ids = rospy.get_param('~aruco_ids', ["id505", "id101"])

    # Compute perspective transformation matrix
    try:
        forward_matrix, _ = get_transformation_matrix(CORNER_MARKERS)
    except Exception as e:
        rospy.logerr(f"Failed to compute transformation matrix: {e}")
        return

    # Load and transform waypoints
    try:
        waypoint_lists = read_and_transform_waypoints(yaml_file, forward_matrix)
        rospy.loginfo(f"Loaded waypoints for {len(waypoint_lists)} robots")
        for i, waypoints in enumerate(waypoint_lists):
            rospy.loginfo(f"Robot {i}: {len(waypoints)} waypoints")
    except Exception as e:
        rospy.logerr(f"Failed to load/transform waypoints: {e}")
        return

    # Verify configuration
    if len(waypoint_lists) != len(turtlebot_names):
        rospy.logwarn(
            f"Number of waypoint lists ({len(waypoint_lists)}) doesn't match "
            f"number of robots ({len(turtlebot_names)}). Adjusting..."
        )
        # Use minimum to avoid index errors
        num_robots = min(len(waypoint_lists), len(turtlebot_names), len(aruco_ids))
        turtlebot_names = turtlebot_names[:num_robots]
        aruco_ids = aruco_ids[:num_robots]
        waypoint_lists = waypoint_lists[:num_robots]

    # Launch multi-robot navigation
    run_multi_robot_navigation(turtlebot_names, aruco_ids, waypoint_lists)

    rospy.loginfo("Navigation system shutdown complete")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted by user")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
        raise
