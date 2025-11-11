#!/usr/bin/env python3
"""
Launch file for YOLO Test standalone detection
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description for YOLO test"""
    
    # Declare launch arguments
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Camera device path (e.g., /dev/video0)'
    )
    
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value='yolov8n.pt',
        description='YOLOv8 model file (yolov8n.pt, yolov8s.pt, etc.)'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.15',
        description='Minimum confidence threshold for YOLO detections'
    )
    
    preset_logitech_arg = DeclareLaunchArgument(
        'preset_logitech',
        default_value='false',
        description='Apply Logitech C920 camera presets (disables autofocus)'
    )
    
    zoom_arg = DeclareLaunchArgument(
        'zoom',
        default_value='200',
        description='Zoom level for cameras that expose zoom_absolute (default 200; 100=no zoom)'
    )

    # Launch the YOLO test node
    yolo_test_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'yolo', 'yolo_test',
            '--camera', LaunchConfiguration('camera_device'),
            '--model', LaunchConfiguration('yolo_model'),
            '--conf', LaunchConfiguration('confidence_threshold'),
            '--preset-logitech', LaunchConfiguration('preset_logitech'),
            '--zoom', LaunchConfiguration('zoom'),
        ],
        output='screen'
    )

    return LaunchDescription([
        camera_device_arg,
        yolo_model_arg,
        confidence_threshold_arg,
        preset_logitech_arg,
        zoom_arg,
        yolo_test_node
    ])
