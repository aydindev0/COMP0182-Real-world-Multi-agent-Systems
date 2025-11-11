#!/usr/bin/env python3
"""
Camera Publisher Node

Publishes camera feed from V4L2 device using GStreamer to ROS topic.

This node:
1. Uses GStreamer to capture video from V4L2 device
2. Converts frames to ROS Image messages
3. Publishes to /ceiling_camera/image_raw topic
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
import threading
import numpy as np


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")
        
        # Declare parameters
        self.declare_parameter("image_topic", "/ceiling_camera/image_raw")
        self.declare_parameter("device", "/dev/video3")
        self.declare_parameter("width", 1920)
        self.declare_parameter("height", 1080)
        self.declare_parameter("framerate", 30)
        self.declare_parameter("publish_rate", 30.0)
        
        # Get parameters
        self.image_topic = self.get_parameter("image_topic").value
        self.device = self.get_parameter("device").value
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.framerate = int(self.get_parameter("framerate").value)
        self.publish_rate = self.get_parameter("publish_rate").value
        
        # CV bridge for image conversion
        self.bridge = CvBridge()
        
        # Publisher
        self.image_pub = self.create_publisher(Image, self.image_topic, 10)
        
        # Build GStreamer pipeline string
        # Format: v4l2src device=DEVICE ! image/jpeg,width=W,height=H,framerate=F/1 ! jpegdec ! videoconvert ! appsink
        self.gst_pipeline = (
            f"v4l2src device={self.device} ! "
            f"image/jpeg,width={self.width},height={self.height},framerate={self.framerate}/1 ! "
            f"jpegdec ! videoconvert ! appsink"
        )
        
        # Camera capture
        self.cap = None
        self.camera_thread = None
        self.running = False
        
        # Start camera capture
        self.start_camera()
        
        # Timer for publishing images
        period = 1.0 / self.publish_rate if self.publish_rate > 0 else 1.0
        self.create_timer(period, self.publish_frame)
        
        self.get_logger().info("Camera Publisher Node initialized")
        self.get_logger().info(f"GStreamer pipeline: {self.gst_pipeline}")
        self.get_logger().info(f"Publishing to: {self.image_topic}")
    
    def start_camera(self):
        """Start camera capture using GStreamer"""
        try:
            # Open camera with GStreamer pipeline
            self.cap = cv2.VideoCapture(self.gst_pipeline, cv2.CAP_GSTREAMER)
            
            if not self.cap.isOpened():
                self.get_logger().error(f"Failed to open camera device: {self.device}")
                self.get_logger().error("Make sure GStreamer is installed and camera device exists")
                return False
            
            # Set camera properties
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.framerate)
            
            self.running = True
            self.get_logger().info(f"Camera opened successfully: {self.device}")
            self.get_logger().info(f"Resolution: {self.width}x{self.height} @ {self.framerate} fps")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error starting camera: {e}")
            return False
    
    def publish_frame(self):
        """Capture and publish a frame"""
        if not self.running or self.cap is None or not self.cap.isOpened():
            return
        
        try:
            # Read frame from camera
            ret, frame = self.cap.read()
            
            if not ret or frame is None:
                self.get_logger().warn("Failed to read frame from camera")
                return
            
            # Convert OpenCV frame to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "ceiling_camera"
            
            # Publish image
            self.image_pub.publish(ros_image)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing frame: {e}")
    
    def destroy_node(self):
        """Clean up camera on shutdown"""
        self.running = False
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
