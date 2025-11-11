#!/usr/bin/env python3
"""
YOLOv8 Inference Node

Processes camera feed snapshots using YOLOv8 for object detection.
Detects laptop, book, and mobile phone from COCO dataset.

This node:
1. Subscribes to ceiling camera feed
2. Takes snapshots at configurable rate
3. Runs YOLOv8 inference on snapshots
4. Filters for target classes (laptop, book, mobile phone)
5. Publishes detections with pixel coordinates and confidence
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("Warning: ultralytics not installed. Install with: pip install ultralytics")


# COCO class IDs for target objects
COCO_CLASSES = {
    63: "laptop",
    73: "book",
    67: "cell phone",  # COCO uses "cell phone" for mobile phone
}

# Map COCO names to our object names
COCO_TO_OBJECT = {
    "laptop": "Laptop",
    "book": "Book",
    "cell phone": "Mobile Phone",
}


class YOLOInference(Node):
    def __init__(self):
        super().__init__("yolo_inference")
        
        # Declare parameters
        self.declare_parameter("detection_topic", "/yolo/detections")
        self.declare_parameter("image_topic", "/ceiling_camera/image_raw")
        self.declare_parameter("snapshot_rate", 1.0)  # Hz - how often to take snapshots
        self.declare_parameter("model_path", "yolov8n.pt")  # YOLOv8 model file
        self.declare_parameter("confidence_threshold", 0.5)  # Minimum confidence for detections
        self.declare_parameter("image_size", 640)  # Inference image size
        
        # Get parameters
        self.detection_topic = self.get_parameter("detection_topic").value
        self.image_topic = self.get_parameter("image_topic").value
        self.snapshot_rate = self.get_parameter("snapshot_rate").value
        self.model_path = self.get_parameter("model_path").value
        self.confidence_threshold = self.get_parameter("confidence_threshold").value
        self.image_size = self.get_parameter("image_size").value
        
        # Check if YOLO is available
        if not YOLO_AVAILABLE:
            self.get_logger().error("ultralytics not installed. Please install with: pip install ultralytics")
            raise RuntimeError("ultralytics package not available")
        
        # Load YOLOv8 model
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f"Loaded YOLOv8 model: {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLOv8 model: {e}")
            raise
        
        # CV bridge for image conversion
        self.bridge = CvBridge()
        
        # Current image
        self.current_image = None
        self.image_received = False
        
        # Publishers - using same format as placeholder for compatibility
        self.class_pub = self.create_publisher(String, f"{self.detection_topic}/class", 10)
        self.pixel_pub = self.create_publisher(Point, f"{self.detection_topic}/pixel", 10)
        self.confidence_pub = self.create_publisher(Float64, f"{self.detection_topic}/confidence", 10)
        
        # Image subscriber
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )
        
        # Timer for taking snapshots and running inference
        period = 1.0 / self.snapshot_rate if self.snapshot_rate > 0 else 1.0
        self.create_timer(period, self.process_snapshot)
        
        self.get_logger().info("YOLOv8 Inference Node initialized")
        self.get_logger().info(f"Subscribing to camera: {self.image_topic}")
        self.get_logger().info(f"Publishing detections on: {self.detection_topic}")
        self.get_logger().info(f"Snapshot rate: {self.snapshot_rate} Hz")
        self.get_logger().info(f"Target classes: {list(COCO_TO_OBJECT.values())}")
    
    def image_callback(self, msg):
        """Handle incoming camera image"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.current_image = cv_image
            self.image_received = True
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
    
    def process_snapshot(self):
        """Take a snapshot and run YOLOv8 inference"""
        if not self.image_received or self.current_image is None:
            return
        
        try:
            # Take snapshot and run YOLOv8 inference
            snapshot = self.current_image.copy()
            results = self.model(
                snapshot,
                conf=self.confidence_threshold,
                imgsz=self.image_size,
                verbose=False
            )
            
            # Process detections
            detections_found = False
            best_detection = None
            best_confidence = 0.0
            
            for result in results:
                boxes = result.boxes
                
                for box in boxes:
                    # Get class ID and confidence
                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    
                    # Check if it's one of our target classes
                    if class_id in COCO_CLASSES:
                        coco_name = COCO_CLASSES[class_id]
                        object_name = COCO_TO_OBJECT[coco_name]
                        
                        # Get bounding box center (pixel coordinates)
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        center_x = (x1 + x2) / 2.0
                        center_y = (y1 + y2) / 2.0
                        
                        # Track best detection (highest confidence)
                        if confidence > best_confidence:
                            best_confidence = confidence
                            best_detection = {
                                'class': object_name,
                                'pixel_x': center_x,
                                'pixel_y': center_y,
                                'confidence': confidence
                            }
                            detections_found = True
            
            # Publish best detection if found
            if detections_found and best_detection:
                # Publish class
                class_msg = String()
                class_msg.data = best_detection['class']
                self.class_pub.publish(class_msg)
                
                # Publish pixel coordinates
                pixel_msg = Point()
                pixel_msg.x = float(best_detection['pixel_x'])
                pixel_msg.y = float(best_detection['pixel_y'])
                pixel_msg.z = 0.0
                self.pixel_pub.publish(pixel_msg)
                
                # Publish confidence
                confidence_msg = Float64()
                confidence_msg.data = best_detection['confidence']
                self.confidence_pub.publish(confidence_msg)
                
                self.get_logger().info(
                    f"Detection: {best_detection['class']} at "
                    f"({best_detection['pixel_x']:.0f}, {best_detection['pixel_y']:.0f}) "
                    f"with confidence {best_detection['confidence']:.2f}"
                )
            else:
                # No detections found
                self.get_logger().debug("No target objects detected in snapshot")
        
        except Exception as e:
            if hasattr(self, 'get_logger'):
                self.get_logger().error(f"Error processing snapshot: {e}")
            else:
                print(f"Error processing snapshot: {e}")


def main():
    rclpy.init()
    try:
        node = YOLOInference()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

