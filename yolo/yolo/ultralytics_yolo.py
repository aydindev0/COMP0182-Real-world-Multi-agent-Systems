#!/usr/bin/env python3
from __future__ import annotations

"""
Ultralytics YOLO Detection Node

Integrates YOLOv8 object detection with the object_nav pipeline.
Publishes detections in the same topic structure that the placeholder node used:
  - /yolo/detections/class (std_msgs/String)
  - /yolo/detections/pixel (geometry_msgs/Point)
  - /yolo/detections/confidence (std_msgs/Float64)

Key features:
  * Optional simulation mode (matches original placeholder behaviour)
  * Configurable snapshot rate and detection thresholds
  * Filters detections to the desired COCO class IDs
  * Provides friendly class labels for downstream consumers
"""
import random
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String

try:
    from ultralytics import YOLO  # type: ignore

    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False


DEFAULT_CLASS_MAP = {
    63: "Laptop",
    67: "Mobile Phone",
    73: "Book",
}


class UltralyticsYoloNode(Node):
    """
    ROS 2 node that wraps the Ultralytics YOLOv8 model and exposes detections
    to the existing object_nav scaffold.
    """

    def __init__(self, *, node_name: str = "ultralytics_yolo", default_simulate: bool = False):
        super().__init__(node_name)

        # --- Declare parameters -------------------------------------------------
        self.declare_parameter("detection_topic", "/yolo/detections")
        self.declare_parameter("image_topic", "/ceiling_camera/image_raw")
        self.declare_parameter("snapshot_rate", 1.0)  # Hz
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("target_class_ids", list(DEFAULT_CLASS_MAP.keys()))
        self.declare_parameter("class_label_overrides", [])
        self.declare_parameter("publish_all_detections", False)
        self.declare_parameter("simulate_detections", default_simulate)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)

        # --- Retrieve parameters ------------------------------------------------
        self.detection_topic: str = self.get_parameter("detection_topic").value
        self.image_topic: str = self.get_parameter("image_topic").value
        self.snapshot_rate: float = float(self.get_parameter("snapshot_rate").value)
        self.model_path: str = self.get_parameter("model_path").value
        self.confidence_threshold: float = float(self.get_parameter("confidence_threshold").value)
        self.image_size: int = int(self.get_parameter("image_size").value)
        self.publish_all_detections: bool = bool(self.get_parameter("publish_all_detections").value)
        self.simulate: bool = bool(self.get_parameter("simulate_detections").value)
        self.image_width: int = int(self.get_parameter("image_width").value)
        self.image_height: int = int(self.get_parameter("image_height").value)

        target_ids_param = self.get_parameter("target_class_ids").value
        self.target_class_ids: List[int] = [int(x) for x in target_ids_param] if target_ids_param else list(DEFAULT_CLASS_MAP.keys())

        self.class_labels: Dict[int, str] = DEFAULT_CLASS_MAP.copy()
        overrides = self.get_parameter("class_label_overrides").value or []
        for entry in overrides:
            try:
                class_id_str, label = entry.split(":", maxsplit=1)
                class_id = int(class_id_str.strip())
                self.class_labels[class_id] = label.strip()
            except ValueError:
                self.get_logger().warn(f"Ignoring invalid class_label_overrides entry: '{entry}'")

        # --- Internal state -----------------------------------------------------
        self.bridge = CvBridge()
        self.current_image: Optional[np.ndarray] = None
        self.image_received = False

        # --- Publishers ---------------------------------------------------------
        detection_base = self.detection_topic.rstrip("/")
        self.class_pub = self.create_publisher(String, f"{detection_base}/class", 10)
        self.pixel_pub = self.create_publisher(Point, f"{detection_base}/pixel", 10)
        self.confidence_pub = self.create_publisher(Float64, f"{detection_base}/confidence", 10)

        # --- Subscriptions ------------------------------------------------------
        if not self.simulate:
            self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        # --- Load model if required ---------------------------------------------
        self.model: Optional[YOLO] = None
        if not self.simulate:
            if not ULTRALYTICS_AVAILABLE:
                self.get_logger().error(
                    "ultralytics package not found. Install with: pip install ultralytics\n"
                    "Falling back to simulated detections."
                )
                self.simulate = True
            else:
                try:
                    self.model = YOLO(self.model_path)
                    self.get_logger().info(f"Loaded Ultralytics model '{self.model_path}'")
                    if hasattr(self.model, "names"):
                        self._update_labels_from_model(self.model.names)
                except Exception as exc:  # pylint: disable=broad-except
                    self.get_logger().error(f"Failed to load model '{self.model_path}': {exc}")
                    self.get_logger().error("Falling back to simulated detections.")
                    self.simulate = True

        # --- Timers -------------------------------------------------------------
        period = 1.0 / self.snapshot_rate if self.snapshot_rate > 0 else 1.0
        self.create_timer(period, self._timer_callback)

        # --- Log configuration --------------------------------------------------
        self.get_logger().info("Ultralytics YOLO node initialised")
        self.get_logger().info(f"Mode: {'simulation' if self.simulate else 'inference'}")
        self.get_logger().info(f"Detection topic base: {detection_base}")
        self.get_logger().info(f"Target class IDs: {self.target_class_ids}")
        self.get_logger().info(f"Class labels: {self.class_labels}")

    # -------------------------------------------------------------------------
    # Helper methods
    # -------------------------------------------------------------------------
    def _update_labels_from_model(self, model_names) -> None:
        """If the Ultralytics model provides class names, refresh label map."""
        if isinstance(model_names, dict):
            for class_id, label in model_names.items():
                if int(class_id) in self.target_class_ids:
                    self.class_labels[int(class_id)] = label.title()

    def image_callback(self, msg: Image) -> None:
        """Store the latest camera frame."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.current_image = cv_image
            self.image_received = True
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert image: {exc}")
            self.image_received = False

    def _timer_callback(self) -> None:
        """Dispatch to inference or simulation depending on mode."""
        if self.simulate:
            self.publish_simulated_detection()
        else:
            self.run_inference()

    def publish_simulated_detection(self) -> None:
        """Generate a random detection to keep downstream pipeline exercised."""
        class_id = random.choice(self.target_class_ids)
        class_name = self.class_labels.get(class_id, str(class_id))

        center_x = self.image_width // 2
        center_y = self.image_height // 2

        pixel_x = random.randint(max(0, center_x - 100), min(self.image_width, center_x + 100))
        pixel_y = random.randint(max(0, center_y - 100), min(self.image_height, center_y + 100))
        confidence = random.uniform(0.7, 0.99)

        self._publish_detection(class_name, float(pixel_x), float(pixel_y), confidence)
        self.get_logger().info(
            f"[SIM] Detection: {class_name} at ({pixel_x}, {pixel_y}) conf={confidence:.2f}"
        )

    def run_inference(self) -> None:
        """Perform YOLO inference on the latest camera frame."""
        if not self.image_received or self.current_image is None or self.model is None:
            return

        frame = self.current_image.copy()

        try:
            results = self.model.predict(
                frame,
                conf=self.confidence_threshold,
                imgsz=self.image_size,
                classes=self.target_class_ids,
                verbose=False,
            )
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"Ultralytics inference failed: {exc}")
            return

        detections = self._extract_detections(results)
        if not detections:
            self.get_logger().debug("No target detections found in frame")
            return

        if self.publish_all_detections:
            for detection in detections:
                self._publish_detection(*detection)
        else:
            best_detection = max(detections, key=lambda det: det[3])
            self._publish_detection(*best_detection)

    def _extract_detections(self, results) -> List[Tuple[str, float, float, float]]:
        """
        Convert Ultralytics results into a list of tuples:
        (class_name, pixel_x, pixel_y, confidence)
        """
        parsed: List[Tuple[str, float, float, float]] = []

        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue

            for box in boxes:
                class_tensor = getattr(box, "cls", None)
                conf_tensor = getattr(box, "conf", None)
                xyxy = getattr(box, "xyxy", None)
                if class_tensor is None or conf_tensor is None or xyxy is None:
                    continue

                class_id = int(class_tensor[0])
                confidence = float(conf_tensor[0])

                if class_id not in self.target_class_ids:
                    continue

                coords = xyxy[0].cpu().numpy() if hasattr(xyxy[0], "cpu") else np.array(xyxy[0])
                x1, y1, x2, y2 = coords.tolist()
                center_x = (x1 + x2) / 2.0
                center_y = (y1 + y2) / 2.0

                class_name = self.class_labels.get(class_id, f"Class {class_id}")
                parsed.append((class_name, float(center_x), float(center_y), confidence))

        return parsed

    def _publish_detection(self, class_name: str, pixel_x: float, pixel_y: float, confidence: float) -> None:
        """Publish detection components on their respective topics."""
        class_msg = String()
        class_msg.data = class_name
        self.class_pub.publish(class_msg)

        pixel_msg = Point()
        pixel_msg.x = pixel_x
        pixel_msg.y = pixel_y
        pixel_msg.z = 0.0
        self.pixel_pub.publish(pixel_msg)

        confidence_msg = Float64()
        confidence_msg.data = float(confidence)
        self.confidence_pub.publish(confidence_msg)

        self.get_logger().info(
            f"Detection: {class_name} at ({pixel_x:.0f}, {pixel_y:.0f}) conf={confidence:.2f}"
        )


def main() -> None:
    rclpy.init()
    node = UltralyticsYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()