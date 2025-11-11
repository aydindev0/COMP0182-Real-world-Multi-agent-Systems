#!/usr/bin/env python3
"""
Standalone YOLOv8 webcam tester.

Streams frames from the default camera, runs Ultralytics YOLO detection on a
small set of COCO classes, displays the annotated image, and optionally saves
snapshots when detections occur.
"""
import argparse
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import cv2
from ultralytics import YOLO

# COCO classes of interest
DEFAULT_CLASS_IDS: List[int] = [63, 3, 11, 67, 73]  # Laptop, Motorcycle, Stop sign, Cell Phone, Book


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Simple YOLOv8 webcam tester.")
    parser.add_argument(
        "--camera",
        type=str,
        default="/dev/video0",
        help="Camera device or index passed to OpenCV (default: /dev/video0)",
    )
    parser.add_argument(
        "--model",
        type=str,
        default="yolov8n.pt",
        help="Path to YOLOv8 model weights (default: yolov8n.pt, cached in ~/.cache/ultralytics/)",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.15,
        help="Confidence threshold for detections (default: 0.15)",
    )
    parser.add_argument(
        "--save-dir",
        type=Path,
        default=None,
        help="Optional directory to store detection snapshots.",
    )
    parser.add_argument(
        "--class-ids",
        type=int,
        nargs="+",
        default=DEFAULT_CLASS_IDS,
        help=f"COCO class IDs to detect (default: {DEFAULT_CLASS_IDS})",
    )
    parser.add_argument(
        "--disable-autofocus",
        action="store_true",
        help="Disable continuous autofocus via v4l2-ctl before streaming.",
    )
    parser.add_argument(
        "--preset-logitech",
        action="store_true",
        help="Apply Logitech C920-style settings (disable AF, focus=60, exposure fix, white balance 3000K).",
    )
    parser.add_argument(
        "--zoom",
        type=int,
        default=200,
        help="Zoom level for cameras that expose zoom_absolute (default 200; 100=no zoom).",
    )
    return parser.parse_args()


def ensure_model(model_path: str) -> str:
    """
    Ensure the requested model exists locally. Models are cached in ~/.cache/ultralytics/
    by the ultralytics library automatically.
    """
    path = Path(model_path)
    
    # If full path provided and exists, use it
    if path.exists():
        return str(path)
    
    # Otherwise just return the model name - ultralytics will handle caching
    # Models are automatically downloaded to ~/.cache/ultralytics/
    print(f"[INFO] Using model: {path.name} (will be cached in ~/.cache/ultralytics/)")
    return path.name


def open_camera(camera: str) -> cv2.VideoCapture:
    """
    Open a camera using either a device path or an integer index string.
    """
    if camera.isdigit():
        return cv2.VideoCapture(int(camera))
    return cv2.VideoCapture(camera, cv2.CAP_V4L2)


def set_v4l2_control(device: str, control: str, value: str, success_msg: str) -> bool:
    try:
        subprocess.run(
            ["v4l2-ctl", "-d", device, "--set-ctrl", f"{control}={value}"],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        print(success_msg)
        return True
    except FileNotFoundError:
        print("[WARN] v4l2-ctl not found. Install v4l-utils to use camera control options.")
    except subprocess.CalledProcessError:
        print(f"[WARN] Failed to set '{control}' on {device}. The control may be unsupported.")
    return False


def configure_camera_controls(args, device: Optional[str]) -> None:
    if device is None:
        return

    if args.disable_autofocus or args.preset_logitech:
        set_v4l2_control(
            device,
            "focus_automatic_continuous",
            "0",
            f"[INFO] Autofocus disabled on {device}.",
        )

    if args.preset_logitech:
        # Manual focus
        set_v4l2_control(
            device,
            "focus_absolute",
            "60",
            "[INFO] Focus set to 60.",
        )
        # Zoom
        if args.zoom is not None:
            set_v4l2_control(
                device,
                "zoom_absolute",
                str(args.zoom),
                f"[INFO] Zoom set to {args.zoom}.",
            )
        # Manual exposure
        set_v4l2_control(
            device,
            "auto_exposure",
            "1",
            "[INFO] Exposure set to manual mode.",
        )
        set_v4l2_control(
            device,
            "exposure_time_absolute",
            "166",
            "[INFO] Exposure time set to 166.",
        )
        # Manual white balance
        set_v4l2_control(
            device,
            "white_balance_automatic",
            "0",
            "[INFO] White balance set to manual.",
        )
        set_v4l2_control(
            device,
            "white_balance_temperature",
            "3000",
            "[INFO] White balance temperature set to 3000K.",
        )


def main() -> int:
    args = parse_args()
    model_path = ensure_model(args.model)

    control_device = args.camera if args.camera.startswith("/dev/") else None
    configure_camera_controls(args, control_device)

    # Load YOLO model
    try:
        model = YOLO(model_path)
    except Exception as exc:  # pylint: disable=broad-except
        print(f"[ERROR] Failed to load model '{model_path}': {exc}")
        return 1

    # Initialise camera
    cap = open_camera(str(args.camera))
    if not cap.isOpened():
        print(f"[ERROR] Could not open camera source {args.camera}.")
        return 1

    actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[INFO] Opened camera source {args.camera} ({actual_width:.0f}x{actual_height:.0f} @ {actual_fps:.1f}fps)")

    # Warm-up frames to allow exposure adjustments
    for _ in range(5):
        cap.read()

    print("[INFO] Press 'q' to quit, 's' to save a snapshot of the current frame.")

    # Prepare snapshot directory if requested
    save_dir = None
    if args.save_dir is not None:
        save_dir = Path(args.save_dir).expanduser().resolve()
        save_dir.mkdir(parents=True, exist_ok=True)
        print(f"[INFO] Snapshots will be saved to: {save_dir}")

    last_reported: set[str] = set()

    try:
        while True:
            success, frame = cap.read()
            if not success:
                print("[WARN] Failed to read frame from camera.")
                break

            # Run YOLO detection
            results = model.predict(
                frame,
                classes=args.class_ids,
                conf=args.conf,
                verbose=False,
            )

            detections = []
            if results and results[0].boxes is not None:
                names = results[0].names
                for box in results[0].boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    if args.class_ids and cls_id not in args.class_ids:
                        continue
                    if isinstance(names, dict):
                        label = names.get(cls_id, f"class_{cls_id}")
                    elif isinstance(names, list) and cls_id < len(names):
                        label = names[cls_id]
                    else:
                        label = f"class_{cls_id}"
                    detections.append((label, conf))

            current_report = {label for label, _ in detections}
            if current_report and current_report != last_reported:
                summary = ", ".join(f"{label} ({conf:.2f})" for label, conf in detections)
                print(f"[DETECTION] {summary}")
                last_reported = current_report
            elif not current_report and last_reported:
                print("[DETECTION] Targets no longer visible.")
                last_reported = set()

            annotated = results[0].plot()
            cv2.imshow("YOLOv8 Detection", annotated)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s") and save_dir is not None:
                timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S_%f")
                output_path = save_dir / f"detection_{timestamp}.png"
                cv2.imwrite(str(output_path), annotated)
                print(f"[INFO] Saved snapshot: {output_path}")

    finally:
        cap.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())
