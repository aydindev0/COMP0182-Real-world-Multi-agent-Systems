# COMP0182 Real-world Multi-agent Systems: Lab Sheet Six

In this lab you will step away from ROS and experiment with **Ultralytics YOLOv8** as a fast visual detector. Yolo(You Only Look Once) models run an entire image through a single neural network pass to predict bounding boxes and classes in real time. YOLOv8 (Ultralytics) provides ready-to-use weights, a simple Python API, and good performance on commodity GPUs or fast CPUs.


```bash
sudo apt update
sudo apt install python3-venv python3-pip v4l-utils
python3 -m pip install --upgrade pip
python3 -m pip install ultralytics opencv-python
```

---

## 1. Clone the repository

Clone the reposioty which contains the script ``yolo_test.py`` to run Yolo object detection
```
git clone --branch yolo @https://github.com/aydindev0/turtlebot_and_phasespace.git 
```

Read the `README.md` for extra camera tuning tips. The lab sheet summarises the key steps below.

---

## 2. Quick Introduction to YOLOv8

- **Single-stage detector:** YOLO predicts bounding boxes and classes in one forward pass.
- **Ultralytics API:** `YOLO('weights.pt')` loads a model; `.predict()` returns detections with bounding boxes, classes, and confidences.
- **COCO classes:** Pre-trained weights are trained on the COCO dataset (80 everyday object classes).
- **Real-time capability:** With modest confidence thresholds (e.g. 0.15) and lightweight models (`yolov8n`, `yolov8s`), you can get ~30 FPS on a modern CPU, faster on GPU.

During this lab you will configure a webcam, run the detector, collect snapshots, and think about how you could fold this into your multi-robot perception pipeline.

---

## 3. Running the YOLO Tester

### 3.1 Basic launch

First identify the name of the camera device you are using:
```
    v4l2-ctl --list-devices
    ls /dev/video*
```

Navigate to the repository root (or `yolo_test/`) and run:

```bash

python3 yolo_test/yolo_test.py \
    --camera /dev/{your usb camera}\
    --model yolo_test/yolov8s.pt \
    --conf 0.15 \
    --preset-logitech
```

Key behaviour:
- If `yolov8s.pt` is missing, Ultralytics auto-downloads it on first run.
- A window named **YOLOv8 Detection** appears with coloured bounding boxes and class labels.
- Press `q` to quit, `s` to save the current frame.

### 3.2 Useful CLI options (all defined in `yolo_test.py`)

- `--camera`: Device path or numeric index. Examples: `/dev/video0`, `1`.
- `--model`: Path to YOLOv8 weights. Defaults to a local `yolov8s.pt`.
- `--conf`: Confidence threshold (0–1). Lower values show more detections.
- `--class-ids`: Space-separated COCO class IDs to filter on. Default: Laptop(63), Motorcycle(3), Stop Sign(11), Cell Phone(67), Book(73).
- `--disable-autofocus`: Turns off continuous autofocus via `v4l2-ctl` before streaming.
- `--preset-logitech`: Applies the tested Logitech C920-style settings (focus=60, zoom=200, fixed exposure/white balance).
- `--zoom`: Override zoom value when the camera supports `zoom_absolute`.
- `--save-dir`: Folder for snapshot PNGs when you press `s`.


---

## 4. Lab Activities

### 4.1 Verify camera access

1. Bring the camera online (optionally with `guvcview` to lock MJPG/YUYV format).
2. Run the YOLO tester with `--preset-logitech`.
3. Confirm detections appear and log messages print when classes are found:
   ```
   [DETECTION] {object name} (0.65)
   ```

### 4.2 Experiment with controls

- Toggle `--preset-logitech` vs `--disable-autofocus` to see how focus/exposure affect detection confidence.
- Adjust `--conf` between 0.10 and 0.40 to balance false positives vs. missed detections.
- Add or remove class IDs, e.g., `--class-ids 0 1` for person + bicycle.
- Consult the official COCO class index (e.g. https://cocodataset.org/#explore) or the Ultralytics YOLO Docs (https://docs.ultralytics.com/datasets/detect/coco/) to discover new object IDs to monitor and test how YOLO responds.
- Save snapshots with `--save-dir ~/yolo_snaps` and zip them for your lab report.

---

## 5. Reflection Questions

1. Which camera settings (exposure, focus, zoom) improved detection confidence the most?
2. How does lowering the confidence threshold affect precision vs. recall?
3. Given the class filter in the script, what happens if you pass an empty list (`--class-ids` omitted)?
4. How could you wrap this script into a ROS 2 node or publish detections for a multi-robot system?
5. What steps would be needed to fine-tune YOLOv8 on a custom dataset (e.g., TurtleBot-specific objects)?

Record your observations and answers in your lab notebook or group report.

---

## 6. Troubleshooting

- **Window opens but no detections:** Increase lighting, ensure the object is within the selected classes, lower `--conf`.
- **Camera fails to open:** Verify the device path, ensure no other processes are using it, and try a smaller resolution.
- **Continuous autofocus ignored:** Some webcams ignore software control—disable it using the manufacturer’s utility or manual focus ring.
- **Snapshots not saved:** Ensure `--save-dir` exists or let the script create it with proper write permissions.
- **Model load failure:** Confirm internet access or pre-download the weights into the `yolo_test` directory.

For additional camera tuning commands, see the `README.md` in `yolo_test/`.

---
