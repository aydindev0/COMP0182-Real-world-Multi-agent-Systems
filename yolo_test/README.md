# YOLO Webcam Smoke Test

Quick standalone script to verify Ultralytics YOLOv8 detection with a webcam.  
No ROS dependencies required; ideal for sanity-checking models before wiring into larger projects.

---

## Setup

1. Ensure Python 3.10+ and `pip` are available.
2. Install dependencies (preferably in a virtual environment):
   ```bash
   python3 -m pip install --upgrade pip
   python3 -m pip install ultralytics opencv-python
   ```
   For GPU acceleration, install a CUDA-enabled PyTorch build before `ultralytics`.
3. Install V4L2 utilities for manual focus/exposure control:
   ```bash
   sudo apt install v4l-utils
   ```
4. Optionally download `yolov8s.pt` to this directory (higher accuracy than `yolov8n.pt`).  
   Ultralytics will auto-download the weight file on first use if it is missing.

---

## Usage

```bash
python3 yolo_test.py --camera /dev/video3 --model yolov8s.pt --conf 0.15 --preset-logitech
```

CLI options:
- `--camera` – Device path or index for OpenCV (`/dev/video3`, `0`, etc.). Default `/dev/video3`.
- `--disable-autofocus` – Disable continuous autofocus before streaming.
- `--preset-logitech` – Apply the tested Logitech-oriented settings (disables AF, focus=60, zoom=200, exposure 166, white balance 3000K).
- `--zoom` – Zoom value (100 = no zoom). Defaults to 200 when the preset is used.
- `--model` – Path or name of YOLOv8 weights (default `yolov8s.pt` in this folder).
- `--conf` – Detection confidence threshold (default `0.15`).
- `--save-dir` – Optional directory for saving annotated frames when you press `s`.
- `--class-ids` – Space-separated COCO class IDs to monitor (defaults to Laptop 63, Motorcycle 3, Stop Sign 11, Mobile Phone 67, Book 73).

Keyboard controls:
- `q` – Quit.
- `s` – Save the current annotated frame (when `--save-dir` is provided).

---

## Notes

- The script uses `model.predict` with the class filter so it only reacts to the specified objects.
- If camera access fails, confirm the device path via `v4l2-ctl --list-devices` and ensure no other process is holding the stream. Many Logitech-style webcams expose a separate `/dev/video*` for MJPG.
- Some devices require a launcher to lock the camera into MJPG (or YUYV) before OpenCV connects. A reliable approach is:
  ```bash
  guvcview -d /dev/video3 -f MJPG -x 1280x720
  ```
  Leave the window open (or minimise it) while running the tester. Use the OpenCV viewer to verify framing, and fall back to command-line zoom controls (or `--zoom`) if needed. If MJPG is unsupported, try:
  ```bash
  gst-launch-1.0 v4l2src device=/dev/video3 ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! videoconvert ! autovideosink
  ```
- Start with `--preset-logitech` to lock the camera into the green/purple-free profile; if unsupported, fall back to `--disable-autofocus` alone. Tweak `--zoom` to adjust framing (100 = no zoom, 300 ≈ medium telephoto).
- Manual sequence equivalent to the preset (for reference):
  ```bash
  v4l2-ctl -d /dev/video3 --set-ctrl=focus_automatic_continuous=0
  v4l2-ctl -d /dev/video3 --set-ctrl=focus_absolute=60
  v4l2-ctl -d /dev/video3 --set-ctrl=zoom_absolute=200
  v4l2-ctl -d /dev/video3 --set-ctrl=auto_exposure=1
  v4l2-ctl -d /dev/video3 --set-ctrl=exposure_time_absolute=166
  v4l2-ctl -d /dev/video3 --set-ctrl=white_balance_automatic=0
  v4l2-ctl -d /dev/video3 --set-ctrl=white_balance_temperature=3000
  ```
- For borderline detections, adding motion (e.g., waving a hand or tilting the target) often increases confidence because it introduces additional frames for YOLO to aggregate.

### Why a quick wave helps – and how to avoid needing it

- **Automatic adjustments**: movement forces the webcam to refresh exposure, white balance, and contrast, often clearing up a dull frame.
- **Residual focus behaviour**: even with autofocus technically disabled, a nudge can land the lens in a sharper position.
- **Contrast boost**: a hand adds shadows and highlights, outlining the floor object more clearly.
- **Training data fidelity**: YOLO sees plenty of dynamic scenes; a static shot is less representative.
- **Frame variety**: motion produces a burst of slightly different frames, increasing the chance that one meets the detection threshold.

To make performance more consistent without relying on motion, lock exposure manually:
```bash
# Turn off automatic exposure
v4l2-ctl --device=/dev/video3 --set-ctrl=auto_exposure=1

# Pick an exposure time suited to your lighting (100–300 is typical)
v4l2-ctl --device=/dev/video3 --set-ctrl=exposure_time_absolute=200
```
Combine fixed exposure with the preset focus/zoom settings. If detections still dip, briefly waving a hand remains a quick way to kick the camera back into a favourable state.
- Saved snapshots (triggered with `s`) land in the folder supplied via `--save-dir`.

