# REACT Mine Simulation

A **software-only prototype/simulation** of the mapping and hazard-localization
system for REACT (Reactive Hazard Exploration and Assessment Technology), a
disaster-response mine rover concept.

> **This is a simulation/proof-of-concept, not a certified mine-safety
> instrument.** It processes an ordinary video (no LiDAR, GPS, ultrasonic,
> or gas-sensor hardware required) and *simulates* those sensors in
> software using reasonable, clearly-labeled assumptions. See
> "Honesty / Limitations" below before relying on any number it prints.

---

## 1. What it does

Feed it a video of a tunnel/mine environment (the rover's front camera
feed) and it will, frame by frame:

1. Estimate the rover's motion and position using **monocular visual
   odometry** (ORB features + Essential Matrix).
2. Simulate **ultrasonic front/left/right distance readings** from image
   edge geometry.
3. Run **YOLO object detection** (person, vehicle, etc.) plus a clearly
   labeled **placeholder fire/smoke heuristic** (color-based, since
   standard YOLO has no fire/smoke classes).
4. Transform every detection into the rover's global map coordinate
   frame.
5. Simulate an **environmental hazard field** (O2, CH4, CO, CO2,
   temperature) based on predefined hazard zones.
6. Render a **live dashboard**: annotated video (left) + growing 2D
   top-down map (right) + a sensor/status text panel (bottom).
7. Save a final map image and CSV logs when the video ends.

---

## 2. Project structure

```
REACT_Mine_Simulation/
├── main.py                  # entry point
├── config.py                # all tunable parameters
├── video_processor.py       # per-frame pipeline orchestration
├── visual_odometry.py       # ORB + Essential Matrix monocular VO
├── object_detection.py      # YOLO wrapper + fire/smoke placeholder
├── simulated_sensors.py     # simulated ultrasonic array
├── hazard_simulation.py     # simulated gas/fire/temperature hazards
├── mapping.py                # live 2D map data structure
├── coordinate_transform.py  # local<->global frame math
├── visualization.py          # dashboard rendering (OpenCV + Matplotlib)
├── models/                   # put yolov8n.pt / your custom model here
├── input/                    # put tunnel_video.mp4 here
└── output/                   # final_mine_map.png + CSV logs land here
```

Every file has a docstring explaining exactly what's real (derived from
video pixels) vs. simulated (an assumed constant or heuristic).

---

## 3. Installation (Windows)

```bash
python -m venv venv
venv\Scripts\activate
pip install opencv-python numpy matplotlib pandas ultralytics
```

(If you don't need real YOLO detection, you can skip `ultralytics` --
the program will fall back to the fire/smoke placeholder heuristic only
and print a warning.)

The first time you run the program with `ENABLE_YOLO = True` and no
model file present, **Ultralytics will auto-download `yolov8n.pt`** to
its cache and/or the working directory. To pin a specific model instead:

1. Download a YOLOv8 weights file (e.g. `yolov8n.pt`) from
   https://github.com/ultralytics/assets/releases
2. Place it at `models/yolov8n.pt` (matches `config.YOLO_MODEL_PATH`).

To use your **own mine-specific model** (trained with real fire/smoke/
obstacle/person classes), place its weights at `models/mine_custom.pt`
(matches `config.CUSTOM_MODEL_PATH`) -- it is picked up automatically
and the color-based fire/smoke placeholder is disabled since your
trained model presumably already covers those classes.

---

## 4. Running it

Put your tunnel video at `input/tunnel_video.mp4`, then:

```bash
python main.py
```

or point it at a different file:

```bash
python main.py path\to\other_video.mp4
```

A window titled **"REACT Mine Simulation Dashboard"** opens showing the
video (left, with detection boxes), the growing 2D map (right), and the
sensor panel (bottom). Press **`q`** in that window to stop early.

When the video ends (or you press `q`), the program writes to `output/`:

- `final_mine_map.png` -- a clean Matplotlib rendering of the whole map
- `trajectory.csv` -- rover pose per frame
- `detections.csv` -- every object detection per frame
- `hazards.csv` -- hazard-zone warnings/readings per frame

---

## 5. Configuration (`config.py`)

Key knobs:

| Setting | What it controls |
|---|---|
| `VIDEO_PATH` | default input video |
| `METERS_PER_FRAME` | **simulated** forward-motion scale per frame (see limitations) |
| `MIN_MATCH_COUNT` | how many ORB matches are required to trust a motion estimate |
| `ENABLE_YOLO` / `CUSTOM_MODEL_PATH` | turn detection on/off, use your own model |
| `SIMULATE_ULTRASONIC` / `SIMULATE_GAS` | toggle each simulated sensor subsystem |
| `HAZARD_ZONES` | list of simulated hazard zones (position, radius, type) |
| `SHOW_VIDEO` / `SHOW_MAP` | toggle the live dashboard window |
| `FRAME_STRIDE` | process every Nth frame, for speed on long videos |

---

## 6. Honesty / Limitations (please read)

This system explicitly **does not** have access to:

- Real GPS or metric ground-truth position
- Real gas/oxygen sensor readings
- Real acoustic ultrasonic/LiDAR ranging
- Calibrated camera intrinsics or true depth

Specifically:

- **Monocular scale ambiguity**: a single camera cannot recover true
  metric distance from feature-matching alone. `visual_odometry.py`
  recovers *direction* of motion honestly (from the Essential Matrix)
  but scales every step by the configurable `METERS_PER_FRAME`
  constant, not a measured quantity. Positions will drift from reality
  over a long video, and there's no loop closure to correct it.
- **Simulated ultrasonic** (`simulated_sensors.py`) uses image
  edge-density as a rough, unvalidated proxy for "closeness" -- it is
  not a physical distance sensor.
- **Simulated gas/hazard field** (`hazard_simulation.py`) is entirely
  generated from a predefined list of hazard zones you configure; it
  does not sense anything in the video.
- **Object distance estimates** (`object_detection.py`) use the classic
  monocular trick `distance ≈ real_height × focal_px / bbox_height_px`,
  which depends on assumed object height and an assumed (uncalibrated)
  focal length -- treat these as rough estimates only.
- **Fire/smoke detection** without a custom-trained model is a crude
  HSV color-threshold heuristic, clearly labeled `fire_placeholder` /
  `smoke_placeholder` in all outputs, and will have false
  positives/negatives. Train and supply a real model at
  `models/mine_custom.pt` for anything beyond a demo.

## 7. Suggested next steps toward a more accurate real system

1. Stereo or RGB-D camera for real calibrated depth
2. Real IMU integration
3. Wheel encoder integration for real odometry scale
4. Real ultrasonic/LiDAR hardware
5. Real calibrated gas sensors
6. RTAB-Map or ORB-SLAM3 for full visual SLAM with loop closure
7. ROS 2 integration for sensor fusion / robot middleware
8. Proper camera calibration (intrinsics/distortion)
9. Learned monocular/stereo depth estimation
10. A mine-specific YOLO model trained on real fire/smoke data
11. EKF/UKF sensor fusion (VO + IMU + encoders)
12. Loop closure detection to correct drift
13. True probabilistic occupancy-grid mapping
