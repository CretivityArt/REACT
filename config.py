"""
config.py
=========
Central configuration for the REACT Mine Simulation.

Everything a user should reasonably want to tune lives here so that
main.py and the other modules never need to be touched for routine
experiments (different videos, different scale factors, turning YOLO
on/off, etc).

READ THIS FIRST — HONESTY NOTE
-------------------------------
METERS_PER_FRAME is a *guess*. A single (monocular) camera cannot
recover true metric scale from video alone -- classic "scale
ambiguity" in monocular visual odometry / SLAM. Everything downstream
(map size, sensor readings, hazard locations) inherits whatever error
is baked into this constant. If you know the rover's real forward
speed and the video's FPS, set METERS_PER_FRAME = real_speed_mps / fps
for a much better (but still not exact) approximation. Better options
long-term: wheel encoders, stereo camera, or an IMU (see the
"Suggested Improvements" section printed at the end of a run).
"""

import os

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

VIDEO_PATH = os.path.join(BASE_DIR, "input", "tunnel_video.mp4")

OUTPUT_DIR = os.path.join(BASE_DIR, "output")
FINAL_MAP_PATH = os.path.join(OUTPUT_DIR, "final_mine_map.png")
TRAJECTORY_CSV = os.path.join(OUTPUT_DIR, "trajectory.csv")
DETECTIONS_CSV = os.path.join(OUTPUT_DIR, "detections.csv")
HAZARDS_CSV = os.path.join(OUTPUT_DIR, "hazards.csv")

YOLO_MODEL_PATH = os.path.join(BASE_DIR, "models", "yolov8n.pt")
# Optional custom mine-specific weights (fire/smoke/person/obstacle).
# If this file does not exist, the code automatically falls back to the
# standard COCO YOLO model and simulates fire/smoke placeholder logic.
CUSTOM_MODEL_PATH = os.path.join(BASE_DIR, "models", "mine_custom.pt")

# ---------------------------------------------------------------------------
# Visual Odometry / Scale
# ---------------------------------------------------------------------------
# SIMULATED scale factor: "meters" of forward travel the rover is assumed
# to cover per video frame of *detected forward motion*. This is a stand-in
# for real metric scale, which monocular vision cannot provide on its own.
METERS_PER_FRAME = 0.05

# Minimum number of good ORB feature matches required to trust a frame's
# motion estimate. Below this, we hold the previous pose (treat as noise).
MIN_MATCH_COUNT = 15

# Max features tracked per frame.
ORB_N_FEATURES = 800

# Resize video frames to this width before processing (speed vs accuracy).
PROCESS_WIDTH = 640

# ---------------------------------------------------------------------------
# Map
# ---------------------------------------------------------------------------
# Half-width/height (in simulated meters) of the local map window kept
# around the rover's full trajectory when plotting.
MAP_SIZE = 100
MAP_PADDING = 5  # meters of extra margin drawn around all plotted content

# ---------------------------------------------------------------------------
# Simulated Ultrasonic Sensor
# ---------------------------------------------------------------------------
SIMULATE_ULTRASONIC = True
# Max range the simulated ultrasonic sensor can "see" (meters).
ULTRASONIC_MAX_RANGE = 8.0
# Baseline tunnel half-width assumption (meters) used when wall/edge
# detection from the video frame is inconclusive.
DEFAULT_TUNNEL_HALF_WIDTH = 2.0

# ---------------------------------------------------------------------------
# Object Detection (YOLO)
# ---------------------------------------------------------------------------
ENABLE_YOLO = True
YOLO_CONFIDENCE_THRESHOLD = 0.35
# Standard COCO classes we care about for a mine-rescue context.
# ("fire" and "smoke" are NOT standard COCO classes -- see
# object_detection.py for the placeholder logic that flags this.)
RELEVANT_COCO_CLASSES = {"person", "car", "truck", "backpack", "suitcase"}

# Assumed real-world height (meters) of a standing/crouching person, used
# for a crude pinhole-camera distance estimate from bounding-box height.
ASSUMED_PERSON_HEIGHT_M = 1.6
# Assumed focal length in pixels (very rough default; recalibrate per
# camera for real accuracy). This is NOT a substitute for real camera
# calibration (see coordinate_transform.py docstring).
ASSUMED_FOCAL_PX = 700

# ---------------------------------------------------------------------------
# Simulated Hazards (gas / fire / temperature)
# ---------------------------------------------------------------------------
SIMULATE_GAS = True

# Ambient/safe baseline environment values.
BASELINE_ENVIRONMENT = {
    "oxygen": 20.9,      # %
    "methane": 0.0,      # %
    "co": 0,              # ppm
    "co2": 400,            # ppm
    "temperature": 22.0,  # C
}

# Predefined hazard zones in the rover's LOCAL/GLOBAL map coordinate
# system (meters, same frame as rover_x / rover_y). Radius is meters.
# type must be one of: "low_oxygen", "methane", "fire", "co"
HAZARD_ZONES = [
    {"id": "ZoneB", "type": "low_oxygen", "x": 6.0, "y": 1.0, "radius": 2.0},
    {"id": "ZoneC", "type": "methane", "x": 12.0, "y": -1.5, "radius": 2.5},
    {"id": "ZoneD", "type": "fire", "x": 18.0, "y": 0.5, "radius": 1.5},
]

# Distance (meters) at which the rover is considered to have "entered"
# a hazard zone and should raise a warning.
HAZARD_WARNING_RADIUS_MULTIPLIER = 1.3  # warn at 1.3x the zone radius

# ---------------------------------------------------------------------------
# Display / Runtime
# ---------------------------------------------------------------------------
SHOW_VIDEO = True
SHOW_MAP = True
# Save a dashboard screenshot every N frames (0 disables periodic saves).
SAVE_EVERY_N_FRAMES = 0
# Process every Nth frame (>1 skips frames to speed up long videos).
FRAME_STRIDE = 1
