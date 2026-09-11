"""
==============================================================================
REACT (Reactive Hazard Exploration/Response Rover)
Global Configuration File: config.py
==============================================================================
Centralizes all hardware settings, geometric coordinates, sensor parameters,
occupancy grid mapping specifications, and filter tuning.
"""

import math

# ----------------------------------------------------------------------------
# 1. SYSTEM OPERATION MODE
# ----------------------------------------------------------------------------
# Set to True to test mapping and visualization without physical hardware.
# Set to False to connect to the actual Arduino Uno over USB Serial.
SIMULATION_MODE = False

# ----------------------------------------------------------------------------
# 2. SERIAL COMMUNICATION SETTINGS
# ----------------------------------------------------------------------------
SERIAL_PORT = "COM5"            # Windows: 'COM3', 'COM4' | Linux: '/dev/ttyUSB0', '/dev/ttyACM0'
BAUD_RATE = 9600                # 9600 baud rate matching Arduino sketch
SERIAL_TIMEOUT = 1.0           # Seconds before read timeout
RECONNECT_INTERVAL = 2.0       # Seconds between reconnection attempts if port drops
SENSOR_TIMEOUT_SECONDS = 2.5   # Trigger warning if no valid packet received

# ----------------------------------------------------------------------------
# 3. ROVER PHYSICAL GEOMETRY (Dimensions in meters)
# ----------------------------------------------------------------------------
ROVER_LENGTH = 0.35            # Length along X-axis (forward/backward) in meters
ROVER_WIDTH = 0.25             # Width along Y-axis (left/right) in meters
WHEEL_BASE = 0.22              # Distance between left and right drive wheels
WHEEL_RADIUS = 0.045           # Drive wheel radius (45 mm)

# ----------------------------------------------------------------------------
# 4. ULTRASONIC SENSORS CONFIGURATION (3x SENSORS: FRONT, LEFT, RIGHT)
# ----------------------------------------------------------------------------
# Coordinate convention for the rover frame:
#   +X = Forward along vehicle centerline
#   +Y = Left along vehicle lateral axis
# Angles measured counter-clockwise from the +X axis (in degrees):
#   FRONT: 0.0 deg    (mounted at front center, pointing straight ahead in +X)
#   LEFT : +90.0 deg  (mounted at left flank, pointing perpendicular to left in +Y)
#   RIGHT: -90.0 deg  (mounted at right flank, pointing perpendicular to right in -Y)
#
# Offsets (offset_x, offset_y) define the physical mounting point of the transducer
# relative to the rover's geometric center (in meters).
SENSOR_CONFIG = {
    "FRONT": {
        "name": "Front Ultrasonic",
        "angle": 0.0,                               # 0 deg (facing straight forward)
        "angle_rad": 0.0,
        "offset_x": ROVER_LENGTH / 2.0,             # +0.175 m (front bumper)
        "offset_y": 0.0,                            # 0.0 m (centerline)
        "min_dist": 0.03,                           # 3 cm minimum HC-SR04 blind zone
        "max_dist": 4.00,                           # 4 meters maximum reliable range
        "beam_angle_deg": 15.0                      # Ultrasonic cone spread
    },
    "LEFT": {
        "name": "Left Ultrasonic (Perpendicular)",
        "angle": 90.0,                              # +90 deg (perpendicular left)
        "angle_rad": math.radians(90.0),
        "offset_x": 0.0,                            # Center of vehicle along X
        "offset_y": ROVER_WIDTH / 2.0,              # +0.125 m (left flank)
        "min_dist": 0.03,
        "max_dist": 4.00,
        "beam_angle_deg": 15.0
    },
    "RIGHT": {
        "name": "Right Ultrasonic (Perpendicular)",
        "angle": -90.0,                             # -90 deg (perpendicular right)
        "angle_rad": math.radians(-90.0),
        "offset_x": 0.0,                            # Center of vehicle along X
        "offset_y": -ROVER_WIDTH / 2.0,             # -0.125 m (right flank)
        "min_dist": 0.03,
        "max_dist": 4.00,
        "beam_angle_deg": 15.0
    }
}

# Aliases for quick indexing and backward compatibility
SENSOR_ALIASES = {
    "F": "FRONT", "DF": "FRONT", "D1": "FRONT",
    "L": "LEFT",  "DL": "LEFT",  "D2": "LEFT",
    "R": "RIGHT", "DR": "RIGHT", "D3": "RIGHT"
}

# General ultrasonic filter parameters
MAX_ULTRASONIC_DISTANCE = 4.0  # Meters (readings above this are discarded)
MIN_ULTRASONIC_DISTANCE = 0.03  # Meters (readings below 3 cm are noise/blind zone)
ULTRASONIC_MEDIAN_WINDOW = 5   # Window size for rolling median noise filter

# ----------------------------------------------------------------------------
# 5. 2D OCCUPANCY GRID MAP SETTINGS
# ----------------------------------------------------------------------------
MAP_RESOLUTION = 0.05          # 5 cm (0.05 meters) per grid cell
MAP_WIDTH_METERS = 20.0        # Total map width (East-West) in meters
MAP_HEIGHT_METERS = 20.0       # Total map height (North-South) in meters

# Grid cell states
CELL_UNKNOWN = 0.5             # Prior probability P(occ) = 0.5 (log-odds = 0)
CELL_FREE = 0.1                # Free space probability
CELL_OCCUPIED = 0.9            # Solid obstacle probability

# Log-odds update values for Bayesian mapping
LOG_ODDS_FREE = -0.4           # Decrement cell probability along ray
LOG_ODDS_OCCUPIED = 0.85       # Increment cell probability at detected obstacle
LOG_ODDS_MAX = 5.0             # Saturation clamp
LOG_ODDS_MIN = -5.0

# ----------------------------------------------------------------------------
# 6. SENSOR FUSION & HEADING SETTINGS (MPU-6050 IMU + GY-271 COMPASS)
# ----------------------------------------------------------------------------
USE_MPU6050 = True             # Enable MPU-6050 6-Axis MotionTracking IMU
# When USE_GY271_HEADING is False or GY-271 is absent, the system uses MPU-6050
# Gyroscope Z rate integration with accelerometer tilt compensation.
# When True, it uses the GY-271 magnetic compass for absolute azimuth.
USE_GY271_HEADING = False      # Default to MPU-6050 heading (set True if GY-271 is attached)
COMPLEMENTARY_ALPHA = 0.96     # Complementary filter weight (0.90 - 0.98) between Gyro and Mag

# Magnetometer calibration parameters (Hard-iron offset correction)
# Calibrated values: corrected = (raw - offset) * scale
MAG_OFFSET_X = 0.0
MAG_OFFSET_Y = 0.0
MAG_OFFSET_Z = 0.0
MAG_SCALE_X = 1.0
MAG_SCALE_Y = 1.0
MAG_SCALE_Z = 1.0

# Local magnetic declination angle (adjust for your mine/test location in degrees)
# e.g., +0.5 deg East
MAGNETIC_DECLINATION_DEG = 0.5 # Degrees East (+) or West (-)

# Gyroscope zero-rate bias offsets (estimated at stationary startup)
GYRO_BIAS_X = 0.0              # deg/s
GYRO_BIAS_Y = 0.0              # deg/s
GYRO_BIAS_Z = 0.0              # deg/s

# ----------------------------------------------------------------------------
# 7. EXECUTION & UPDATE RATES
# ----------------------------------------------------------------------------
IMU_UPDATE_RATE = 50           # Hz (target internal filter rate)
MAPPING_UPDATE_RATE = 15       # Hz (occupancy grid insertion rate)
VIS_UPDATE_RATE = 20           # Hz (Matplotlib/PyQt refresh rate)
QUEUE_MAX_SIZE = 100           # Thread-safe packet buffer limit
