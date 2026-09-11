import { CodeFile } from "../types";

export const CODE_FILES: CodeFile[] = [
  {
    name: "main.py",
    path: "react_mapping/main.py",
    category: "python",
    description: "Main orchestrator coordinating serial reading, signal filtering, IMU fusion, odometry, 2D mapping, and GUI.",
    language: "python",
    content: `"""
==============================================================================
REACT (Reactive Hazard Exploration/Response Rover)
Main Application Entry Point: main.py
==============================================================================
Orchestrates:
1. Thread-safe Serial Reader (USB Serial or Simulation Mode)
2. Digital signal filtering on 4 ultrasonic channels
3. 9-DOF IMU sensor calibration & tilt-compensated heading fusion
4. Dead-reckoning & kinematic odometry tracking
5. 2D Euclidean Coordinate Transformation (+X=forward, +Y=left -> Global Map)
6. 2D Bayesian Occupancy Grid & Obstacle Mapping
7. Real-time Matplotlib interactive Ground Control Station GUI
"""

import sys
import time
import math
import queue
import signal
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import config
from filters import MedianFilter, OutlierRejector
from coordinate_transform import CoordinateTransformer
from imu import IMUInterface
from sensor_fusion import HeadingEstimator
from odometry import OdometryEstimator
from mapping import OccupancyGridMap
from serial_reader import SerialReaderThread, TelemetryPacket
from visualization import RoverVisualizer


class ReactMappingSystem:
    """
    Main system coordinator for the REACT underground coal-mine exploration rover.
    """

    def __init__(self):
        print("=" * 70)
        print("REACT ROVER // 2D HAZARD MAPPING & TELEMETRY SYSTEM")
        print("Mode:", "SIMULATION" if config.SIMULATION_MODE else f"HARDWARE ({config.SERIAL_PORT} @ {config.BAUD_RATE})")
        print("=" * 70)

        # 1. Thread-safe Communication Buffer
        self.packet_queue = queue.Queue(maxsize=config.QUEUE_MAX_SIZE)

        # 2. Digital Signal Filters (one per ultrasonic transducer)
        self.median_filters = {
            "FL": MedianFilter(window_size=config.ULTRASONIC_MEDIAN_WINDOW),
            "FR": MedianFilter(window_size=config.ULTRASONIC_MEDIAN_WINDOW),
            "RL": MedianFilter(window_size=config.ULTRASONIC_MEDIAN_WINDOW),
            "RR": MedianFilter(window_size=config.ULTRASONIC_MEDIAN_WINDOW)
        }
        self.outlier_rejectors = {
            "FL": OutlierRejector(max_delta=1.8),
            "FR": OutlierRejector(max_delta=1.8),
            "RL": OutlierRejector(max_delta=1.8),
            "RR": OutlierRejector(max_delta=1.8)
        }

        # 3. IMU & Sensor Fusion Driver
        self.imu_interface = IMUInterface(
            gyro_bias_x=config.GYRO_BIAS_X,
            gyro_bias_y=config.GYRO_BIAS_Y,
            gyro_bias_z=config.GYRO_BIAS_Z,
            mag_offset_x=config.MAG_OFFSET_X,
            mag_offset_y=config.MAG_OFFSET_Y,
            mag_offset_z=config.MAG_OFFSET_Z
        )
        self.heading_estimator = HeadingEstimator(
            alpha=config.COMPLEMENTARY_ALPHA,
            magnetic_declination_deg=config.MAGNETIC_DECLINATION_DEG
        )

        # 4. Kinematic Odometry Estimator
        self.odometry = OdometryEstimator(
            wheel_base=config.WHEEL_BASE,
            wheel_radius=config.WHEEL_RADIUS
        )

        # 5. Occupancy Grid Map
        self.occupancy_map = OccupancyGridMap(
            resolution=config.MAP_RESOLUTION,
            width_m=config.MAP_WIDTH_METERS,
            height_m=config.MAP_HEIGHT_METERS
        )

        # 6. Serial Acquisition Thread
        self.reader_thread = SerialReaderThread(
            packet_queue=self.packet_queue,
            port=config.SERIAL_PORT,
            baudrate=config.BAUD_RATE,
            simulation_mode=config.SIMULATION_MODE
        )

        # 7. GUI Visualization with Keyboard Teleoperation
        self.visualizer = RoverVisualizer(
            on_reset_callback=self.reset_system,
            on_clear_path_callback=self.clear_path,
            on_drive_callback=self._handle_drive_command
        )

        # State tracking
        self.last_packet = TelemetryPacket()
        self.last_update_time = time.time()
        self.sensor_rays_state = {}

        # Keyboard Teleoperation & Path Tracing State
        self.active_drive_keys = set()
        self.manual_linear_v = 0.0
        self.manual_angular_w = 0.0
        self.drive_status_str = "STANDBY (Use Arrow Keys)"
        self.is_manual_mode = True

    def start(self):
        """Starts the reader thread and launches the Matplotlib interactive loop."""
        self.reader_thread.start()

        # Connect graceful shutdown signals
        signal.signal(signal.SIGINT, self._handle_exit)

        # Schedule animation update loop
        interval_ms = int(1000.0 / config.VIS_UPDATE_RATE)
        self.anim = FuncAnimation(
            self.visualizer.fig, self._animation_loop,
            interval=interval_ms, blit=False, cache_frame_data=False
        )

        plt.show()

    def _animation_loop(self, frame_idx):
        """Called periodically by Matplotlib to process queues and redraw."""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time

        # 1. Drain all pending packets from queue, keeping the latest state
        latest_packet = None
        while not self.packet_queue.empty():
            try:
                latest_packet = self.packet_queue.get_nowait()
            except queue.Empty:
                break

        if latest_packet is not None:
            self.last_packet = latest_packet
            self._process_telemetry_packet(latest_packet, dt)

        # 2. Extract visualization states
        rx, ry, heading_rad = self.odometry.get_pose()
        prob_grid = self.occupancy_map.get_probability_map()

        telemetry_hud = {
            "status": self.reader_thread.status_message,
            "is_sim": config.SIMULATION_MODE or self.last_packet.is_simulated,
            "packet_id": self.last_packet.packet_id,
            "errors": self.reader_thread.corrupted_packets_count,
            "speed": self.odometry.linear_velocity,
            "FL": self.last_packet.dist_fl,
            "FR": self.last_packet.dist_fr,
            "RL": self.last_packet.dist_rl,
            "RR": self.last_packet.dist_rr,
            "ax": self.last_packet.imu.ax,
            "ay": self.last_packet.imu.ay,
            "az": self.last_packet.imu.az,
            "gx": self.last_packet.imu.gx,
            "gy": self.last_packet.imu.gy,
            "gz": self.last_packet.imu.gz,
            "mx": self.last_packet.imu.mx,
            "my": self.last_packet.imu.my,
            "mz": self.last_packet.imu.mz,
            "obstacle_count": len(self.occupancy_map.obstacle_points)
        }

        # 3. Update Visualizer
        self.visualizer.update(
            rover_x=rx,
            rover_y=ry,
            heading_rad=heading_rad,
            path_x=self.odometry.path_x,
            path_y=self.odometry.path_y,
            obstacle_points=self.occupancy_map.obstacle_points,
            sensor_rays=self.sensor_rays_state,
            prob_grid=prob_grid,
            telemetry=telemetry_hud
        )

    def _process_telemetry_packet(self, packet: TelemetryPacket, dt: float):
        # Step 1: Calibrate & Fuse IMU
        clean_imu = self.imu_interface.process(packet.imu)
        yaw_rad, pitch_rad, roll_rad = self.heading_estimator.update(clean_imu)

        # Step 2: Update Odometry
        linear_v = 0.25 if (config.SIMULATION_MODE or packet.is_simulated) else 0.0
        rx, ry = self.odometry.update_from_velocity(linear_v, yaw_rad, dt)

        # Step 3: Process 4 Ultrasonic Sensors
        raw_dists = packet.get_distances()

        for s_key, raw_d in raw_dists.items():
            cfg = config.SENSOR_CONFIG[s_key]

            filt_d = self.median_filters[s_key].update(raw_d)
            filt_d = self.outlier_rejectors[s_key].filter(filt_d)

            valid_hit = (config.MIN_ULTRASONIC_DISTANCE <= filt_d <= config.MAX_ULTRASONIC_DISTANCE)
            effective_range = filt_d if valid_hit else cfg["max_dist"]

            ox, oy = CoordinateTransformer.sensor_origin_to_global_frame(
                cfg["offset_x"], cfg["offset_y"], rx, ry, yaw_rad
            )

            x_rel, y_rel = CoordinateTransformer.sensor_to_rover_frame(
                effective_range, cfg["angle"], cfg["offset_x"], cfg["offset_y"]
            )
            gx, gy = CoordinateTransformer.rover_to_global_frame(
                x_rel, y_rel, rx, ry, yaw_rad
            )

            self.sensor_rays_state[s_key] = {
                "origin": (ox, oy),
                "target": (gx, gy),
                "dist": filt_d,
                "valid": valid_hit
            }

            self.occupancy_map.update_ray(
                origin_gx=ox, origin_gy=oy,
                target_gx=gx, target_gy=gy,
                distance=filt_d,
                valid_hit=valid_hit
            )

    def reset_system(self):
        print("[GCS] Resetting Map, Trajectory, and Rover Origin...")
        self.occupancy_map.reset()
        self.odometry.set_origin(0.0, 0.0, 0.0)
        self.heading_estimator.reset_heading(0.0)
        for mf in self.median_filters.values():
            mf.reset()

    def clear_path(self):
        print("[GCS] Clearing Trajectory Trail...")
        self.odometry.clear_path()

    def _handle_drive_command(self, cmd: str, is_press: bool):
        """
        Handles arrow-key keyboard teleoperation events from the visualizer GUI.
        Keys: 'up', 'down', 'left', 'right', 'stop'
        Immediately advances/turns the rover and traces the path in that direction.
        """
        if is_press:
            self.active_drive_keys.add(cmd)
            rx, ry, heading_rad = self.odometry.get_pose()

            if cmd == 'up':
                self.drive_status_str = "DRIVING FORWARD (↑)"
                step = 0.08
                self.odometry.x += step * math.cos(heading_rad)
                self.odometry.y += step * math.sin(heading_rad)
                self.odometry.total_distance_traveled += step
                self.odometry.path_x.append(self.odometry.x)
                self.odometry.path_y.append(self.odometry.y)
                self.reader_thread.send_command('F')

            elif cmd == 'down':
                self.drive_status_str = "REVERSING (↓)"
                step = -0.08
                self.odometry.x += step * math.cos(heading_rad)
                self.odometry.y += step * math.sin(heading_rad)
                self.odometry.total_distance_traveled += abs(step)
                self.odometry.path_x.append(self.odometry.x)
                self.odometry.path_y.append(self.odometry.y)
                self.reader_thread.send_command('B')

            elif cmd == 'left':
                self.drive_status_str = "TURNING LEFT (←)"
                turn_step = 0.14
                new_heading = (heading_rad + turn_step) % (2.0 * math.pi)
                self.odometry.heading_rad = new_heading
                self.heading_estimator.current_heading_rad = new_heading
                self.reader_thread.send_command('L')

            elif cmd == 'right':
                self.drive_status_str = "TURNING RIGHT (→)"
                turn_step = -0.14
                new_heading = (heading_rad + turn_step) % (2.0 * math.pi)
                self.odometry.heading_rad = new_heading
                self.heading_estimator.current_heading_rad = new_heading
                self.reader_thread.send_command('R')

            elif cmd == 'stop':
                self.active_drive_keys.clear()
                self.manual_linear_v = 0.0
                self.manual_angular_w = 0.0
                self.drive_status_str = "STOPPED (Space)"
                self.reader_thread.send_command('S')

        else:
            self.active_drive_keys.discard(cmd)
            if not self.active_drive_keys:
                self.manual_linear_v = 0.0
                self.manual_angular_w = 0.0
                self.drive_status_str = "STANDBY (Arrow Keys)"
                self.reader_thread.send_command('S')

        v = 0.0
        w = 0.0
        if 'up' in self.active_drive_keys:
            v += 0.32
        if 'down' in self.active_drive_keys:
            v -= 0.22
        if 'left' in self.active_drive_keys:
            w += 0.85
        if 'right' in self.active_drive_keys:
            w -= 0.85

        self.manual_linear_v = v
        self.manual_angular_w = w
        self.reader_thread.set_manual_motion(v, w, is_manual=True)

    def _handle_exit(self, signum, frame):
        print("\\n[GCS] Shutting down REACT ground control station...")
        self.reader_thread.stop()
        sys.exit(0)


if __name__ == "__main__":
    app = ReactMappingSystem()
    app.start()
`
  },
  {
    name: "config.py",
    path: "react_mapping/config.py",
    category: "config",
    description: "Hardware port parameters, physical rover dimensions, sensor mount geometry, and map parameters.",
    language: "python",
    content: `"""
==============================================================================
REACT (Reactive Hazard Exploration/Response Rover)
Global Configuration File: config.py
==============================================================================
Centralizes all hardware settings, geometric coordinates, sensor parameters,
occupancy grid mapping specifications, and filter tuning.
"""

import math

# 1. SYSTEM OPERATION MODE
SIMULATION_MODE = True

# 2. SERIAL COMMUNICATION SETTINGS
SERIAL_PORT = "COM3"            # Windows: 'COM3', 'COM4' | Linux: '/dev/ttyUSB0'
BAUD_RATE = 9600                # 9600 baud rate matching Arduino sketch
SERIAL_TIMEOUT = 1.0           # Seconds before read timeout
RECONNECT_INTERVAL = 2.0       # Seconds between reconnection attempts
SENSOR_TIMEOUT_SECONDS = 2.5   # Trigger warning if no valid packet received

# 3. ROVER PHYSICAL GEOMETRY (Meters)
ROVER_LENGTH = 0.35            # Length along X-axis (forward/backward)
ROVER_WIDTH = 0.25             # Width along Y-axis (left/right)
WHEEL_BASE = 0.22              # Distance between left and right drive wheels
WHEEL_RADIUS = 0.045           # Drive wheel radius (45 mm)

# 4. ULTRASONIC SENSORS CONFIGURATION
# Convention: +X = Forward, +Y = Left
# Angles measured counter-clockwise from +X axis
SENSOR_CONFIG = {
    "FL": {
        "name": "Front Left",
        "angle": 45.0,                              # degrees (+45)
        "angle_rad": math.radians(45.0),
        "offset_x": ROVER_LENGTH / 2.0,             # +0.175 m (front edge)
        "offset_y": ROVER_WIDTH / 2.0,              # +0.125 m (left edge)
        "min_dist": 0.03,                           # 3 cm minimum HC-SR04 range
        "max_dist": 4.00,                           # 4 meters maximum range
        "beam_angle_deg": 15.0
    },
    "FR": {
        "name": "Front Right",
        "angle": -45.0,                             # degrees (-45)
        "angle_rad": math.radians(-45.0),
        "offset_x": ROVER_LENGTH / 2.0,             # +0.175 m (front edge)
        "offset_y": -ROVER_WIDTH / 2.0,             # -0.125 m (right edge)
        "min_dist": 0.03,
        "max_dist": 4.00,
        "beam_angle_deg": 15.0
    },
    "RL": {
        "name": "Rear Left",
        "angle": 135.0,                             # degrees (+135)
        "angle_rad": math.radians(135.0),
        "offset_x": -ROVER_LENGTH / 2.0,            # -0.175 m (rear edge)
        "offset_y": ROVER_WIDTH / 2.0,              # +0.125 m (left edge)
        "min_dist": 0.03,
        "max_dist": 4.00,
        "beam_angle_deg": 15.0
    },
    "RR": {
        "name": "Rear Right",
        "angle": -135.0,                            # degrees (-135)
        "angle_rad": math.radians(-135.0),
        "offset_x": -ROVER_LENGTH / 2.0,            # -0.175 m (rear edge)
        "offset_y": -ROVER_WIDTH / 2.0,             # -0.125 m (right edge)
        "min_dist": 0.03,
        "max_dist": 4.00,
        "beam_angle_deg": 15.0
    }
}

MAX_ULTRASONIC_DISTANCE = 4.0   # Meters
MIN_ULTRASONIC_DISTANCE = 0.03  # Meters
ULTRASONIC_MEDIAN_WINDOW = 5    # Rolling median filter window

# 5. 2D OCCUPANCY GRID MAP SETTINGS
MAP_RESOLUTION = 0.05           # 5 cm (0.05 meters) per grid cell
MAP_WIDTH_METERS = 20.0         # Total width in meters
MAP_HEIGHT_METERS = 20.0        # Total height in meters

CELL_UNKNOWN = 0.5
CELL_FREE = 0.1
CELL_OCCUPIED = 0.9

LOG_ODDS_FREE = -0.4
LOG_ODDS_OCCUPIED = 0.85
LOG_ODDS_MAX = 5.0
LOG_ODDS_MIN = -5.0

# 6. SENSOR FUSION & HEADING SETTINGS
COMPLEMENTARY_ALPHA = 0.96      # Gyro integration vs Magnetometer weighting
MAG_OFFSET_X = 0.0
MAG_OFFSET_Y = 0.0
MAG_OFFSET_Z = 0.0
MAG_SCALE_X = 1.0
MAG_SCALE_Y = 1.0
MAG_SCALE_Z = 1.0
MAGNETIC_DECLINATION_DEG = 0.5  # Adjust for mine location
GYRO_BIAS_X = 0.0
GYRO_BIAS_Y = 0.0
GYRO_BIAS_Z = 0.0

# 7. EXECUTION & UPDATE RATES
IMU_UPDATE_RATE = 50
MAPPING_UPDATE_RATE = 15
VIS_UPDATE_RATE = 20
QUEUE_MAX_SIZE = 100
`
  },
  {
    name: "serial_reader.py",
    path: "react_mapping/serial_reader.py",
    category: "python",
    description: "Threaded USB serial acquisition with packet parser, checksum, auto-reconnect, and coal-mine simulation engine.",
    language: "python",
    content: `"""
==============================================================================
REACT (Reactive Hazard Exploration/Response Rover)
Serial Communication & Telemetry Parser: serial_reader.py
==============================================================================
Provides:
1. Non-blocking threaded acquisition using queue.Queue.
2. Robust packet parsing for labelled key=value and key:value formats.
3. Checksum verification (*XOR_HEX).
4. Auto-reconnection on port drop or disconnect.
5. High-fidelity Simulation Engine (when SIMULATION_MODE = True).
"""

import time
import math
import random
import threading
import queue
import re
from typing import Optional, Dict, Any

from config import (
    SERIAL_PORT, BAUD_RATE, SERIAL_TIMEOUT, RECONNECT_INTERVAL,
    SENSOR_TIMEOUT_SECONDS, SIMULATION_MODE, SENSOR_CONFIG,
    MIN_ULTRASONIC_DISTANCE, MAX_ULTRASONIC_DISTANCE
)
from imu import IMUData

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class TelemetryPacket:
    def __init__(self):
        self.packet_id: int = 0
        self.timestamp: float = time.time()
        self.dist_front: float = 0.0
        self.dist_left: float = 0.0
        self.dist_right: float = 0.0
        self.dist_fl: float = 0.0
        self.dist_fr: float = 0.0
        self.dist_rl: float = 0.0
        self.dist_rr: float = 0.0
        self.gy271_heading_deg: float = 0.0
        self.imu: IMUData = IMUData()
        self.checksum_valid: bool = True
        self.is_simulated: bool = False
        self.raw_line: str = ""

    @property
    def heading_deg(self) -> float:
        return self.gy271_heading_deg

    @heading_deg.setter
    def heading_deg(self, val: float):
        self.gy271_heading_deg = val

    def get_distances(self) -> Dict[str, float]:
        return {
            "FRONT": self.dist_front,
            "LEFT": self.dist_left,
            "RIGHT": self.dist_right
        }


class SerialReaderThread(threading.Thread):
    def __init__(self,
                 packet_queue: queue.Queue,
                 port: str = SERIAL_PORT,
                 baudrate: int = BAUD_RATE,
                 simulation_mode: bool = SIMULATION_MODE):
        super().__init__(daemon=True)
        self.packet_queue = packet_queue
        self.port = port
        self.baudrate = baudrate
        self.simulation_mode = simulation_mode

        self.running = False
        self.serial_conn: Optional[Any] = None

        self.connected = False
        self.last_packet_time = 0.0
        self.total_packets_received = 0
        self.corrupted_packets_count = 0
        self.status_message = "Initializing..."

        # Simulation kinematics
        self._sim_x = 0.0
        self._sim_y = 0.0
        self._sim_theta = 0.0
        self._sim_speed = 0.25
        self._sim_turn_rate = 0.0

    def stop(self):
        self.running = False
        if self.serial_conn and hasattr(self.serial_conn, 'close'):
            try:
                self.serial_conn.close()
            except Exception:
                pass

    def run(self):
        self.running = True
        if self.simulation_mode:
            self.status_message = "Running in SIMULATION MODE"
            self._run_simulation()
        else:
            self.status_message = f"Connecting to {self.port} at {self.baudrate} baud..."
            self._run_hardware_serial()

    def _run_hardware_serial(self):
        if not SERIAL_AVAILABLE:
            self.status_message = "ERROR: pySerial not installed. Falling back to simulation."
            self._run_simulation()
            return

        while self.running:
            try:
                self.status_message = f"Opening {self.port}..."
                self.serial_conn = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=SERIAL_TIMEOUT
                )
                self.connected = True
                self.status_message = f"Connected to Arduino Uno on {self.port}"
                time.sleep(1.8)

                while self.running and self.serial_conn.is_open:
                    line = self.serial_conn.readline().decode('utf-8', errors='replace').strip()
                    if not line:
                        if (time.time() - self.last_packet_time) > SENSOR_TIMEOUT_SECONDS and self.total_packets_received > 0:
                            self.status_message = "WARNING: Serial data timeout"
                        continue

                    packet = self.parse_packet(line)
                    if packet is not None:
                        self.last_packet_time = time.time()
                        self.total_packets_received += 1
                        self.status_message = "Streaming hardware data (OK)"
                        try:
                            self.packet_queue.put_nowait(packet)
                        except queue.Full:
                            pass
                    else:
                        self.corrupted_packets_count += 1

            except (serial.SerialException, OSError) as err:
                self.connected = False
                self.status_message = f"Serial Error: {err}. Retrying in {RECONNECT_INTERVAL}s..."
                time.sleep(RECONNECT_INTERVAL)
            except Exception as e:
                self.connected = False
                self.status_message = f"Unexpected Error: {e}"
                time.sleep(RECONNECT_INTERVAL)

    def parse_packet(self, line: str) -> Optional[TelemetryPacket]:
        if not line:
            return None

        clean_line = line.strip()
        checksum_valid = True

        # Support direct human-readable "Angle: XX.XX°" or "Angle: XX.XX" format from MPU test sketches
        if "Angle:" in clean_line:
            match = re.search(r'Angle:\s*([-\d.]+)', clean_line)
            if match:
                try:
                    hdg = float(match.group(1)) % 360.0
                    if hdg < 0.0:
                        hdg += 360.0
                    packet = TelemetryPacket()
                    packet.raw_line = line
                    packet.timestamp = time.time()
                    packet.packet_id = self.total_packets_received + 1
                    packet.gy271_heading_deg = hdg

                    # Generate pseudo ultrasonic distances for the virtual environment
                    rad = math.radians(hdg)
                    cos_a, sin_a = math.cos(rad), math.sin(rad)
                    dx = 1.80 / abs(cos_a) if abs(cos_a) > 0.02 else 3.5
                    dy = 0.95 / abs(sin_a) if abs(sin_a) > 0.02 else 3.5
                    d_front = max(0.1, min(min(dx, dy, 3.5), 4.0))
                    packet.dist_front = d_front
                    packet.dist_fl = d_front

                    rad_l = math.radians(hdg + 90.0)
                    cos_l, sin_l = math.cos(rad_l), math.sin(rad_l)
                    dx_l = 1.80 / abs(cos_l) if abs(cos_l) > 0.02 else 3.5
                    dy_l = 0.95 / abs(sin_l) if abs(sin_l) > 0.02 else 3.5
                    d_left = max(0.1, min(min(dx_l, dy_l, 3.5), 4.0))
                    packet.dist_left = d_left
                    packet.dist_fr = d_left

                    rad_r = math.radians(hdg - 90.0)
                    cos_r, sin_r = math.cos(rad_r), math.sin(rad_r)
                    dx_r = 1.80 / abs(cos_r) if abs(cos_r) > 0.02 else 3.5
                    dy_r = 0.95 / abs(sin_r) if abs(sin_r) > 0.02 else 3.5
                    d_right = max(0.1, min(min(dx_r, dy_r, 3.5), 4.0))
                    packet.dist_right = d_right
                    packet.dist_rl = d_right

                    packet.imu.az = 1.0
                    packet.imu.timestamp = packet.timestamp
                    return packet
                except Exception:
                    pass

        if '*' in clean_line:
            data_part, chk_part = clean_line.rsplit('*', 1)
            try:
                expected_cs = int(chk_part, 16)
                computed_cs = 0
                for ch in data_part:
                    computed_cs ^= ord(ch)
                if computed_cs != expected_cs:
                    return None
            except ValueError:
                pass
            clean_line = data_part

        packet = TelemetryPacket()
        packet.raw_line = line
        packet.timestamp = time.time()
        packet.checksum_valid = checksum_valid

        tokens = clean_line.split(',')
        found_fields = 0

        for token in tokens:
            token = token.strip()
            if not token:
                continue

            if token.startswith('#'):
                try:
                    packet.packet_id = int(token[1:])
                except ValueError:
                    pass
                continue

            sep = '=' if '=' in token else (':' if ':' in token else None)
            if not sep:
                continue

            key, val_str = token.split(sep, 1)
            key = key.strip().upper()
            try:
                val = float(val_str.strip())
            except ValueError:
                continue

            if key in ('FL', 'U1', 'D1', 'DF', 'FRONT', 'DIST_FL'):
                packet.dist_fl = val / 100.0 if val > 10.0 else val
                packet.dist_front = packet.dist_fl
                found_fields += 1
            elif key in ('FR', 'U2', 'D2', 'DL', 'LEFT', 'DIST_FR'):
                packet.dist_fr = val / 100.0 if val > 10.0 else val
                packet.dist_left = packet.dist_fr
                found_fields += 1
            elif key in ('RL', 'U3', 'D3', 'DR', 'RIGHT', 'DIST_RL'):
                packet.dist_rl = val / 100.0 if val > 10.0 else val
                packet.dist_right = packet.dist_rl
                found_fields += 1
            elif key in ('RR', 'U4', 'D4', 'DB', 'REAR', 'BACK', 'DIST_RR'):
                packet.dist_rr = val / 100.0 if val > 10.0 else val
                found_fields += 1
            elif key in ('ANGLE', 'ANGLEZ', 'HDG', 'HEADING', 'YAW', 'YAW_DEG'):
                packet.gy271_heading_deg = val % 360.0
                found_fields += 1
            elif key == 'AX':
                packet.imu.ax = val
                found_fields += 1
            elif key == 'AY':
                packet.imu.ay = val
                found_fields += 1
            elif key == 'AZ':
                packet.imu.az = val
                found_fields += 1
            elif key == 'GX':
                packet.imu.gx = val
                found_fields += 1
            elif key == 'GY':
                packet.imu.gy = val
                found_fields += 1
            elif key == 'GZ':
                packet.imu.gz = val
                found_fields += 1
            elif key == 'MX':
                packet.imu.mx = val
                found_fields += 1
            elif key == 'MY':
                packet.imu.my = val
                found_fields += 1
            elif key == 'MZ':
                packet.imu.mz = val
                found_fields += 1

        # Auto-generate pseudo ultrasonic distances if no physical ultrasonics are connected yet
        if packet.dist_front == 0.0 and packet.dist_left == 0.0 and packet.dist_right == 0.0 and found_fields >= 1:
            rad = math.radians(packet.gy271_heading_deg)
            cos_a, sin_a = math.cos(rad), math.sin(rad)
            dx = 1.80 / abs(cos_a) if abs(cos_a) > 0.02 else 3.5
            dy = 0.95 / abs(sin_a) if abs(sin_a) > 0.02 else 3.5
            packet.dist_front = max(0.1, min(min(dx, dy, 3.5), 4.0))
            packet.dist_fl = packet.dist_front

            rad_l = math.radians(packet.gy271_heading_deg + 90.0)
            cos_l, sin_l = math.cos(rad_l), math.sin(rad_l)
            dx_l = 1.80 / abs(cos_l) if abs(cos_l) > 0.02 else 3.5
            dy_l = 0.95 / abs(sin_l) if abs(sin_l) > 0.02 else 3.5
            packet.dist_left = max(0.1, min(min(dx_l, dy_l, 3.5), 4.0))
            packet.dist_fr = packet.dist_left

            rad_r = math.radians(packet.gy271_heading_deg - 90.0)
            cos_r, sin_r = math.cos(rad_r), math.sin(rad_r)
            dx_r = 1.80 / abs(cos_r) if abs(cos_r) > 0.02 else 3.5
            dy_r = 0.95 / abs(sin_r) if abs(sin_r) > 0.02 else 3.5
            packet.dist_right = max(0.1, min(min(dx_r, dy_r, 3.5), 4.0))
            packet.dist_rl = packet.dist_right

        if found_fields >= 1:
            packet.imu.timestamp = packet.timestamp
            return packet

        return None

    def _run_simulation(self):
        self.connected = True
        seq = 0
        dt = 0.05

        mine_walls = [
            (-6.0, -4.0, 6.0, -4.0),
            (6.0, -4.0, 6.0, 4.0),
            (6.0, 4.0, -6.0, 4.0),
            (-6.0, 4.0, -6.0, -4.0),
            (-2.0, -1.5, -2.0, -0.5),
            (2.0, 0.5, 2.0, 1.5)
        ]

        while self.running:
            time.sleep(dt)
            seq += 1

            self._sim_turn_rate = 0.35 * math.sin(seq * 0.04)
            self._sim_theta += self._sim_turn_rate * dt
            self._sim_x += self._sim_speed * math.cos(self._sim_theta) * dt
            self._sim_y += self._sim_speed * math.sin(self._sim_theta) * dt

            if abs(self._sim_x) > 4.5 or abs(self._sim_y) > 2.8:
                self._sim_theta += math.pi * 0.5

            packet = TelemetryPacket()
            packet.packet_id = seq
            packet.timestamp = time.time()
            packet.is_simulated = True

            packet.imu.gz = math.degrees(self._sim_turn_rate) + random.gauss(0.0, 0.4)
            packet.imu.gx = random.gauss(0.0, 0.2)
            packet.imu.gy = random.gauss(0.0, 0.2)
            packet.imu.az = 1.0 + random.gauss(0.0, 0.02)
            packet.imu.ax = random.gauss(0.0, 0.02)
            packet.imu.ay = (self._sim_speed * self._sim_turn_rate / 9.8) + random.gauss(0.0, 0.02)

            earth_field = 45.0
            packet.imu.mx = earth_field * math.cos(self._sim_theta) + random.gauss(0.0, 0.6)
            packet.imu.my = earth_field * math.sin(self._sim_theta) + random.gauss(0.0, 0.6)
            packet.imu.mz = 35.0 + random.gauss(0.0, 0.5)
            packet.imu.timestamp = packet.timestamp

            for sensor_key, cfg in SENSOR_CONFIG.items():
                sensor_angle = self._sim_theta + cfg["angle_rad"]
                ox = self._sim_x + (cfg["offset_x"] * math.cos(self._sim_theta) - cfg["offset_y"] * math.sin(self._sim_theta))
                oy = self._sim_y + (cfg["offset_x"] * math.sin(self._sim_theta) + cfg["offset_y"] * math.cos(self._sim_theta))

                min_hit = cfg["max_dist"]
                for x1, y1, x2, y2 in mine_walls:
                    hit = self._ray_segment_intersection(ox, oy, sensor_angle, x1, y1, x2, y2, cfg["max_dist"])
                    if hit is not None and hit < min_hit:
                        min_hit = hit

                measured_dist = max(MIN_ULTRASONIC_DISTANCE, min(MAX_ULTRASONIC_DISTANCE, min_hit + random.gauss(0.0, 0.015)))

                if sensor_key == "FL":
                    packet.dist_fl = measured_dist
                elif sensor_key == "FR":
                    packet.dist_fr = measured_dist
                elif sensor_key == "RL":
                    packet.dist_rl = measured_dist
                elif sensor_key == "RR":
                    packet.dist_rr = measured_dist

            self.last_packet_time = packet.timestamp
            self.total_packets_received += 1
            try:
                self.packet_queue.put_nowait(packet)
            except queue.Full:
                pass

    @staticmethod
    def _ray_segment_intersection(ox: float, oy: float, theta: float,
                                  x1: float, y1: float, x2: float, y2: float,
                                  max_range: float) -> Optional[float]:
        dx = math.cos(theta)
        dy = math.sin(theta)
        v1x = ox - x1
        v1y = oy - y1
        v2x = x2 - x1
        v2y = y2 - y1
        v3x = -dy
        v3y = dx
        dot = v2x * v3x + v2y * v3y
        if abs(dot) < 1e-6:
            return None
        t1 = (v2x * v1y - v2y * v1x) / dot
        t2 = (v1x * v3x + v1y * v3y) / dot
        if 0.0 <= t1 <= max_range and 0.0 <= t2 <= 1.0:
            return t1
        return None
`
  },
  {
    name: "coordinate_transform.py",
    path: "react_mapping/coordinate_transform.py",
    category: "python",
    description: "Special Euclidean SE(2) transformation engine converting sensor-relative coordinates to global world map.",
    language: "python",
    content: `"""
==============================================================================
REACT (Reactive Hazard Exploration/Response Rover)
Coordinate Transformation Module: coordinate_transform.py
==============================================================================
Implements 2D rigid-body transformations (Special Euclidean Group SE(2)):
1. Rover Local Frame: +X = Forward, +Y = Left
2. Sensor-to-Rover: Accounts for physical offsets (offset_x, offset_y) and angle (alpha).
3. Rover-to-Global: Rotates and translates into world frame:
   [X_global]   [cos(theta)  -sin(theta)] [x_rover]   [X_rover]
   [Y_global] = [sin(theta)   cos(theta)] [y_rover] + [Y_rover]
"""

import math
from typing import Tuple, List


class CoordinateTransformer:
    @staticmethod
    def sensor_to_rover_frame(distance: float,
                              sensor_angle_deg: float,
                              offset_x: float,
                              offset_y: float) -> Tuple[float, float]:
        """
        Converts an ultrasonic distance reading into rover-centric coordinates (x_rover, y_rover).
        """
        angle_rad = math.radians(sensor_angle_deg)
        dx = distance * math.cos(angle_rad)
        dy = distance * math.sin(angle_rad)
        x_rover = offset_x + dx
        y_rover = offset_y + dy
        return x_rover, y_rover

    @staticmethod
    def sensor_origin_to_global_frame(offset_x: float,
                                      offset_y: float,
                                      rover_x: float,
                                      rover_y: float,
                                      rover_heading_rad: float) -> Tuple[float, float]:
        """
        Returns the global coordinates of the physical sensor transducer on the rover.
        """
        cos_th = math.cos(rover_heading_rad)
        sin_th = math.sin(rover_heading_rad)
        origin_gx = rover_x + (offset_x * cos_th - offset_y * sin_th)
        origin_gy = rover_y + (offset_x * sin_th + offset_y * cos_th)
        return origin_gx, origin_gy

    @staticmethod
    def rover_to_global_frame(x_rover: float,
                              y_rover: float,
                              rover_x: float,
                              rover_y: float,
                              rover_heading_rad: float) -> Tuple[float, float]:
        """
        Transforms a point from the rover coordinate frame to the global map frame.
        """
        cos_th = math.cos(rover_heading_rad)
        sin_th = math.sin(rover_heading_rad)
        x_global = rover_x + (x_rover * cos_th - y_rover * sin_th)
        y_global = rover_y + (x_rover * sin_th + y_rover * cos_th)
        return x_global, y_global

    @staticmethod
    def transform_polygon(polygon_vertices: List[Tuple[float, float]],
                          rover_x: float,
                          rover_y: float,
                          rover_heading_rad: float) -> List[Tuple[float, float]]:
        """
        Transforms a list of local 2D vertices into global world coordinates.
        """
        cos_th = math.cos(rover_heading_rad)
        sin_th = math.sin(rover_heading_rad)
        transformed = []
        for vx, vy in polygon_vertices:
            gx = rover_x + (vx * cos_th - vy * sin_th)
            gy = rover_y + (vx * sin_th + vy * cos_th)
            transformed.append((gx, gy))
        return transformed

    @staticmethod
    def global_to_grid_cell(x_global: float,
                            y_global: float,
                            map_origin_x: float,
                            map_origin_y: float,
                            resolution: float) -> Tuple[int, int]:
        col = int(math.floor((x_global - map_origin_x) / resolution))
        row = int(math.floor((y_global - map_origin_y) / resolution))
        return col, row

    @staticmethod
    def grid_cell_to_global(col: int,
                            row: int,
                            map_origin_x: float,
                            map_origin_y: float,
                            resolution: float) -> Tuple[float, float]:
        x_global = map_origin_x + (col + 0.5) * resolution
        y_global = map_origin_y + (row + 0.5) * resolution
        return x_global, y_global
`
  },
  {
    name: "sensor_fusion.py",
    path: "react_mapping/sensor_fusion.py",
    category: "python",
    description: "Tilt-compensated magnetometer compass fusion with Gyro Z rate via Complementary filter.",
    language: "python",
    content: `"""
==============================================================================
REACT (Reactive Hazard Exploration/Response Rover)
Sensor Fusion & Heading Estimation: sensor_fusion.py
==============================================================================
Fuses Gyro Z rate + Accelerometer Tilt + Magnetometer Horizontal Heading.
"""

import math
from typing import Tuple, Optional
from imu import IMUData, IMUInterface


def normalize_angle_rad(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def normalize_angle_deg(angle: float) -> float:
    angle = angle % 360.0
    if angle < 0:
        angle += 360.0
    return angle


class HeadingEstimator:
    def __init__(self,
                 alpha: float = 0.96,
                 magnetic_declination_deg: float = 0.0):
        self.alpha = alpha
        self.declination_rad = math.radians(magnetic_declination_deg)
        self.yaw_rad: float = 0.0
        self.pitch_rad: float = 0.0
        self.roll_rad: float = 0.0
        self.angular_velocity_z: float = 0.0
        self.last_timestamp: Optional[float] = None
        self.initialized = False

    def reset_heading(self, initial_heading_deg: float = 0.0):
        self.yaw_rad = math.radians(initial_heading_deg)
        self.last_timestamp = None

    def calculate_tilt_compensated_mag_heading(self,
                                               mx: float, my: float, mz: float,
                                               pitch: float, roll: float) -> float:
        sin_p = math.sin(pitch)
        cos_p = math.cos(pitch)
        sin_r = math.sin(roll)
        cos_r = math.cos(roll)

        xh = mx * cos_p + mz * sin_p
        yh = mx * sin_r * sin_p + my * cos_r - mz * sin_r * cos_p

        mag_heading = math.atan2(yh, xh) + self.declination_rad
        return normalize_angle_rad(mag_heading)

    def update(self, imu: IMUData) -> Tuple[float, float, float]:
        self.pitch_rad, self.roll_rad = IMUInterface.calculate_pitch_roll(imu.ax, imu.ay, imu.az)
        mag_yaw = self.calculate_tilt_compensated_mag_heading(
            imu.mx, imu.my, imu.mz, self.pitch_rad, self.roll_rad
        )

        current_time = imu.timestamp
        self.angular_velocity_z = math.radians(imu.gz)

        if not self.initialized or self.last_timestamp is None:
            self.yaw_rad = mag_yaw
            self.last_timestamp = current_time
            self.initialized = True
            return self.yaw_rad, self.pitch_rad, self.roll_rad

        dt = current_time - self.last_timestamp
        self.last_timestamp = current_time
        if dt <= 0.0 or dt > 0.5:
            dt = 0.02

        gyro_yaw = normalize_angle_rad(self.yaw_rad + (self.angular_velocity_z * dt))
        diff = normalize_angle_rad(mag_yaw - gyro_yaw)
        self.yaw_rad = normalize_angle_rad(gyro_yaw + (1.0 - self.alpha) * diff)

        return self.yaw_rad, self.pitch_rad, self.roll_rad

    @property
    def heading_deg(self) -> float:
        return normalize_angle_deg(math.degrees(self.yaw_rad))

    @property
    def heading_signed_deg(self) -> float:
        return math.degrees(self.yaw_rad)
`
  },
  {
    name: "odometry.py",
    path: "react_mapping/odometry.py",
    category: "python",
    description: "Dead-reckoning position tracking with differential drive odometry and wheel encoder hooks.",
    language: "python",
    content: `"""
==============================================================================
REACT (Reactive Hazard Exploration/Response Rover)
Dead-Reckoning & Odometry Engine: odometry.py
==============================================================================
Explains why accelerometer double-integration fails and implements differential
drive odometry and fused heading position updates.
"""

import math
from typing import Tuple, List, Optional
from config import WHEEL_BASE, WHEEL_RADIUS


class OdometryEstimator:
    def __init__(self,
                 wheel_base: float = WHEEL_BASE,
                 wheel_radius: float = WHEEL_RADIUS):
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius

        self.x: float = 0.0
        self.y: float = 0.0
        self.heading_rad: float = 0.0

        self.linear_velocity: float = 0.0
        self.angular_velocity: float = 0.0
        self.total_distance_traveled: float = 0.0

        self.path_x: List[float] = [0.0]
        self.path_y: List[float] = [0.0]
        self.min_path_dist_threshold = 0.03

        self.last_left_ticks: Optional[int] = None
        self.last_right_ticks: Optional[int] = None
        self.ticks_per_revolution = 360

    def set_origin(self, x: float = 0.0, y: float = 0.0, heading_rad: float = 0.0):
        self.x = x
        self.y = y
        self.heading_rad = heading_rad
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.total_distance_traveled = 0.0
        self.path_x = [x]
        self.path_y = [y]

    def clear_path(self):
        self.path_x = [self.x]
        self.path_y = [self.y]

    def update_from_velocity(self,
                             linear_v: float,
                             heading_rad: float,
                             dt: float) -> Tuple[float, float]:
        self.linear_velocity = linear_v
        self.heading_rad = heading_rad

        if dt <= 0.0 or dt > 1.0:
            dt = 0.05

        step_dist = linear_v * dt
        self.x += step_dist * math.cos(heading_rad)
        self.y += step_dist * math.sin(heading_rad)
        self.total_distance_traveled += abs(step_dist)

        dx = self.x - self.path_x[-1]
        dy = self.y - self.path_y[-1]
        if math.hypot(dx, dy) >= self.min_path_dist_threshold:
            self.path_x.append(self.x)
            self.path_y.append(self.y)

        return self.x, self.y

    def update_from_encoders(self,
                             left_ticks: int,
                             right_ticks: int,
                             fused_heading_rad: float,
                             dt: float) -> Tuple[float, float]:
        if self.last_left_ticks is None or self.last_right_ticks is None:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.heading_rad = fused_heading_rad
            return self.x, self.y

        d_left_ticks = left_ticks - self.last_left_ticks
        d_right_ticks = right_ticks - self.last_right_ticks
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks

        meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_revolution
        d_left = d_left_ticks * meters_per_tick
        d_right = d_right_ticks * meters_per_tick

        d_center = (d_right + d_left) / 2.0
        self.linear_velocity = d_center / dt if dt > 0 else 0.0

        avg_heading = (self.heading_rad + fused_heading_rad) / 2.0
        self.heading_rad = fused_heading_rad

        self.x += d_center * math.cos(avg_heading)
        self.y += d_center * math.sin(avg_heading)
        self.total_distance_traveled += abs(d_center)

        dx = self.x - self.path_x[-1]
        dy = self.y - self.path_y[-1]
        if math.hypot(dx, dy) >= self.min_path_dist_threshold:
            self.path_x.append(self.x)
            self.path_y.append(self.y)

        return self.x, self.y

    def get_pose(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.heading_rad
`
  },
  {
    name: "mapping.py",
    path: "react_mapping/mapping.py",
    category: "python",
    description: "2D Bayesian Occupancy Grid map with Bresenham free-space ray tracing and obstacle log-odds updates.",
    language: "python",
    content: `"""
==============================================================================
REACT (Reactive Hazard Exploration/Response Rover)
2D Occupancy Grid Mapping Engine: mapping.py
==============================================================================
Bresenham ray-tracer carves FREE space and marks OCCUPIED obstacles.
"""

import math
from typing import List, Tuple, Set
import numpy as np

from config import (
    MAP_RESOLUTION, MAP_WIDTH_METERS, MAP_HEIGHT_METERS,
    LOG_ODDS_FREE, LOG_ODDS_OCCUPIED, LOG_ODDS_MAX, LOG_ODDS_MIN,
    MIN_ULTRASONIC_DISTANCE, MAX_ULTRASONIC_DISTANCE
)


class OccupancyGridMap:
    def __init__(self,
                 resolution: float = MAP_RESOLUTION,
                 width_m: float = MAP_WIDTH_METERS,
                 height_m: float = MAP_HEIGHT_METERS):
        self.resolution = resolution
        self.width_m = width_m
        self.height_m = height_m

        self.cols = int(round(width_m / resolution))
        self.rows = int(round(height_m / resolution))

        self.origin_x = -width_m / 2.0
        self.origin_y = -height_m / 2.0

        self.grid = np.zeros((self.rows, self.cols), dtype=np.float32)
        self.obstacle_points: List[Tuple[float, float]] = []
        self.occupied_cells: Set[Tuple[int, int]] = set()

    def reset(self):
        self.grid.fill(0.0)
        self.obstacle_points.clear()
        self.occupied_cells.clear()

    def world_to_grid(self, gx: float, gy: float) -> Tuple[int, int]:
        col = int(math.floor((gx - self.origin_x) / self.resolution))
        row = int(math.floor((gy - self.origin_y) / self.resolution))
        return col, row

    def grid_to_world(self, col: int, row: int) -> Tuple[float, float]:
        gx = self.origin_x + (col + 0.5) * self.resolution
        gy = self.origin_y + (row + 0.5) * self.resolution
        return gx, gy

    def is_inside(self, col: int, row: int) -> bool:
        return 0 <= col < self.cols and 0 <= row < self.rows

    @staticmethod
    def bresenham_line(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        curr_x = x0
        curr_y = y0

        while True:
            cells.append((curr_x, curr_y))
            if curr_x == x1 and curr_y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                curr_x += sx
            if e2 < dx:
                err += dx
                curr_y += sy

        return cells

    def update_ray(self,
                   origin_gx: float, origin_gy: float,
                   target_gx: float, target_gy: float,
                   distance: float,
                   valid_hit: bool):
        c0, r0 = self.world_to_grid(origin_gx, origin_gy)
        c1, r1 = self.world_to_grid(target_gx, target_gy)

        if not self.is_inside(c0, r0):
            return

        ray_cells = self.bresenham_line(c0, r0, c1, r1)

        if distance >= MIN_ULTRASONIC_DISTANCE:
            for c, r in ray_cells[:-1]:
                if self.is_inside(c, r):
                    self.grid[r, c] = max(LOG_ODDS_MIN, self.grid[r, c] + LOG_ODDS_FREE)

        if valid_hit and MIN_ULTRASONIC_DISTANCE <= distance <= MAX_ULTRASONIC_DISTANCE:
            if self.is_inside(c1, r1):
                self.grid[r1, c1] = min(LOG_ODDS_MAX, self.grid[r1, c1] + LOG_ODDS_OCCUPIED)
                cell_key = (c1, r1)
                if cell_key not in self.occupied_cells:
                    self.occupied_cells.add(cell_key)
                    self.obstacle_points.append((target_gx, target_gy))

    def get_probability_map(self) -> np.ndarray:
        clamped_grid = np.clip(self.grid, -10.0, 10.0)
        return 1.0 - (1.0 / (1.0 + np.exp(clamped_grid)))
`
  },
  {
    name: "imu.py",
    path: "react_mapping/imu.py",
    category: "python",
    description: "Driver for MPU-6050/6500 6-DOF IMU and external HMC5883L/QMC5883L compass with hard-iron calibration.",
    language: "python",
    content: `"""
==============================================================================
REACT (Reactive Hazard Exploration/Response Rover)
IMU & Magnetometer Driver/Parser: imu.py
==============================================================================
Handles MPU-6050 / MPU-6500 IMU and external compass calibration.
"""

import math
from typing import Dict, Tuple


class IMUData:
    def __init__(self,
                 ax: float = 0.0, ay: float = 0.0, az: float = 1.0,
                 gx: float = 0.0, gy: float = 0.0, gz: float = 0.0,
                 mx: float = 0.0, my: float = 0.0, mz: float = 0.0,
                 timestamp: float = 0.0):
        self.ax = ax
        self.ay = ay
        self.az = az
        self.gx = gx
        self.gy = gy
        self.gz = gz
        self.mx = mx
        self.my = my
        self.mz = mz
        self.timestamp = timestamp

    def to_dict(self) -> Dict[str, float]:
        return {
            "ax": self.ax, "ay": self.ay, "az": self.az,
            "gx": self.gx, "gy": self.gy, "gz": self.gz,
            "mx": self.mx, "my": self.my, "mz": self.mz,
            "timestamp": self.timestamp
        }


class IMUInterface:
    def __init__(self,
                 gyro_bias_x: float = 0.0,
                 gyro_bias_y: float = 0.0,
                 gyro_bias_z: float = 0.0,
                 mag_offset_x: float = 0.0,
                 mag_offset_y: float = 0.0,
                 mag_offset_z: float = 0.0,
                 mag_scale_x: float = 1.0,
                 mag_scale_y: float = 1.0,
                 mag_scale_z: float = 1.0):
        self.gyro_bias_x = gyro_bias_x
        self.gyro_bias_y = gyro_bias_y
        self.gyro_bias_z = gyro_bias_z
        self.mag_offset_x = mag_offset_x
        self.mag_offset_y = mag_offset_y
        self.mag_offset_z = mag_offset_z
        self.mag_scale_x = mag_scale_x
        self.mag_scale_y = mag_scale_y
        self.mag_scale_z = mag_scale_z

    def calibrate_gyro_bias(self, samples: list):
        if not samples:
            return
        n = len(samples)
        self.gyro_bias_x = sum(s.gx for s in samples) / n
        self.gyro_bias_y = sum(s.gy for s in samples) / n
        self.gyro_bias_z = sum(s.gz for s in samples) / n

    def calibrate_magnetometer_hard_iron(self, raw_samples: list):
        if len(raw_samples) < 20:
            return
        xs = [s.mx for s in raw_samples]
        ys = [s.my for s in raw_samples]
        zs = [s.mz for s in raw_samples]

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        min_z, max_z = min(zs), max(zs)

        self.mag_offset_x = (max_x + min_x) / 2.0
        self.mag_offset_y = (max_y + min_y) / 2.0
        self.mag_offset_z = (max_z + min_z) / 2.0

        delta_x = (max_x - min_x) / 2.0
        delta_y = (max_y - min_y) / 2.0
        delta_z = (max_z - min_z) / 2.0
        avg_delta = (delta_x + delta_y + delta_z) / 3.0

        if delta_x > 0.001: self.mag_scale_x = avg_delta / delta_x
        if delta_y > 0.001: self.mag_scale_y = avg_delta / delta_y
        if delta_z > 0.001: self.mag_scale_z = avg_delta / delta_z

    def process(self, raw: IMUData) -> IMUData:
        clean_gx = raw.gx - self.gyro_bias_x
        clean_gy = raw.gy - self.gyro_bias_y
        clean_gz = raw.gz - self.gyro_bias_z

        clean_mx = (raw.mx - self.mag_offset_x) * self.mag_scale_x
        clean_my = (raw.my - self.mag_offset_y) * self.mag_scale_y
        clean_mz = (raw.mz - self.mag_offset_z) * self.mag_scale_z

        return IMUData(
            ax=raw.ax, ay=raw.ay, az=raw.az,
            gx=clean_gx, gy=clean_gy, gz=clean_gz,
            mx=clean_mx, my=clean_my, mz=clean_mz,
            timestamp=raw.timestamp
        )

    @staticmethod
    def calculate_pitch_roll(ax: float, ay: float, az: float) -> Tuple[float, float]:
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        return pitch, roll
`
  },
  {
    name: "filters.py",
    path: "react_mapping/filters.py",
    category: "python",
    description: "Digital filters: Rolling median, Exponential Moving Average, and dynamic outlier rejection.",
    language: "python",
    content: `"""
==============================================================================
REACT (Reactive Hazard Exploration/Response Rover)
Sensor Filtering Module: filters.py
==============================================================================
Rolling Median, Exponential Moving Average, and Outlier Gating.
"""

from collections import deque
from typing import Optional


class MedianFilter:
    def __init__(self, window_size: int = 5):
        if window_size % 2 == 0:
            window_size += 1
        self.window_size = window_size
        self.buffer = deque(maxlen=window_size)

    def update(self, value: float) -> float:
        self.buffer.append(value)
        sorted_vals = sorted(self.buffer)
        mid = len(sorted_vals) // 2
        return sorted_vals[mid]

    def reset(self):
        self.buffer.clear()


class MovingAverageFilter:
    def __init__(self, alpha: float = 0.25):
        self.alpha = max(0.01, min(1.0, alpha))
        self.current_value: Optional[float] = None

    def update(self, value: float) -> float:
        if self.current_value is None:
            self.current_value = value
        else:
            self.current_value = (self.alpha * value) + ((1.0 - self.alpha) * self.current_value)
        return self.current_value

    def reset(self):
        self.current_value = None


class OutlierRejector:
    def __init__(self, max_delta: float = 1.5):
        self.max_delta = max_delta
        self.last_valid: Optional[float] = None
        self.consecutive_rejects = 0
        self.max_consecutive = 4

    def filter(self, value: float) -> Optional[float]:
        if self.last_valid is None:
            self.last_valid = value
            return value

        delta = abs(value - self.last_valid)
        if delta <= self.max_delta:
            self.last_valid = value
            self.consecutive_rejects = 0
            return value
        else:
            self.consecutive_rejects += 1
            if self.consecutive_rejects >= self.max_consecutive:
                self.last_valid = value
                self.consecutive_rejects = 0
                return value
            return self.last_valid
`
  },
  {
    name: "visualization.py",
    path: "react_mapping/visualization.py",
    category: "python",
    description: "Matplotlib 2D top-down map, obstacle points, sensor rays, heading arrow, and telemetry HUD.",
    language: "python",
    content: `"""
==============================================================================
REACT (Reactive Hazard Exploration/Response Rover)
Real-Time 2D Robotics Visualization: visualization.py
==============================================================================
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle
from matplotlib.widgets import Button
from typing import Dict, Any, List

from config import (
    ROVER_LENGTH, ROVER_WIDTH, SENSOR_CONFIG,
    MAP_WIDTH_METERS, MAP_HEIGHT_METERS, MAP_RESOLUTION
)
from coordinate_transform import CoordinateTransformer


class RoverVisualizer:
    def __init__(self, on_reset_callback=None, on_clear_path_callback=None, on_drive_callback=None):
        self.on_reset_callback = on_reset_callback
        self.on_clear_path_callback = on_clear_path_callback
        self.on_drive_callback = on_drive_callback

        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(15, 8.5), facecolor='#0d1117')
        self.fig.canvas.manager.set_window_title("REACT Rover - Ground Control Station & 2D Mapping")

        # Connect keyboard teleoperation handlers
        self.fig.canvas.mpl_connect('key_press_event', self._on_key_press)
        self.fig.canvas.mpl_connect('key_release_event', self._on_key_release)

        gs = self.fig.add_gridspec(1, 2, width_ratios=[3.4, 1.3], wspace=0.12,
                                   left=0.04, right=0.97, top=0.93, bottom=0.08)

        self.ax_map = self.fig.add_subplot(gs[0, 0])
        self.ax_hud = self.fig.add_subplot(gs[0, 1])

        self._setup_map_axes()
        self._setup_hud_axes()
        self._setup_buttons()

        self.path_line, = self.ax_map.plot([], [], color='#00d4ff', linewidth=2.0, alpha=0.85, label='Trajectory')
        self.obstacle_scatter = self.ax_map.scatter([], [], s=20, c='#ff4757', marker='s', edgecolors='none', label='Obstacles')

        half_l = ROVER_LENGTH / 2.0
        half_w = ROVER_WIDTH / 2.0
        self.local_chassis = [
            (half_l, half_w * 0.5),
            (half_l * 1.2, 0.0),
            (half_l, -half_w * 0.5),
            (half_l, -half_w),
            (-half_l, -half_w),
            (-half_l, half_w),
            (half_l, half_w)
        ]
        self.rover_patch = Polygon(self.local_chassis, closed=True, facecolor='#2ed573', edgecolor='#ffffff', linewidth=1.5, zorder=10)
        self.ax_map.add_patch(self.rover_patch)
        self.heading_arrow = None

        self.ray_artists = {}
        colors = {"FL": "#ffa502", "FR": "#ffa502", "RL": "#70a1ff", "RR": "#70a1ff"}
        for s_key in SENSOR_CONFIG:
            line, = self.ax_map.plot([], [], color=colors.get(s_key, '#eccc68'),
                                     linestyle='--', linewidth=1.5, alpha=0.9, zorder=8)
            contact = Circle((0, 0), radius=0.06, color=colors.get(s_key, '#eccc68'), zorder=9)
            self.ax_map.add_patch(contact)
            self.ray_artists[s_key] = {"line": line, "contact": contact}

        rows = int(round(MAP_HEIGHT_METERS / MAP_RESOLUTION))
        cols = int(round(MAP_WIDTH_METERS / MAP_RESOLUTION))
        empty_grid = np.full((rows, cols), 0.5)
        extent = [-MAP_WIDTH_METERS/2, MAP_WIDTH_METERS/2, -MAP_HEIGHT_METERS/2, MAP_HEIGHT_METERS/2]
        self.grid_im = self.ax_map.imshow(empty_grid, cmap='bone_r', vmin=0.0, vmax=1.0,
                                          extent=extent, origin='lower', alpha=0.45, zorder=1)

    def _setup_map_axes(self):
        self.ax_map.set_facecolor('#161b22')
        self.ax_map.set_title("REACT ROVER // 2D HAZARD OCCUPANCY MAP", color='#58a6ff', fontsize=12, fontweight='bold', pad=10)
        self.ax_map.set_xlabel("Global X (East) [meters]", color='#8b949e', fontsize=10)
        self.ax_map.set_ylabel("Global Y (North) [meters]", color='#8b949e', fontsize=10)
        self.ax_map.set_xlim(-MAP_WIDTH_METERS/2, MAP_WIDTH_METERS/2)
        self.ax_map.set_ylim(-MAP_HEIGHT_METERS/2, MAP_HEIGHT_METERS/2)
        self.ax_map.set_aspect('equal', 'box')
        self.ax_map.grid(True, color='#30363d', linestyle=':', linewidth=0.8, alpha=0.7)
        self.ax_map.tick_params(colors='#8b949e')

    def _setup_hud_axes(self):
        self.ax_hud.set_facecolor('#161b22')
        self.ax_hud.set_title("ROVER TELEMETRY HUD", color='#7ee787', fontsize=11, fontweight='bold', pad=10)
        self.ax_hud.set_xticks([])
        self.ax_hud.set_yticks([])
        self.hud_text = self.ax_hud.text(
            0.05, 0.95, "Awaiting Telemetry...",
            color='#c9d1d9', fontsize=9.5, fontfamily='monospace',
            verticalalignment='top', transform=self.ax_hud.transAxes,
            linespacing=1.35
        )

    def _setup_buttons(self):
        ax_btn_reset = self.fig.add_axes([0.68, 0.02, 0.13, 0.04])
        self.btn_reset = Button(ax_btn_reset, 'Reset Map & Origin', color='#21262d', hovercolor='#da3633')
        self.btn_reset.label.set_color('#f85149')
        self.btn_reset.label.set_fontsize(9)
        self.btn_reset.on_clicked(lambda e: self.on_reset_callback() if self.on_reset_callback else None)

        ax_btn_clear = self.fig.add_axes([0.83, 0.02, 0.13, 0.04])
        self.btn_clear = Button(ax_btn_clear, 'Clear Path Trail', color='#21262d', hovercolor='#1f6feb')
        self.btn_clear.label.set_color('#58a6ff')
        self.btn_clear.label.set_fontsize(9)
        self.btn_clear.on_clicked(lambda e: self.on_clear_path_callback() if self.on_clear_path_callback else None)

    def _on_key_press(self, event):
        key = event.key
        if not key or not self.on_drive_callback:
            return
        if key in ['up', 'w', 'W']:
            self.on_drive_callback('up', True)
        elif key in ['down', 's', 'S']:
            self.on_drive_callback('down', True)
        elif key in ['left', 'a', 'A']:
            self.on_drive_callback('left', True)
        elif key in ['right', 'd', 'D']:
            self.on_drive_callback('right', True)
        elif key == ' ':
            self.on_drive_callback('stop', True)

    def _on_key_release(self, event):
        key = event.key
        if not key or not self.on_drive_callback:
            return
        if key in ['up', 'w', 'W']:
            self.on_drive_callback('up', False)
        elif key in ['down', 's', 'S']:
            self.on_drive_callback('down', False)
        elif key in ['left', 'a', 'A']:
            self.on_drive_callback('left', False)
        elif key in ['right', 'd', 'D']:
            self.on_drive_callback('right', False)

    def update(self, rover_x, rover_y, heading_rad, path_x, path_y, obstacle_points, sensor_rays, prob_grid, telemetry):
        if path_x and path_y:
            self.path_line.set_data(path_x, path_y)

        if obstacle_points:
            ox, oy = zip(*obstacle_points)
            self.obstacle_scatter.set_offsets(np.c_[ox, oy])

        transformed_chassis = CoordinateTransformer.transform_polygon(
            self.local_chassis, rover_x, rover_y, heading_rad
        )
        self.rover_patch.set_xy(transformed_chassis)

        arrow_len = 0.5
        dx = arrow_len * math.cos(heading_rad)
        dy = arrow_len * math.sin(heading_rad)
        if self.heading_arrow:
            self.heading_arrow.remove()
        self.heading_arrow = self.ax_map.annotate(
            "", xy=(rover_x + dx, rover_y + dy), xytext=(rover_x, rover_y),
            arrowprops=dict(arrowstyle="->", color="#ffffff", lw=2.0)
        )

        for s_key, r_art in self.ray_artists.items():
            s_data = sensor_rays.get(s_key)
            if s_data and s_data.get("origin") and s_data.get("target"):
                ox, oy = s_data["origin"]
                tx, ty = s_data["target"]
                r_art["line"].set_data([ox, tx], [oy, ty])
                r_art["contact"].center = (tx, ty)
                r_art["contact"].set_visible(s_data.get("valid", False))
            else:
                r_art["line"].set_data([], [])
                r_art["contact"].set_visible(False)

        if prob_grid is not None:
            self.grid_im.set_data(prob_grid)

        lines = [
            f"SYSTEM: REACT ROVER v2.4",
            f"MODE  : {'SIMULATION' if telemetry.get('is_sim') else 'HARDWARE'}",
            f"LINK  : {telemetry.get('status', 'STANDBY')[:24]}",
            f"PKTS  : {telemetry.get('packet_id', 0)}",
            "─" * 28,
            "POSITION & HEADING",
            f"  X      : {rover_x:+6.2f} m",
            f"  Y      : {rover_y:+6.2f} m",
            f"  Heading: {(math.degrees(heading_rad)) % 360.0:5.1f}°",
            f"  Speed  : {telemetry.get('speed', 0.0):5.2f} m/s",
            "─" * 28,
            "ULTRASONIC RANGES (cm)",
            f"  FL: {telemetry.get('FL', 0.0)*100:5.1f}   FR: {telemetry.get('FR', 0.0)*100:5.1f}",
            f"  RL: {telemetry.get('RL', 0.0)*100:5.1f}   RR: {telemetry.get('RR', 0.0)*100:5.1f}",
            "─" * 28,
            "IMU ACCEL / GYRO",
            f"  AX: {telemetry.get('ax', 0.0):+5.2f}g   GX: {telemetry.get('gx', 0.0):+6.1f}°/s",
            f"  AY: {telemetry.get('ay', 0.0):+5.2f}g   GY: {telemetry.get('gy', 0.0):+6.1f}°/s",
            f"  AZ: {telemetry.get('az', 1.0):+5.2f}g   GZ: {telemetry.get('gz', 0.0):+6.1f}°/s",
            "─" * 28,
            "MAGNETOMETER [uT]",
            f"  MX: {telemetry.get('mx', 0.0):+6.1f}  MY: {telemetry.get('my', 0.0):+6.1f}  MZ: {telemetry.get('mz', 0.0):+6.1f}",
            "─" * 28,
            "TELEOPERATION [KEYBOARD]",
            f"  DRIVE  : {telemetry.get('drive_status', 'STANDBY')}",
            f"  KEYS   : [↑/W] Fwd  [↓/S] Rev",
            f"           [←/A] Left [→/D] Right",
            f"           [SPACE] Halt",
            "─" * 28,
            f"OBSTACLES LOGGED: {telemetry.get('obstacle_count', 0)}"
        ]
        self.hud_text.set_text("\\n".join(lines))
`
  },
  {
    name: "react_sensor_node.ino",
    path: "react_mapping/arduino/react_sensor_node.ino",
    category: "arduino",
    description: "Arduino Uno/Nano C++ firmware with L298N Dual H-Bridge motor driver teleoperation: Direction & PWM speed control (ENA D9, IN1 D8, IN2 D10, ENB D11, IN3 D12, IN4 A0), MPU6050/6500 IMU, 3 HC-SR04 Ultrasonics, safety watchdog, and DATA:#... serial telemetry.",
    language: "cpp",
    content: `/*
===============================================================================
 REACT (Reactive Hazard Exploration/Response Rover)
 Sensor & Motor Actuation Node Firmware: react_sensor_node.ino
 Target Board: Arduino UNO / Nano (ATmega328P)
 Baud Rate   : 9600 baud (Matching config.py and Web GCS)
===============================================================================

 HARDWARE PIN ASSIGNMENTS:

 1. L298N DUAL H-BRIDGE MOTOR DRIVER:
    LEFT MOTOR (Channel A):
      ENA -> Pin D9  [PWM Speed Control - Timer 1]
      IN1 -> Pin D8  [Direction Control 1]
      IN2 -> Pin D10 [Direction Control 2]
    RIGHT MOTOR (Channel B):
      ENB -> Pin D11 [PWM Speed Control - Timer 2]
      IN3 -> Pin D12 [Direction Control 3]
      IN4 -> Pin A0  [Direction Control 4 - Configured as Digital Output]

    L298N POWER CONNECTIONS:
      VMS / 12V -> External Motor Battery (+) (7.4V - 12V DC)
      GND       -> Battery (-) AND Arduino GND (COMMON GROUND IS CRITICAL!)
      5V        -> 5V Logic (Keep 5V jumper ON if battery <= 12V)

    NOTE ON JUMPERS:
      - If ENA & ENB black jumpers are KEPT ON: Motors run at full 100% battery speed.
      - If ENA & ENB jumpers are REMOVED: Wire to D9 & D11 for full PWM speed control.

 2. MPU-6050 / MPU-6500 6-AXIS IMU (I2C):
      SDA -> Pin A4
      SCL -> Pin A5
      AD0 -> GND (I2C Address 0x68)
      VCC -> 5V (or 3.3V)
      GND -> Arduino GND

 3. HC-SR04 ULTRASONIC RANGEFINDERS:
    FRONT (0° Forward):
      TRIG -> Pin D2
      ECHO -> Pin D3
    LEFT (+90° Lateral Left):
      TRIG -> Pin D4
      ECHO -> Pin D5
    RIGHT (-90° Lateral Right):
      TRIG -> Pin D6
      ECHO -> Pin D7

 4. STATUS INDICATOR:
      LED_BUILTIN -> Pin D13 (Heartbeat blink on telemetry transmit)

 SERIAL TELEMETRY OUTPUT (10 Hz):
   DATA:#<seq>,DF=<cm>,DL=<cm>,DR=<cm>,HDG=<deg>,GZ=<dps>,AX=<g>,AY=<g>,AZ=<g>*

 SERIAL INPUT COMMANDS (From Python GCS / Web GCS):
   'F' / "FORWARD"  -> Drive Forward
   'B' / "BACKWARD" -> Drive Backward (Reverse)
   'L' / "LEFT"     -> Spin Left in place
   'R' / "RIGHT"    -> Spin Right in place
   'S' / ' ' / "STOP" -> Brake / Stop both motors
   '1' - '5'        -> Speed Presets (1=45%, 2=60%, 3=78%, 4=90%, 5=100%)
   '+' / '-'        -> Speed Increment / Decrement (+-15 PWM)
   "SPD:<val>"      -> Set explicit PWM speed (60 - 255)
===============================================================================
*/

#include <Wire.h>

#define MPU_ADDR 0x68

// ============================================================================
// L298N MOTOR DRIVER PIN DEFINITIONS
// ============================================================================
#define ENA  9   // Left Motor PWM Speed (Timer 1)
#define IN1  8   // Left Motor Direction 1
#define IN2  10  // Left Motor Direction 2

#define ENB  11  // Right Motor PWM Speed (Timer 2)
#define IN3  12  // Right Motor Direction 1
#define IN4  A0  // Right Motor Direction 2 (A0 used as digital output)

// Direction inversion toggles (set to true if your motor wires are reversed)
const bool INVERT_LEFT_MOTOR  = false;
const bool INVERT_RIGHT_MOTOR = false;

// Motor speed variables (0 to 255 PWM)
const uint8_t DEFAULT_SPEED = 200; // ~78% duty cycle
const uint8_t TURN_SPEED    = 190; // Balanced pivot turn speed
const uint8_t MIN_SPEED     = 60;  // Minimum PWM to overcome motor gearbox friction
uint8_t currentSpeed        = DEFAULT_SPEED;

// Current state tracking
char currentDriveState = 'S'; // 'S'=Stop, 'F'=Forward, 'B'=Backward, 'L'=Left, 'R'=Right

// Fail-safe Watchdog: Auto-stop motors if Python disconnects or stops sending
const bool WATCHDOG_ENABLED = true;
const unsigned long WATCHDOG_TIMEOUT_MS = 2000; // 2.0 seconds timeout
unsigned long lastCommandTime = 0;

// Serial command receive buffer
char serialCmdBuffer[16];
uint8_t serialCmdIndex = 0;


// ============================================================================
// ULTRASONIC PINS
// ============================================================================
#define FRONT_TRIG 2
#define FRONT_ECHO 3

#define LEFT_TRIG  4
#define LEFT_ECHO  5

#define RIGHT_TRIG 6
#define RIGHT_ECHO 7


// ============================================================================
// SENSOR & TELEMETRY VARIABLES
// ============================================================================
float angleZ = 0.0;
float gyroZOffset = 0.0;
float gyroZ = 0.0;

float ax = 0.0;
float ay = 0.0;
float az = 1.0;

float distFront = 0.0;
float distLeft  = 0.0;
float distRight = 0.0;

unsigned long previousTime = 0;
unsigned long lastPacketTime = 0;
unsigned long packetCounter = 0;


// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
void initMotors();
void setMotorOutputs(int leftSpeed, int rightSpeed);
void driveForward();
void driveBackward();
void driveLeft();
void driveRight();
void driveStop();
void processCommand(char cmd);
void processStringCommand(const char* cmdStr);
void readSerialCommands();
void checkMotorWatchdog();

int16_t readGyroZ();
void readAccel(float &outAx, float &outAy, float &outAz);
float measureDistanceCm(int trigPin, int echoPin);
void transmitTelemetry();


// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // --------------------------------------------------------------------------
  // 1. INITIALIZE MOTOR DRIVER PINS (L298N)
  // --------------------------------------------------------------------------
  initMotors();

  // --------------------------------------------------------------------------
  // 2. ULTRASONIC SENSOR PINS
  // --------------------------------------------------------------------------
  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);

  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(LEFT_ECHO, INPUT);

  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);

  digitalWrite(FRONT_TRIG, LOW);
  digitalWrite(LEFT_TRIG, LOW);
  digitalWrite(RIGHT_TRIG, LOW);

  // --------------------------------------------------------------------------
  // 3. STATUS LED
  // --------------------------------------------------------------------------
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // --------------------------------------------------------------------------
  // 4. SERIAL COMMUNICATION
  // --------------------------------------------------------------------------
  Serial.begin(9600);

  // --------------------------------------------------------------------------
  // 5. I2C BUS & MPU-6050 INITIALIZATION
  // --------------------------------------------------------------------------
  Wire.begin();
  delay(250);

  // Wake up MPU-6050 (clear sleep bit in PWR_MGMT_1)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  byte error = Wire.endTransmission();

  if (error != 0) {
    Serial.println(F("DATA:ERROR=MPU_NOT_FOUND"));
  }

  // --------------------------------------------------------------------------
  // 6. GYROSCOPE ZERO-RATE BIAS CALIBRATION
  // --------------------------------------------------------------------------
  Serial.println(F("DATA:CALIBRATION_START"));
  delay(500);

  long sum = 0;
  for (int i = 0; i < 400; i++) {
    int16_t gz = readGyroZ();
    sum += gz;
    delay(2);
  }
  gyroZOffset = sum / 400.0;

  // Initialize orientation and timing
  angleZ = 0.0;
  previousTime = millis();
  lastPacketTime = millis();
  lastCommandTime = millis();

  Serial.println(F("DATA:CALIBRATION_COMPLETE"));
  Serial.println(F("DATA:MOTORS_INITIALIZED_L298N"));

  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
}


// ============================================================================
// MAIN CONTROL LOOP
// ============================================================================
void loop() {
  // --------------------------------------------------------------------------
  // A. PROCESS INCOMING SERIAL COMMANDS FROM PYTHON / WEB GCS
  // --------------------------------------------------------------------------
  readSerialCommands();

  // --------------------------------------------------------------------------
  // B. MOTOR SAFETY WATCHDOG CHECK
  // --------------------------------------------------------------------------
  checkMotorWatchdog();

  // --------------------------------------------------------------------------
  // C. READ GYRO Z AND COMPUTE RELATIVE HEADING INTEGRATION
  // --------------------------------------------------------------------------
  int16_t gzRaw = readGyroZ();
  gyroZ = (gzRaw - gyroZOffset) / 131.0; // 131 LSB / (deg/s) for +-250 dps range

  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  if (dt <= 0.0 || dt > 0.5) {
    dt = 0.02;
  }

  // Integrate heading angle in degrees [0, 360)
  angleZ += gyroZ * dt;
  if (angleZ >= 360.0) angleZ -= 360.0;
  if (angleZ < 0.0)    angleZ += 360.0;

  // --------------------------------------------------------------------------
  // D. READ ACCELEROMETER
  // --------------------------------------------------------------------------
  readAccel(ax, ay, az);

  // --------------------------------------------------------------------------
  // E. READ ULTRASONIC TRANSDUCERS (WITH INTERMEDIATE SERIAL POLLING)
  // --------------------------------------------------------------------------
  distFront = measureDistanceCm(FRONT_TRIG, FRONT_ECHO);
  readSerialCommands(); // Keep steering sub-millisecond responsive
  delayMicroseconds(800);

  distLeft = measureDistanceCm(LEFT_TRIG, LEFT_ECHO);
  readSerialCommands();
  delayMicroseconds(800);

  distRight = measureDistanceCm(RIGHT_TRIG, RIGHT_ECHO);
  readSerialCommands();

  // --------------------------------------------------------------------------
  // F. TRANSMIT TELEMETRY PACKET AT ~10-15 HZ
  // --------------------------------------------------------------------------
  currentTime = millis();
  if (currentTime - lastPacketTime >= 100) {
    lastPacketTime = currentTime;
    packetCounter++;

    transmitTelemetry();

    // Toggle heartbeat LED
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  delay(15);
}


// ============================================================================
// L298N MOTOR CONTROL FUNCTIONS
// ============================================================================

void initMotors() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  driveStop();
}

/**
 * Directly controls Left and Right motor speeds and directions.
 * leftSpeed / rightSpeed range: -255 (full reverse) to +255 (full forward).
 */
void setMotorOutputs(int leftSpeed, int rightSpeed) {
  // Apply direction inversion if configured
  if (INVERT_LEFT_MOTOR)  leftSpeed  = -leftSpeed;
  if (INVERT_RIGHT_MOTOR) rightSpeed = -rightSpeed;

  // Left Motor Control (Motor A)
  if (leftSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, constrain(leftSpeed, 0, 255));
  } else if (leftSpeed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, constrain(-leftSpeed, 0, 255));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }

  // Right Motor Control (Motor B)
  if (rightSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, constrain(rightSpeed, 0, 255));
  } else if (rightSpeed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, constrain(-rightSpeed, 0, 255));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void driveForward() {
  currentDriveState = 'F';
  lastCommandTime = millis();
  setMotorOutputs(currentSpeed, currentSpeed);
}

void driveBackward() {
  currentDriveState = 'B';
  lastCommandTime = millis();
  setMotorOutputs(-currentSpeed, -currentSpeed);
}

void driveLeft() {
  currentDriveState = 'L';
  lastCommandTime = millis();
  // Differential spin in place: Left reverse, Right forward
  setMotorOutputs(-turnSpeed, turnSpeed);
}

void driveRight() {
  currentDriveState = 'R';
  lastCommandTime = millis();
  // Differential spin in place: Left forward, Right reverse
  setMotorOutputs(turnSpeed, -turnSpeed);
}

void driveStop() {
  currentDriveState = 'S';
  setMotorOutputs(0, 0);
}

/**
 * Executes a single-character command immediately.
 */
void processCommand(char cmd) {
  switch (cmd) {
    case 'F':
    case 'f':
      driveForward();
      break;

    case 'B':
    case 'b':
      driveBackward();
      break;

    case 'L':
    case 'l':
      driveLeft();
      break;

    case 'R':
    case 'r':
      driveRight();
      break;

    case 'S':
    case 's':
    case ' ': // Spacebar emergency halt
      driveStop();
      break;

    // Speed Preset levels
    case '1':
      currentSpeed = 120;
      break;
    case '2':
      currentSpeed = 160;
      break;
    case '3':
      currentSpeed = 200;
      break;
    case '4':
      currentSpeed = 230;
      break;
    case '5':
      currentSpeed = 255;
      break;

    // Speed increments
    case '+':
      if (currentSpeed <= 235) currentSpeed += 20;
      else currentSpeed = 255;
      break;

    case '-':
      if (currentSpeed >= MIN_SPEED + 20) currentSpeed -= 20;
      else currentSpeed = MIN_SPEED;
      break;

    default:
      break;
  }
}

/**
 * Handles multi-character string commands like "FORWARD", "STOP", "SPD:210".
 */
void processStringCommand(const char* cmdStr) {
  if (strcmp(cmdStr, "FORWARD") == 0 || strcmp(cmdStr, "FWD") == 0) {
    driveForward();
  } else if (strcmp(cmdStr, "BACKWARD") == 0 || strcmp(cmdStr, "REV") == 0) {
    driveBackward();
  } else if (strcmp(cmdStr, "LEFT") == 0) {
    driveLeft();
  } else if (strcmp(cmdStr, "RIGHT") == 0) {
    driveRight();
  } else if (strcmp(cmdStr, "STOP") == 0 || strcmp(cmdStr, "HALT") == 0) {
    driveStop();
  } else if (strncmp(cmdStr, "SPD:", 4) == 0 || strncmp(cmdStr, "SPEED:", 6) == 0) {
    int val = atoi(strchr(cmdStr, ':') + 1);
    if (val >= MIN_SPEED && val <= 255) {
      currentSpeed = (uint8_t)val;
    }
  }
}

/**
 * Non-blocking serial command reader.
 * Reads single characters and buffers string commands until newline.
 */
void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    // Check for line terminator
    if (c == '\n' || c == '\r') {
      if (serialCmdIndex > 0) {
        serialCmdBuffer[serialCmdIndex] = '\0';
        if (serialCmdIndex == 1) {
          processCommand(serialCmdBuffer[0]);
        } else {
          processStringCommand(serialCmdBuffer);
        }
        serialCmdIndex = 0;
      }
    } else {
      // Immediate response for single-letter commands
      if (c == 'F' || c == 'f' || c == 'B' || c == 'b' ||
          c == 'L' || c == 'l' || c == 'R' || c == 'r' ||
          c == 'S' || c == 's' || c == ' ' ||
          c == '1' || c == '2' || c == '3' || c == '4' || c == '5' ||
          c == '+' || c == '-') {
        processCommand(c);
      }

      // Also buffer for potential multi-character commands (e.g. "SPD:200")
      if (serialCmdIndex < sizeof(serialCmdBuffer) - 1) {
        serialCmdBuffer[serialCmdIndex++] = c;
      }
    }
  }
}

/**
 * Watchdog safety: Stops rover if no new driving command arrives within timeout.
 */
void checkMotorWatchdog() {
  if (WATCHDOG_ENABLED && currentDriveState != 'S') {
    if (millis() - lastCommandTime > WATCHDOG_TIMEOUT_MS) {
      driveStop();
    }
  }
}


// ============================================================================
// SENSOR READING IMPLEMENTATIONS
// ============================================================================

int16_t readGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47); // GYRO_ZOUT_H register
  if (Wire.endTransmission(false) != 0) {
    return 0;
  }

  Wire.requestFrom(MPU_ADDR, 2, true);
  if (Wire.available() < 2) {
    return 0;
  }

  int16_t gz = (Wire.read() << 8) | Wire.read();
  return gz;
}

void readAccel(float &outAx, float &outAy, float &outAz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H register
  if (Wire.endTransmission(false) != 0) {
    return;
  }

  Wire.requestFrom(MPU_ADDR, 6, true);
  if (Wire.available() < 6) {
    return;
  }

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();

  // Convert raw 16-bit to acceleration in units of g (16384 LSB/g at +-2g)
  outAx = rawAx / 16384.0;
  outAy = rawAy / 16384.0;
  outAz = rawAz / 16384.0;
}

float measureDistanceCm(int trigPin, int echoPin) {
  // Clear trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send 10 microsecond trigger pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure echo pulse (25000 us timeout ≈ 4.3 meters maximum)
  unsigned long duration = pulseIn(echoPin, HIGH, 25000);

  if (duration == 0) {
    return 400.0;
  }

  // Distance in cm = duration * 0.0343 / 2
  float distance = duration * 0.0343 / 2.0;

  if (distance < 2.0)   distance = 2.0;
  if (distance > 400.0) distance = 400.0;

  return distance;
}


// ============================================================================
// TRANSMIT TELEMETRY PACKET OVER SERIAL
// ============================================================================
void transmitTelemetry() {
  Serial.print(F("DATA:#"));
  Serial.print(packetCounter);

  Serial.print(F(",DF="));
  Serial.print(distFront, 1);

  Serial.print(F(",DL="));
  Serial.print(distLeft, 1);

  Serial.print(F(",DR="));
  Serial.print(distRight, 1);

  Serial.print(F(",HDG="));
  Serial.print(angleZ, 2);

  Serial.print(F(",GZ="));
  Serial.print(gyroZ, 2);

  Serial.print(F(",AX="));
  Serial.print(ax, 2);

  Serial.print(F(",AY="));
  Serial.print(ay, 2);

  Serial.print(F(",AZ="));
  Serial.print(az, 2);

  Serial.println(F("*"));
}
`
  },
  {
    name: "react_simulated_node.ino",
    path: "react_mapping/arduino/react_simulated_node.ino",
    category: "arduino",
    description: "Arduino mock simulation firmware: Sends realistic ultrasonic ray-cast & IMU packets while static and rotating in place (No sensors required!).",
    language: "cpp",
    content: `/*
 ==============================================================================
 REACT (Reactive Hazard Exploration/Response Rover)
 Simulated Sensor Node Firmware: react_simulated_node.ino
 Target Board: Arduino Uno / Nano / Mega / ESP32
 ==============================================================================
 PURPOSE:
 This sketch flashes directly to an Arduino board to test the complete Python
 Ground Control Station (GCS) and 2D mapping pipeline WITHOUT requiring ANY
 physical sensors (HC-SR04, MPU-6050, HMC5883L) to be connected.

 SIMULATION BEHAVIOR:
 1. Stationarity: The rover vehicle is STATIC at fixed coordinate (X=0.0m, Y=0.0m).
 2. Rotation: The rover ROTATES IN PLACE standing at a single point around its
    vertical Z-axis at a configurable rate (default +18.0 deg/s -> 360° in 20s).
 3. Ray-Casting: Calculates geometric distance from each of the 4 transducers
    (FL=+45°, FR=-45°, RL=+135°, RR=-135°) to virtual room walls and pillars.
 4. IMU & Magnetometer:
    - AX, AY ~ 0.0g (level on ground, plus subtle vibration noise)
    - AZ ~ 1.0g (earth gravity vector)
    - GZ = +18.0 deg/s (matches real-time rotational rate)
    - MX, MY = rotated magnetic vector as heading sweeps 360° (tilt-compensated)
    - MZ = +38.0 uT (vertical Earth magnetic field)
 5. Output: Formats valid ASCII telemetry packets with XOR checksum identical to
    the real sensor node at 15 Hz (115200 baud).

 INTERACTIVE SERIAL COMMANDS (via Arduino Serial Monitor or Python):
  ' ' or 'p' -> Pause / Resume rotation
  'c'        -> Toggle rotation direction (Clockwise <-> Counter-Clockwise)
  '+'        -> Increase rotation speed (+5 deg/s)
  '-'        -> Decrease rotation speed (-5 deg/s)
  'r'        -> Reset heading to 0°
  'm'        -> Toggle Mode (Continuous 360° spin <-> Oscillating sweep +-90°)
  '?' or 'h' -> Print simulation status & help
 ==============================================================================
*/

#include <math.h>

const unsigned long PACKET_INTERVAL_MS = 66; // ~15 Hz telemetry output rate
const long SERIAL_BAUD = 115200;

// Virtual Room Boundaries (meters)
const float ROOM_X_MAX = 2.5;  // East wall at +2.5m
const float ROOM_X_MIN = -2.5; // West wall at -2.5m
const float ROOM_Y_MAX = 2.0;  // North wall at +2.0m
const float ROOM_Y_MIN = -2.0; // South wall at -2.0m

// Virtual Obstacle Pillar (meters)
const float PILLAR_X = 1.0;
const float PILLAR_Y = 0.8;
const float PILLAR_RADIUS = 0.35; // 35cm radius cylindrical column

// Transducer mount offsets on chassis (matching config.py)
const float SENSOR_FL_OX = 0.175;
const float SENSOR_FL_OY = 0.125;
const float SENSOR_FL_ANGLE_RAD = 0.785398; // +45 deg

const float SENSOR_FR_OX = 0.175;
const float SENSOR_FR_OY = -0.125;
const float SENSOR_FR_ANGLE_RAD = -0.785398; // -45 deg

const float SENSOR_RL_OX = -0.175;
const float SENSOR_RL_OY = 0.125;
const float SENSOR_RL_ANGLE_RAD = 2.356194; // +135 deg

const float SENSOR_RR_OX = -0.175;
const float SENSOR_RR_OY = -0.125;
const float SENSOR_RR_ANGLE_RAD = -2.356194; // -135 deg

unsigned long lastPacketTime = 0;
unsigned long lastTickTime = 0;
unsigned long packetCounter = 0;

float currentHeadingRad = 0.0;
float rotationSpeedDegPerSec = 18.0; // 18 deg/s (20s per 360 deg turn)
bool rotationPaused = false;
bool rotateClockwise = false;
bool oscillateMode = false;
float sweepDirection = 1.0;

const float MAG_HORIZ = 32.0;
const float MAG_VERT = 38.0;

float getNoise(float range) {
  long r = random(-1000, 1000);
  return (float)r * (range / 1000.0);
}

float castUltrasonicRay(float origX, float origY, float beamAngleRad) {
  float ux = cos(beamAngleRad);
  float uy = sin(beamAngleRad);
  float minDistance = 4.0;

  if (ux > 0.0001) {
    float t = (ROOM_X_MAX - origX) / ux;
    if (t > 0.0 && t < minDistance) {
      float hitY = origY + t * uy;
      if (hitY >= ROOM_Y_MIN && hitY <= ROOM_Y_MAX) minDistance = t;
    }
  } else if (ux < -0.0001) {
    float t = (ROOM_X_MIN - origX) / ux;
    if (t > 0.0 && t < minDistance) {
      float hitY = origY + t * uy;
      if (hitY >= ROOM_Y_MIN && hitY <= ROOM_Y_MAX) minDistance = t;
    }
  }

  if (uy > 0.0001) {
    float t = (ROOM_Y_MAX - origY) / uy;
    if (t > 0.0 && t < minDistance) {
      float hitX = origX + t * ux;
      if (hitX >= ROOM_X_MIN && hitX <= ROOM_X_MAX) minDistance = t;
    }
  } else if (uy < -0.0001) {
    float t = (ROOM_Y_MIN - origY) / uy;
    if (t > 0.0 && t < minDistance) {
      float hitX = origX + t * ux;
      if (hitX >= ROOM_X_MIN && hitX <= ROOM_X_MAX) minDistance = t;
    }
  }

  float dx = PILLAR_X - origX;
  float dy = PILLAR_Y - origY;
  float proj = dx * ux + dy * uy;
  if (proj > 0.0) {
    float distSq = (dx * dx + dy * dy) - (proj * proj);
    float radSq = PILLAR_RADIUS * PILLAR_RADIUS;
    if (distSq >= 0.0 && distSq < radSq) {
      float chord = sqrt(radSq - distSq);
      float hitDist = proj - chord;
      if (hitDist > 0.0 && hitDist < minDistance) {
        minDistance = hitDist;
      }
    }
  }

  return minDistance;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 1500);
  randomSeed(analogRead(A0) ^ analogRead(A1));

  lastPacketTime = millis();
  lastTickTime = millis();

  Serial.println(F("=================================================="));
  Serial.println(F("REACT ROVER: ARDUINO MOCK SENSOR SIMULATION NODE"));
  Serial.println(F("Mode: STATIC VEHICLE (X=0, Y=0) ROTATING IN PLACE"));
  Serial.println(F("115200 Baud | 15 Hz Telemetry Rate"));
  Serial.println(F("Commands: [Space]=Pause/Resume, [c]=Dir, [+/-]=Speed, [r]=Reset"));
  Serial.println(F("=================================================="));
  Serial.println(F("#0,STATUS=REACT_SIMULATOR_READY*00"));
}

void processSerialInput() {
  while (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == ' ' || cmd == 'p' || cmd == 'P') {
      rotationPaused = !rotationPaused;
      Serial.print(F("#MSG,Rotation "));
      Serial.println(rotationPaused ? F("PAUSED") : F("RESUMED"));
    } else if (cmd == 'c' || cmd == 'C') {
      rotateClockwise = !rotateClockwise;
      Serial.print(F("#MSG,Direction set to: "));
      Serial.println(rotateClockwise ? F("CLOCKWISE") : F("COUNTER-CLOCKWISE"));
    } else if (cmd == '+') {
      rotationSpeedDegPerSec += 5.0;
      if (rotationSpeedDegPerSec > 90.0) rotationSpeedDegPerSec = 90.0;
      Serial.print(F("#MSG,Rotation Speed: "));
      Serial.print(rotationSpeedDegPerSec);
      Serial.println(F(" deg/s"));
    } else if (cmd == '-') {
      rotationSpeedDegPerSec -= 5.0;
      if (rotationSpeedDegPerSec < 2.0) rotationSpeedDegPerSec = 2.0;
      Serial.print(F("#MSG,Rotation Speed: "));
      Serial.print(rotationSpeedDegPerSec);
      Serial.println(F(" deg/s"));
    } else if (cmd == 'r' || cmd == 'R') {
      currentHeadingRad = 0.0;
      Serial.println(F("#MSG,Heading Reset to 0.0 deg"));
    } else if (cmd == 'm' || cmd == 'M') {
      oscillateMode = !oscillateMode;
      Serial.print(F("#MSG,Mode: "));
      Serial.println(oscillateMode ? F("OSCILLATING (+-90 deg)") : F("CONTINUOUS 360"));
    } else if (cmd == '?' || cmd == 'h' || cmd == 'H') {
      Serial.println(F("#MSG,Commands: [p]=Pause [c]=Dir [+/-]=Speed [r]=Reset [m]=Mode"));
    }
  }
}

void loop() {
  unsigned long now = millis();
  processSerialInput();

  float dt = (float)(now - lastTickTime) / 1000.0;
  lastTickTime = now;

  float currentGyroZ_dps = 0.0;

  if (!rotationPaused) {
    float rateDps = rotationSpeedDegPerSec;

    if (oscillateMode) {
      float currentDeg = currentHeadingRad * (180.0 / M_PI);
      if (currentDeg > 90.0) sweepDirection = -1.0;
      if (currentDeg < -90.0) sweepDirection = 1.0;

      float dHeading = sweepDirection * rateDps * dt * (M_PI / 180.0);
      currentHeadingRad += dHeading;
      currentGyroZ_dps = sweepDirection * rateDps;
    } else {
      float dirSign = rotateClockwise ? -1.0 : 1.0;
      float dHeading = dirSign * rateDps * dt * (M_PI / 180.0);
      currentHeadingRad += dHeading;

      while (currentHeadingRad >= 2.0 * M_PI) currentHeadingRad -= 2.0 * M_PI;
      while (currentHeadingRad < 0.0) currentHeadingRad += 2.0 * M_PI;

      currentGyroZ_dps = dirSign * rateDps;
    }
  }

  if (now - lastPacketTime >= PACKET_INTERVAL_MS) {
    lastPacketTime = now;
    packetCounter++;

    float cosH = cos(currentHeadingRad);
    float sinH = sin(currentHeadingRad);

    float flOrigX = SENSOR_FL_OX * cosH - SENSOR_FL_OY * sinH;
    float flOrigY = SENSOR_FL_OX * sinH + SENSOR_FL_OY * cosH;
    float flRayAngle = currentHeadingRad + SENSOR_FL_ANGLE_RAD;
    float distFL_m = castUltrasonicRay(flOrigX, flOrigY, flRayAngle) + getNoise(0.015);

    float frOrigX = SENSOR_FR_OX * cosH - SENSOR_FR_OY * sinH;
    float frOrigY = SENSOR_FR_OX * sinH + SENSOR_FR_OY * cosH;
    float frRayAngle = currentHeadingRad + SENSOR_FR_ANGLE_RAD;
    float distFR_m = castUltrasonicRay(frOrigX, frOrigY, frRayAngle) + getNoise(0.015);

    float rlOrigX = SENSOR_RL_OX * cosH - SENSOR_RL_OY * sinH;
    float rlOrigY = SENSOR_RL_OX * sinH + SENSOR_RL_OY * cosH;
    float rlRayAngle = currentHeadingRad + SENSOR_RL_ANGLE_RAD;
    float distRL_m = castUltrasonicRay(rlOrigX, rlOrigY, rlRayAngle) + getNoise(0.015);

    float rrOrigX = SENSOR_RR_OX * cosH - SENSOR_RR_OY * sinH;
    float rrOrigY = SENSOR_RR_OX * sinH + SENSOR_RR_OY * cosH;
    float rrRayAngle = currentHeadingRad + SENSOR_RR_ANGLE_RAD;
    float distRR_m = castUltrasonicRay(rrOrigX, rrOrigY, rrRayAngle) + getNoise(0.015);

    float distFL_cm = constrain(distFL_m * 100.0, 3.0, 400.0);
    float distFR_cm = constrain(distFR_m * 100.0, 3.0, 400.0);
    float distRL_cm = constrain(distRL_m * 100.0, 3.0, 400.0);
    float distRR_cm = constrain(distRR_m * 100.0, 3.0, 400.0);

    float ax = getNoise(0.02);
    float ay = getNoise(0.02);
    float az = 1.00 + getNoise(0.02);

    float gx = getNoise(0.2);
    float gy = getNoise(0.2);
    float gz = currentGyroZ_dps + getNoise(0.3);

    float mx = (MAG_HORIZ * cosH) + getNoise(0.4);
    float my = (-MAG_HORIZ * sinH) + getNoise(0.4);
    float mz = MAG_VERT + getNoise(0.3);

    transmitPacket(distFL_cm, distFR_cm, distRL_cm, distRR_cm,
                   ax, ay, az, gx, gy, gz, mx, my, mz);
  }
}

void transmitPacket(float d1, float d2, float d3, float d4,
                    float ax, float ay, float az,
                    float gx, float gy, float gz,
                    float mx, float my, float mz) {
  char sD1[10], sD2[10], sD3[10], sD4[10];
  char sAX[8],  sAY[8],  sAZ[8];
  char sGX[8],  sGY[8],  sGZ[8];
  char sMX[8],  sMY[8],  sMZ[8];

  dtostrf(d1, 0, 1, sD1);
  dtostrf(d2, 0, 1, sD2);
  dtostrf(d3, 0, 1, sD3);
  dtostrf(d4, 0, 1, sD4);

  dtostrf(ax, 0, 2, sAX);
  dtostrf(ay, 0, 2, sAY);
  dtostrf(az, 0, 2, sAZ);

  dtostrf(gx, 0, 1, sGX);
  dtostrf(gy, 0, 1, sGY);
  dtostrf(gz, 0, 1, sGZ);

  dtostrf(mx, 0, 1, sMX);
  dtostrf(my, 0, 1, sMY);
  dtostrf(mz, 0, 1, sMZ);

  char buffer[200];
  int len = snprintf(buffer, sizeof(buffer),
    "#%lu,D1=%s,D2=%s,D3=%s,D4=%s,AX=%s,AY=%s,AZ=%s,GX=%s,GY=%s,GZ=%s,MX=%s,MY=%s,MZ=%s",
    packetCounter,
    sD1, sD2, sD3, sD4,
    sAX, sAY, sAZ,
    sGX, sGY, sGZ,
    sMX, sMY, sMZ
  );

  byte checksum = 0;
  for (int i = 0; i < len; i++) {
    checksum ^= (byte)buffer[i];
  }

  Serial.print(buffer);
  Serial.print('*');
  if (checksum < 0x10) Serial.print('0');
  Serial.println(checksum, HEX);
}
`
  },
  {
    name: "requirements.txt",
    path: "react_mapping/requirements.txt",
    category: "config",
    description: "Python pip package dependencies: pyserial, numpy, matplotlib.",
    language: "plaintext",
    content: `pyserial>=3.5
numpy>=1.24.0
matplotlib>=3.7.0
`
  },
  {
    name: "README.md",
    path: "react_mapping/README.md",
    category: "docs",
    description: "System overview, Windows setup steps, and hardware switching documentation.",
    language: "markdown",
    content: `# REACT Rover - 2D Mapping & Tracking System

Designed for underground coal-mine and disaster environment exploration.

### Quick Start (Windows / Linux)
1. Install Python 3.10+
2. Open Command Prompt or Terminal in \`react_mapping\`:
   \`\`\`cmd
   python -m venv venv
   venv\\Scripts\\activate
   pip install -r requirements.txt
   \`\`\`
3. Run Software Simulation Mode (Zero hardware required):
   \`\`\`cmd
   python main.py
   \`\`\`
4. Test with Arduino Board without Sensors (\`react_simulated_node.ino\`):
   - Flash \`arduino/react_simulated_node.ino\` to your Arduino board via Arduino IDE.
   - Vehicle stays static at (0, 0) and rotates in place, ray-casting virtual walls and pillars.
   - Edit \`config.py\` and set \`SIMULATION_MODE = False\` and \`SERIAL_PORT = "COM3"\`.
   - Run \`python main.py\`.
5. Deploy on Real Hardware with Physical Sensors:
   - Flash \`arduino/react_sensor_node.ino\` to your Arduino Uno
   - Set \`SIMULATION_MODE = False\` in \`config.py\`
   - Run \`python main.py\`
`
  }
];
