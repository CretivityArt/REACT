"""
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

        # 2. Digital Signal Filters (one per ultrasonic transducer: FRONT, LEFT, RIGHT)
        self.median_filters = {
            s: MedianFilter(window_size=config.ULTRASONIC_MEDIAN_WINDOW)
            for s in config.SENSOR_CONFIG.keys()
        }
        self.outlier_rejectors = {
            s: OutlierRejector(max_delta=1.8)
            for s in config.SENSOR_CONFIG.keys()
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
            alpha=getattr(config, "COMPLEMENTARY_ALPHA", 0.96),
            magnetic_declination_deg=getattr(config, "MAGNETIC_DECLINATION_DEG", 0.0)
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
        elif self.manual_linear_v != 0.0 or self.manual_angular_w != 0.0:
            # Continue manual dead-reckoning even between incoming serial frames
            if self.manual_angular_w != 0.0:
                current_h = self.odometry.heading_rad
                new_h = (current_h + self.manual_angular_w * dt) % (2.0 * math.pi)
                self.odometry.heading_rad = new_h
                self.heading_estimator.current_heading_rad = new_h
            if self.manual_linear_v != 0.0:
                self.odometry.update_from_velocity(self.manual_linear_v, self.odometry.heading_rad, dt)

        # 2. Extract visualization states
        rx, ry, heading_rad = self.odometry.get_pose()
        prob_grid = self.occupancy_map.get_probability_map()

        telemetry_hud = {
            "status": self.reader_thread.status_message,
            "is_sim": config.SIMULATION_MODE or self.last_packet.is_simulated,
            "packet_id": self.last_packet.packet_id,
            "errors": self.reader_thread.corrupted_packets_count,
            "speed": self.odometry.linear_velocity,
            "FRONT": self.last_packet.dist_front,
            "LEFT": self.last_packet.dist_left,
            "RIGHT": self.last_packet.dist_right,
            "HDG": self.last_packet.gy271_heading_deg,
            "drive_status": self.drive_status_str,
            "path_len": len(self.odometry.path_x),
            # Backward compatibility keys
            "FL": self.last_packet.dist_front,
            "FR": self.last_packet.dist_left,
            "RL": self.last_packet.dist_right,
            "RR": 0.0,
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
        """
        Executes the robotic data processing pipeline:
        Sensor data -> GY-271 Compass Heading -> Odometry -> Coordinate Transform -> Occupancy Mapping
        """
        # Step 1: Heading from GY-271 Digital Compass (or fall back to IMU fusion)
        clean_imu = self.imu_interface.process(packet.imu)
        if getattr(config, "USE_GY271_HEADING", True):
            yaw_rad = self.heading_estimator.update_from_gy271(
                packet.gy271_heading_deg, clean_imu.mx, clean_imu.my
            )
        else:
            yaw_rad, _, _ = self.heading_estimator.update(clean_imu)

        # Step 2: Update Odometry
        # If manual keyboard teleoperation is active, use commanded speed; otherwise use patrol speed
        if self.manual_linear_v != 0.0:
            linear_v = self.manual_linear_v
        elif self.is_manual_mode:
            linear_v = 0.0
        else:
            linear_v = 0.25 if (config.SIMULATION_MODE or packet.is_simulated) else 0.0
        rx, ry = self.odometry.update_from_velocity(linear_v, yaw_rad, dt)

        # Step 3: Process Ultrasonic Sensors (FRONT, LEFT, RIGHT)
        raw_dists = packet.get_distances()

        for s_key, raw_d in raw_dists.items():
            cfg = config.SENSOR_CONFIG[s_key]

            # Digital Filtering: Median filter + Outlier gating
            filt_d = self.median_filters[s_key].update(raw_d)
            filt_d = self.outlier_rejectors[s_key].filter(filt_d)

            valid_hit = (config.MIN_ULTRASONIC_DISTANCE <= filt_d <= config.MAX_ULTRASONIC_DISTANCE)
            effective_range = filt_d if valid_hit else cfg["max_dist"]

            # Calculate Global Sensor Origin on Rover Chassis
            ox, oy = CoordinateTransformer.sensor_origin_to_global_frame(
                cfg["offset_x"], cfg["offset_y"], rx, ry, yaw_rad
            )

            # Transform relative measurement into global coordinates
            x_rel, y_rel = CoordinateTransformer.sensor_to_rover_frame(
                effective_range, cfg["angle"], cfg["offset_x"], cfg["offset_y"]
            )
            gx, gy = CoordinateTransformer.rover_to_global_frame(
                x_rel, y_rel, rx, ry, yaw_rad
            )

            # Store for ray drawing
            self.sensor_rays_state[s_key] = {
                "origin": (ox, oy),
                "target": (gx, gy),
                "dist": filt_d,
                "valid": valid_hit
            }

            # Step 4: Insert Ray into 2D Occupancy Grid Map
            self.occupancy_map.update_ray(
                origin_gx=ox, origin_gy=oy,
                target_gx=gx, target_gy=gy,
                distance=filt_d,
                valid_hit=valid_hit
            )

    def reset_system(self):
        """User clicked [Reset Map & Origin]."""
        print("[GCS] Resetting Map, Trajectory, and Rover Origin...")
        self.occupancy_map.reset()
        self.odometry.set_origin(0.0, 0.0, 0.0)
        self.heading_estimator.reset_heading(0.0)
        for mf in self.median_filters.values():
            mf.reset()

    def clear_path(self):
        """User clicked [Clear Path Trail]."""
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
                # Immediate discrete step for instant path trace and tactile response
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
                turn_step = 0.14  # ~8 degrees
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

        # Update continuous velocities for hold-down driving
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
        print("\n[GCS] Shutting down REACT ground control station...")
        self.reader_thread.stop()
        sys.exit(0)


if __name__ == "__main__":
    app = ReactMappingSystem()
    app.start()
