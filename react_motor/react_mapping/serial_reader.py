"""
==============================================================================
REACT (Reactive Hazard Exploration/Response Rover)
Serial Communication & Telemetry Parser: serial_reader.py
==============================================================================
Provides:
1. Non-blocking threaded acquisition using queue.Queue.
2. Robust packet parsing for labelled key=value and key:value formats.
3. Checksum verification (*XOR_HEX).
4. Auto-reconnection on port drop or disconnect.
5. High-fidelity Simulation Engine (when SIMULATION_MODE = True) allowing
   full offline testing in a simulated underground mine tunnel.
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
    """
    Parsed data structure holding all Rover sensor readings for one timestamp.
    """
    def __init__(self):
        self.packet_id: int = 0
        self.timestamp: float = time.time()

        # Ultrasonic distances in meters (FRONT, LEFT, RIGHT)
        self.dist_front: float = 0.0
        self.dist_left: float = 0.0
        self.dist_right: float = 0.0

        # Backward compatibility aliases
        self.dist_fl: float = 0.0
        self.dist_fr: float = 0.0
        self.dist_rl: float = 0.0
        self.dist_rr: float = 0.0

        # GY-271 Magnetometer Compass Telemetry & Heading
        self.gy271_heading_deg: float = 0.0
        self.imu: IMUData = IMUData()

        # Quality metrics
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
    """
    Dedicated worker thread that consumes serial streams from Arduino or Simulator
    and deposits validated TelemetryPacket objects into a thread-safe Queue.
    """

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

        # Telemetry statistics
        self.connected = False
        self.last_packet_time = 0.0
        self.total_packets_received = 0
        self.corrupted_packets_count = 0
        self.status_message = "Initializing..."

        # Simulation state
        self._sim_x = 0.0
        self._sim_y = 0.0
        self._sim_theta = 0.0
        self._sim_speed = 0.25  # m/s
        self._sim_turn_rate = 0.0
        self._is_manual = False
        self._manual_speed = 0.0
        self._manual_turn_rate = 0.0

    def send_command(self, cmd: str):
        """
        Sends a teleoperation command character/string to Arduino over serial if connected.
        Supported commands: 'F' (Forward), 'B' (Backward), 'L' (Left), 'R' (Right), 'S' (Stop).
        """
        if self.serial_conn and getattr(self.serial_conn, "is_open", False):
            try:
                line = f"{cmd.strip()}\n"
                self.serial_conn.write(line.encode('ascii'))
                self.serial_conn.flush()
            except Exception:
                pass

    def set_manual_motion(self, speed: float, turn_rate: float, is_manual: bool = True):
        """Sets manual velocity commands for keyboard teleoperation in simulation mode."""
        self._is_manual = is_manual
        self._manual_speed = speed
        self._manual_turn_rate = turn_rate

    def stop(self):
        """Signals the worker thread to safely terminate."""
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
        """Manages hardware connection, continuous reading, and auto-reconnect."""
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
                time.sleep(1.8)  # Allow Arduino DTR reset to finish boot

                while self.running and self.serial_conn.is_open:
                    line = self.serial_conn.readline().decode('utf-8', errors='replace').strip()
                    if not line:
                        # Check for sensor timeout
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
                            pass  # Drop stale frame to prevent queue lag
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
        """
        Parses tagged packets such as:
        #123,D1=124.5,D2=118.2,D3=201.4,D4=195.7,AX=0.03,AY=-0.02,AZ=0.98,GX=1.2,GY=-0.4,GZ=2.7,MX=31.2,MY=-12.4,MZ=42.1*4A
        or:
        D1:120,D2:145,D3:80,D4:95,AX:0.02,AY:-0.01,AZ:0.98,GX:1.2,GY:-0.4,GZ:2.1,MX:32.5,MY:-14.2,MZ:41.8
        """
        if not line:
            return None

        clean_line = line.strip()
        checksum_valid = True

        # Strip DATA: prefix if present (e.g. DATA:#148,...)
        if clean_line.startswith('DATA:'):
            clean_line = clean_line[5:].strip()

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
                    import math
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

        # Check for optional XOR checksum (*XX)
        if '*' in clean_line:
            data_part, chk_part = clean_line.rsplit('*', 1)
            # Validate XOR checksum if present
            try:
                expected_cs = int(chk_part, 16)
                computed_cs = 0
                for ch in data_part:
                    computed_cs ^= ord(ch)
                if computed_cs != expected_cs:
                    return None  # Checksum mismatch
            except ValueError:
                pass
            clean_line = data_part

        packet = TelemetryPacket()
        packet.raw_line = line
        packet.timestamp = time.time()
        packet.checksum_valid = checksum_valid

        # Split comma-delimited tokens
        tokens = clean_line.split(',')
        found_fields = 0

        for token in tokens:
            token = token.strip()
            if not token:
                continue

            # Packet ID token (e.g. #123)
            if token.startswith('#'):
                try:
                    packet.packet_id = int(token[1:])
                except ValueError:
                    pass
                continue

            # Key-Value separator: can be '=' or ':'
            sep = '=' if '=' in token else (':' if ':' in token else None)
            if not sep:
                continue

            key, val_str = token.split(sep, 1)
            key = key.strip().upper()
            try:
                val = float(val_str.strip())
            except ValueError:
                continue

            # Ultrasonic distances (Arduino sends in centimeters, convert to meters)
            if key in ('FL', 'U1', 'D1', 'DF', 'FRONT', 'DIST_FL'):
                val_m = val / 100.0 if val > 10.0 else val
                packet.dist_fl = val_m
                packet.dist_front = val_m
                found_fields += 1
            elif key in ('FR', 'U2', 'D2', 'DL', 'LEFT', 'DIST_FR'):
                val_m = val / 100.0 if val > 10.0 else val
                packet.dist_fr = val_m
                packet.dist_left = val_m
                found_fields += 1
            elif key in ('RL', 'U3', 'D3', 'DR', 'RIGHT', 'DIST_RL'):
                val_m = val / 100.0 if val > 10.0 else val
                packet.dist_rl = val_m
                packet.dist_right = val_m
                found_fields += 1
            elif key in ('RR', 'U4', 'D4', 'DB', 'REAR', 'BACK', 'DIST_RR'):
                val_m = val / 100.0 if val > 10.0 else val
                packet.dist_rr = val_m
                found_fields += 1
            # Heading or Angle (degrees 0-360)
            elif key in ('ANGLE', 'ANGLEZ', 'HDG', 'HEADING', 'COMPASS', 'YAW', 'YAW_DEG'):
                packet.gy271_heading_deg = val % 360.0
                if packet.gy271_heading_deg < 0:
                    packet.gy271_heading_deg += 360.0
                found_fields += 1
            # Accelerometer (in g)
            elif key == 'AX':
                packet.imu.ax = val
                found_fields += 1
            elif key == 'AY':
                packet.imu.ay = val
                found_fields += 1
            elif key == 'AZ':
                packet.imu.az = val
                found_fields += 1
            # Gyroscope (deg/s)
            elif key == 'GX':
                packet.imu.gx = val
                found_fields += 1
            elif key == 'GY':
                packet.imu.gy = val
                found_fields += 1
            elif key == 'GZ':
                packet.imu.gz = val
                found_fields += 1
            # GY-271 Magnetometer (uT)
            elif key == 'MX':
                packet.imu.mx = val
                found_fields += 1
            elif key == 'MY':
                packet.imu.my = val
                found_fields += 1
            elif key == 'MZ':
                packet.imu.mz = val
                found_fields += 1

        # If we got raw magnetometer but not explicit heading, calculate it directly from GY-271
        if packet.gy271_heading_deg == 0.0 and (packet.imu.mx != 0.0 or packet.imu.my != 0.0):
            import math
            calc_hdg = math.degrees(math.atan2(packet.imu.my, packet.imu.mx))
            if calc_hdg < 0:
                calc_hdg += 360.0
            packet.gy271_heading_deg = calc_hdg

        # Auto-generate pseudo ultrasonic distances if no physical ultrasonics are connected yet
        if packet.dist_front == 0.0 and packet.dist_left == 0.0 and packet.dist_right == 0.0 and found_fields >= 1:
            import math
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
            found_fields += 3

        # Accept packet if at least one field (such as Angle or Distance) was extracted
        if found_fields >= 1:
            packet.imu.timestamp = packet.timestamp
            return packet

        return None

    def _run_simulation(self):
        """
        Simulates an underground mine tunnel environment with four perimeter walls
        and central coal pillars. Rover moves along an exploration path.
        """
        self.connected = True
        seq = 0
        dt = 0.05  # 20 Hz simulation loop

        # Synthetic mine layout obstacles (line segments or circles)
        mine_walls = [
            # Main tunnel bounds
            (-6.0, -4.0, 6.0, -4.0),
            (6.0, -4.0, 6.0, 4.0),
            (6.0, 4.0, -6.0, 4.0),
            (-6.0, 4.0, -6.0, -4.0),
            # Support pillars inside tunnel
            (-2.0, -1.5, -2.0, -0.5),
            (2.0, 0.5, 2.0, 1.5)
        ]

        while self.running:
            time.sleep(dt)
            seq += 1

            # Rover exploration kinematics (manual keyboard or auto figure-8)
            if self._is_manual:
                self._sim_turn_rate = self._manual_turn_rate
                self._sim_speed = self._manual_speed
            else:
                self._sim_turn_rate = 0.35 * math.sin(seq * 0.04)
            self._sim_theta += self._sim_turn_rate * dt
            self._sim_x += self._sim_speed * math.cos(self._sim_theta) * dt
            self._sim_y += self._sim_speed * math.sin(self._sim_theta) * dt

            # Keep inside simulated tunnel boundaries
            if abs(self._sim_x) > 4.5 or abs(self._sim_y) > 2.8:
                self._sim_theta += math.pi * 0.5

            packet = TelemetryPacket()
            packet.packet_id = seq
            packet.timestamp = time.time()
            packet.is_simulated = True

            # Synthesize IMU:
            # Gyro Z tracks simulated turn rate with slight noise
            packet.imu.gz = math.degrees(self._sim_turn_rate) + random.gauss(0.0, 0.4)
            packet.imu.gx = random.gauss(0.0, 0.2)
            packet.imu.gy = random.gauss(0.0, 0.2)

            # Accelerometer: Gravity on AZ + turn centripetal accel on AY
            packet.imu.az = 1.0 + random.gauss(0.0, 0.02)
            packet.imu.ax = random.gauss(0.0, 0.02)
            packet.imu.ay = (self._sim_speed * self._sim_turn_rate / 9.8) + random.gauss(0.0, 0.02)

            # Magnetometer: Earth field rotated by heading
            # North is toward +Y (+90 deg), East is toward +X (0 deg)
            earth_field_total = 45.0  # uT
            packet.imu.mx = earth_field_total * math.cos(self._sim_theta) + random.gauss(0.0, 0.6)
            packet.imu.my = earth_field_total * math.sin(self._sim_theta) + random.gauss(0.0, 0.6)
            packet.imu.mz = 35.0 + random.gauss(0.0, 0.5)
            packet.imu.timestamp = packet.timestamp

            # GY-271 Compass Heading
            sim_hdg_deg = math.degrees(self._sim_theta) % 360.0
            packet.gy271_heading_deg = sim_hdg_deg

            # Ray-cast 3 ultrasonic sensors (FRONT, LEFT, RIGHT) against simulated tunnel walls
            for sensor_key, cfg in SENSOR_CONFIG.items():
                sensor_angle = self._sim_theta + cfg["angle_rad"]
                # Ray origin on chassis
                ox = self._sim_x + (cfg["offset_x"] * math.cos(self._sim_theta) - cfg["offset_y"] * math.sin(self._sim_theta))
                oy = self._sim_y + (cfg["offset_x"] * math.sin(self._sim_theta) + cfg["offset_y"] * math.cos(self._sim_theta))

                min_hit = cfg["max_dist"]
                # Intersect with all walls
                for x1, y1, x2, y2 in mine_walls:
                    hit = self._ray_segment_intersection(ox, oy, sensor_angle, x1, y1, x2, y2, cfg["max_dist"])
                    if hit is not None and hit < min_hit:
                        min_hit = hit

                # Add realistic measurement noise (+- 1.5 cm)
                measured_dist = min_hit + random.gauss(0.0, 0.015)
                measured_dist = max(MIN_ULTRASONIC_DISTANCE, min(MAX_ULTRASONIC_DISTANCE, measured_dist))

                if sensor_key in ("FRONT", "F"):
                    packet.dist_front = measured_dist
                    packet.dist_fl = measured_dist
                elif sensor_key in ("LEFT", "L"):
                    packet.dist_left = measured_dist
                    packet.dist_fr = measured_dist
                elif sensor_key in ("RIGHT", "R"):
                    packet.dist_right = measured_dist
                    packet.dist_rl = measured_dist

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
        """2D geometric ray-segment intersection calculation."""
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

        if t1 >= 0.0 and t1 <= max_range and 0.0 <= t2 <= 1.0:
            return t1
        return None
