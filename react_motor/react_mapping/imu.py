"""
==============================================================================
REACT (Reactive Hazard Exploration/Response Rover)
IMU & Magnetometer Driver/Parser: imu.py
==============================================================================
Modular interface for:
- MPU-6050 / MPU-6500 6-DOF IMU (3-axis Accelerometer + 3-axis Gyroscope)
- External 3-axis Magnetometer (HMC5883L, QMC5883L, or AK8963)
Note: MPU-6050 does NOT have an integrated compass. This module handles both
the 6-DOF internal data and 3-DOF external magnetometer packets seamlessly.
"""

import math
from typing import Dict, Any, Tuple


class IMUData:
    """
    Standardized container for 9-DOF inertial and magnetic telemetry.
    """
    def __init__(self,
                 ax: float = 0.0, ay: float = 0.0, az: float = 1.0,
                 gx: float = 0.0, gy: float = 0.0, gz: float = 0.0,
                 mx: float = 0.0, my: float = 0.0, mz: float = 0.0,
                 timestamp: float = 0.0):
        # Accelerometer in g (1.0 g approx 9.80665 m/s^2)
        self.ax = ax
        self.ay = ay
        self.az = az

        # Gyroscope in degrees per second (deg/s)
        self.gx = gx
        self.gy = gy
        self.gz = gz

        # Magnetometer in microteslas (uT) or raw calibrated units
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
    """
    Modular processor for IMU telemetry, calibration offsets, and tilt estimation.
    """
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
        # Gyro zero-rate bias offsets (deg/s)
        self.gyro_bias_x = gyro_bias_x
        self.gyro_bias_y = gyro_bias_y
        self.gyro_bias_z = gyro_bias_z

        # Magnetometer hard-iron offset and soft-iron scale factors
        self.mag_offset_x = mag_offset_x
        self.mag_offset_y = mag_offset_y
        self.mag_offset_z = mag_offset_z
        self.mag_scale_x = mag_scale_x
        self.mag_scale_y = mag_scale_y
        self.mag_scale_z = mag_scale_z

    def calibrate_gyro_bias(self, samples: list):
        """
        Calculates zero-rate drift offsets from a list of stationary IMUData samples.
        Rover must remain completely stationary during this procedure.
        """
        if not samples:
            return
        n = len(samples)
        self.gyro_bias_x = sum(s.gx for s in samples) / n
        self.gyro_bias_y = sum(s.gy for s in samples) / n
        self.gyro_bias_z = sum(s.gz for s in samples) / n

    def calibrate_magnetometer_hard_iron(self, raw_samples: list):
        """
        Hard-iron calibration using min/max bounding box method.
        Rover is rotated through 360 degrees in the mine or test area.
        """
        if len(raw_samples) < 20:
            return
        xs = [s.mx for s in raw_samples]
        ys = [s.my for s in raw_samples]
        zs = [s.mz for s in raw_samples]

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        min_z, max_z = min(zs), max(zs)

        # Hard-iron offset is the center of the magnetic sphere/ellipse
        self.mag_offset_x = (max_x + min_x) / 2.0
        self.mag_offset_y = (max_y + min_y) / 2.0
        self.mag_offset_z = (max_z + min_z) / 2.0

        # Soft-iron scale adjustment to circularize the response
        delta_x = (max_x - min_x) / 2.0
        delta_y = (max_y - min_y) / 2.0
        delta_z = (max_z - min_z) / 2.0
        avg_delta = (delta_x + delta_y + delta_z) / 3.0

        if delta_x > 0.001:
            self.mag_scale_x = avg_delta / delta_x
        if delta_y > 0.001:
            self.mag_scale_y = avg_delta / delta_y
        if delta_z > 0.001:
            self.mag_scale_z = avg_delta / delta_z

    def process(self, raw: IMUData) -> IMUData:
        """
        Applies calibration corrections to raw sensor readings.
        """
        # Subtract zero-rate bias from gyroscope
        clean_gx = raw.gx - self.gyro_bias_x
        clean_gy = raw.gy - self.gyro_bias_y
        clean_gz = raw.gz - self.gyro_bias_z

        # Apply hard-iron offset and soft-iron scale to magnetometer
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
        """
        Estimates static tilt angles (pitch, roll) in radians from gravity vector.
        Pitch: rotation about Y-axis (nose up/down)
        Roll: rotation about X-axis (tilt left/right)
        """
        # Standard aerospace gravity decomposition
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        return pitch, roll
