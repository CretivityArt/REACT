"""
==============================================================================
REACT (Reactive Hazard Exploration/Response Rover)
Sensor Fusion & Heading Estimation: sensor_fusion.py
==============================================================================
Implements sensor fusion to calculate drift-free rover heading (Yaw):
1. Gyroscope: High-frequency, low-latency relative angular changes.
2. Accelerometer: Resolves gravity vector to provide Pitch & Roll tilt angles.
3. Magnetometer: Provides absolute magnetic North reference.
4. Tilt-Compensated Compass: Projects magnetic field onto the horizontal plane.
5. Complementary Filter with phase unwrapping for robust heading estimation.
"""

import math
from typing import Tuple, Optional
from imu import IMUData, IMUInterface


def normalize_angle_rad(angle: float) -> float:
    """Normalizes an angle to the range [-pi, +pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def normalize_angle_deg(angle: float) -> float:
    """Normalizes an angle to the range [0, 360)."""
    angle = angle % 360.0
    if angle < 0:
        angle += 360.0
    return angle


class HeadingEstimator:
    """
    Sensor fusion engine for rover 2D heading determination.
    Fuses Gyro Z rate + Accelerometer Tilt + Magnetometer Horizontal Heading.
    """

    def __init__(self,
                 alpha: float = 0.96,
                 magnetic_declination_deg: float = 0.0):
        """
        :param alpha: Weight of gyroscope integration (typically 0.90 to 0.98).
                      Higher alpha = smoother, less sensitive to magnetic anomalies.
        :param magnetic_declination_deg: Difference between magnetic north and true north.
        """
        self.alpha = alpha
        self.declination_rad = math.radians(magnetic_declination_deg)

        # Estimated attitude states
        self.yaw_rad: float = 0.0          # Heading: 0 rad = East (+X), pi/2 = North (+Y)
        self.pitch_rad: float = 0.0        # Tilt forward/backward
        self.roll_rad: float = 0.0         # Tilt lateral
        self.angular_velocity_z: float = 0.0  # rad/s

        self.last_timestamp: Optional[float] = None
        self.initialized = False

    def reset_heading(self, initial_heading_deg: float = 0.0):
        """Resets the rover heading to a specified initial direction."""
        self.yaw_rad = math.radians(initial_heading_deg)
        self.last_timestamp = None

    def calculate_tilt_compensated_mag_heading(self,
                                               mx: float, my: float, mz: float,
                                               pitch: float, roll: float) -> float:
        """
        Rotates raw magnetic vector by pitch and roll into the horizontal ground plane.
        Prevents heading errors when the rover traverses uneven mine debris.

        Mathematics:
            Xh = mx * cos(pitch) + mz * sin(pitch)
            Yh = mx * sin(roll)*sin(pitch) + my * cos(roll) - mz * sin(roll)*cos(pitch)
            yaw_mag = atan2(-Yh, Xh) + declination
        """
        sin_p = math.sin(pitch)
        cos_p = math.cos(pitch)
        sin_r = math.sin(roll)
        cos_r = math.cos(roll)

        xh = mx * cos_p + mz * sin_p
        yh = mx * sin_r * sin_p + my * cos_r - mz * sin_r * cos_p

        # In robotics/mapping convention: atan2(Y, X)
        # For standard compass pointing where X is forward:
        mag_heading = math.atan2(yh, xh) + self.declination_rad
        return normalize_angle_rad(mag_heading)

    def update_from_gy271(self, heading_deg: float, mx: float = 0.0, my: float = 0.0) -> float:
        """
        Directly updates the vehicle heading using the GY-271 Digital Compass module
        (instead of relying on MPU gyroscope integration).
        """
        if heading_deg != 0.0:
            self.yaw_rad = normalize_angle_rad(math.radians(heading_deg))
        elif mx != 0.0 or my != 0.0:
            raw_hdg = math.atan2(my, mx) + self.declination_rad
            self.yaw_rad = normalize_angle_rad(raw_hdg)
        self.initialized = True
        return self.yaw_rad

    def update(self, imu: IMUData) -> Tuple[float, float, float]:
        """
        Updates heading state with new IMU and Magnetometer telemetry.
        :return: (yaw_rad, pitch_rad, roll_rad)
        """
        # 1. Compute Pitch and Roll from gravity vector (accelerometer)
        self.pitch_rad, self.roll_rad = IMUInterface.calculate_pitch_roll(imu.ax, imu.ay, imu.az)

        # 2. Compute absolute magnetic heading with tilt compensation
        mag_yaw = self.calculate_tilt_compensated_mag_heading(
            imu.mx, imu.my, imu.mz, self.pitch_rad, self.roll_rad
        )

        current_time = imu.timestamp
        self.angular_velocity_z = math.radians(imu.gz)  # convert deg/s to rad/s

        if not self.initialized or self.last_timestamp is None:
            # Cold start: Align directly to magnetometer reading
            self.yaw_rad = mag_yaw
            self.last_timestamp = current_time
            self.initialized = True
            return self.yaw_rad, self.pitch_rad, self.roll_rad

        dt = current_time - self.last_timestamp
        self.last_timestamp = current_time

        # Safety clamp on dt (e.g. if serial paused or missed frames)
        if dt <= 0.0 or dt > 0.5:
            dt = 0.02  # fallback to nominal 50Hz

        # 3. Integrate Gyroscope Z rate for rapid orientation change
        gyro_yaw = self.yaw_rad + (self.angular_velocity_z * dt)
        gyro_yaw = normalize_angle_rad(gyro_yaw)

        # 4. Complementary filter fusion with phase-wrap correction
        # Calculate smallest angular difference between mag_yaw and gyro_yaw
        diff = normalize_angle_rad(mag_yaw - gyro_yaw)

        # Fuse: yaw = gyro_yaw + (1 - alpha) * diff
        self.yaw_rad = normalize_angle_rad(gyro_yaw + (1.0 - self.alpha) * diff)

        return self.yaw_rad, self.pitch_rad, self.roll_rad

    @property
    def heading_deg(self) -> float:
        """Returns heading in degrees [0, 360)."""
        return normalize_angle_deg(math.degrees(self.yaw_rad))

    @property
    def heading_signed_deg(self) -> float:
        """Returns heading in degrees [-180, +180]."""
        return math.degrees(self.yaw_rad)
