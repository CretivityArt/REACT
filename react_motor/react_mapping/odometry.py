"""
==============================================================================
REACT (Reactive Hazard Exploration/Response Rover)
Dead-Reckoning & Odometry Engine: odometry.py
==============================================================================
Position estimation architecture:

IMPORTANT ENGINEERING PRINCIPLE - WHY NOT ACCELEROMETER INTEGRATION:
In low-cost robotics (especially underground without GPS), integrating
accelerometer data twice (a -> v -> x) to calculate position causes catastrophic
cubic error growth:
    Error(t) = 0.5 * (bias_accel) * t^2
An accelerometer offset of merely 0.05 m/s^2 (0.005 g) drifts by over 90 meters
in just 60 seconds! Furthermore, gravity leakage during minor surface bumps
corrupts the horizontal acceleration signal.

Therefore, this module implements:
1. Differential-drive kinematic odometry based on wheel encoder displacement
   or calibrated motor velocity estimation.
2. Fused orientation from sensor_fusion (IMU + Compass) rather than pure wheel
   encoder steering (which suffers from wheel slippage in coal dust/mud).
3. Extensible hooks for future rotary encoders (left_encoder, right_encoder).
"""

import math
from typing import Tuple, List, Optional
from config import WHEEL_BASE, WHEEL_RADIUS


class OdometryEstimator:
    """
    Tracks the rover's 2D Cartesian position (x, y) and trajectory path.
    """

    def __init__(self,
                 wheel_base: float = WHEEL_BASE,
                 wheel_radius: float = WHEEL_RADIUS):
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius

        # Global position (meters)
        self.x: float = 0.0
        self.y: float = 0.0
        self.heading_rad: float = 0.0

        # Motion metrics
        self.linear_velocity: float = 0.0     # m/s
        self.angular_velocity: float = 0.0    # rad/s
        self.total_distance_traveled: float = 0.0  # meters

        # Trajectory history for visualization
        self.path_x: List[float] = [0.0]
        self.path_y: List[float] = [0.0]
        self.min_path_dist_threshold = 0.03   # Record point every 3 cm of travel

        # Encoder state tracking
        self.last_left_ticks: Optional[int] = None
        self.last_right_ticks: Optional[int] = None
        self.ticks_per_revolution = 360       # Configurable for optical/hall encoders

    def set_origin(self, x: float = 0.0, y: float = 0.0, heading_rad: float = 0.0):
        """Re-zeros or resets the rover origin in the global map coordinate frame."""
        self.x = x
        self.y = y
        self.heading_rad = heading_rad
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.total_distance_traveled = 0.0
        self.path_x = [x]
        self.path_y = [y]

    def clear_path(self):
        """Clears the breadcrumb trajectory trail while maintaining current position."""
        self.path_x = [self.x]
        self.path_y = [self.y]

    def update_from_velocity(self,
                             linear_v: float,
                             heading_rad: float,
                             dt: float) -> Tuple[float, float]:
        """
        Updates position when using speed estimation / motor command feedforward
        (used when wheel encoders are not yet mounted or in simulation mode).

        Mathematics:
            dx = v * cos(heading) * dt
            dy = v * sin(heading) * dt
            X_new = X_prev + dx
            Y_new = Y_prev + dy
        """
        self.linear_velocity = linear_v
        self.heading_rad = heading_rad

        if dt <= 0.0 or dt > 1.0:
            dt = 0.05  # clamp

        step_dist = linear_v * dt
        self.x += step_dist * math.cos(heading_rad)
        self.y += step_dist * math.sin(heading_rad)
        self.total_distance_traveled += abs(step_dist)

        # Append to breadcrumb trail if rover moved sufficiently
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
        """
        Differential Drive Odometry with IMU-fused heading.
        When hardware wheel encoders are added, call this method.

        :param left_ticks: Cumulative pulse count from left motor encoder.
        :param right_ticks: Cumulative pulse count from right motor encoder.
        :param fused_heading_rad: Drift-free yaw from sensor_fusion module.
        :param dt: Time interval in seconds.
        """
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

        # Arc center displacement
        d_center = (d_right + d_left) / 2.0
        self.linear_velocity = d_center / dt if dt > 0 else 0.0

        # Use IMU-fused heading for position update (more accurate than pure encoder heading)
        avg_heading = (self.heading_rad + fused_heading_rad) / 2.0
        self.heading_rad = fused_heading_rad

        self.x += d_center * math.cos(avg_heading)
        self.y += d_center * math.sin(avg_heading)
        self.total_distance_traveled += abs(d_center)

        # Append to path
        dx = self.x - self.path_x[-1]
        dy = self.y - self.path_y[-1]
        if math.hypot(dx, dy) >= self.min_path_dist_threshold:
            self.path_x.append(self.x)
            self.path_y.append(self.y)

        return self.x, self.y

    def get_pose(self) -> Tuple[float, float, float]:
        """Returns (x, y, heading_rad)."""
        return self.x, self.y, self.heading_rad
