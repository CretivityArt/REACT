"""
==============================================================================
REACT (Reactive Hazard Exploration/Response Rover)
Coordinate Transformation Module: coordinate_transform.py
==============================================================================
Implements 2D rigid-body transformations (Special Euclidean Group SE(2)):
1. Rover Local Coordinate Frame:
   +X = Forward along vehicle longitudinal axis
   +Y = Left along vehicle lateral axis
   Heading angle theta = 0 rad aligned with Global East (+X_global)
   Positive theta rotates counter-clockwise toward Global North (+Y_global)

2. Sensor-to-Rover Frame:
   Accounts for physical transducer mounting offsets (offset_x, offset_y)
   and beam mounting orientation angle (alpha).

3. Rover-to-Global Map Frame:
   Transforms local obstacle detections and rover chassis vertices into
   fixed world coordinates:
   [X_global]   [cos(theta)  -sin(theta)] [x_rover]   [X_rover]
   [Y_global] = [sin(theta)   cos(theta)] [y_rover] + [Y_rover]
"""

import math
from typing import Tuple, List
import numpy as np


class CoordinateTransformer:
    """
    Handles forward and inverse 2D kinematic coordinate projections.
    """

    @staticmethod
    def sensor_to_rover_frame(distance: float,
                              sensor_angle_deg: float,
                              offset_x: float,
                              offset_y: float) -> Tuple[float, float]:
        """
        Converts an ultrasonic distance reading into rover-centric coordinates (x_rover, y_rover).

        :param distance: Measured obstacle range in meters.
        :param sensor_angle_deg: Angle of the sensor ray relative to +X_rover (deg).
        :param offset_x: Sensor mount distance forward of rover center (meters).
        :param offset_y: Sensor mount distance left of rover center (meters).
        :return: (x_rover, y_rover) in meters.
        """
        angle_rad = math.radians(sensor_angle_deg)
        # Vector from sensor face to obstacle in sensor frame
        dx = distance * math.cos(angle_rad)
        dy = distance * math.sin(angle_rad)

        # Translated by physical transducer mount position on rover chassis
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
        Essential for drawing ray origin lines from the exact rover corner.
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

        Mathematics:
            X_global = X_rover + x_rover * cos(theta) - y_rover * sin(theta)
            Y_global = Y_rover + x_rover * sin(theta) + y_rover * cos(theta)

        :param x_rover: Point forward coordinate in meters.
        :param y_rover: Point lateral coordinate in meters.
        :param rover_x: Global X position of rover origin in meters.
        :param rover_y: Global Y position of rover origin in meters.
        :param rover_heading_rad: Rover orientation in radians (counter-clockwise from +X_global).
        :return: (X_global, Y_global) in meters.
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
        Transforms a list of local 2D vertices (e.g. rover chassis box or triangle)
        into global world coordinates.
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
        """
        Converts continuous global metric coordinates into discrete 2D grid matrix indices (row, col).
        """
        col = int(math.floor((x_global - map_origin_x) / resolution))
        row = int(math.floor((y_global - map_origin_y) / resolution))
        return col, row

    @staticmethod
    def grid_cell_to_global(col: int,
                            row: int,
                            map_origin_x: float,
                            map_origin_y: float,
                            resolution: float) -> Tuple[float, float]:
        """
        Converts discrete grid matrix coordinates back to continuous metric cell center coordinates.
        """
        x_global = map_origin_x + (col + 0.5) * resolution
        y_global = map_origin_y + (row + 0.5) * resolution
        return x_global, y_global
