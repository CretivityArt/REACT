"""
==============================================================================
REACT (Reactive Hazard Exploration/Response Rover)
2D Occupancy Grid Mapping Engine: mapping.py
==============================================================================
Implements a 2D robotic occupancy grid map:
1. Continuous-to-discrete spatial mapping with configurable resolution (e.g. 5 cm).
2. Bresenham's line algorithm for fast ray tracing through grid cells.
3. Bayesian log-odds occupancy probability updates:
   - Cells along the ultrasonic ray are carved as FREE space.
   - Cells at the detected distance endpoint are marked as OCCUPIED obstacles.
   - Cells beyond max sensor range or untouched remain UNKNOWN.
4. Independent point-cloud repository for vector obstacle rendering.
"""

import math
from typing import List, Tuple, Dict, Set
import numpy as np

from config import (
    MAP_RESOLUTION, MAP_WIDTH_METERS, MAP_HEIGHT_METERS,
    LOG_ODDS_FREE, LOG_ODDS_OCCUPIED, LOG_ODDS_MAX, LOG_ODDS_MIN,
    MIN_ULTRASONIC_DISTANCE, MAX_ULTRASONIC_DISTANCE
)
from coordinate_transform import CoordinateTransformer


class OccupancyGridMap:
    """
    2D Log-Odds Occupancy Grid for underground tunnel/hazard mapping.
    """

    def __init__(self,
                 resolution: float = MAP_RESOLUTION,
                 width_m: float = MAP_WIDTH_METERS,
                 height_m: float = MAP_HEIGHT_METERS):
        self.resolution = resolution
        self.width_m = width_m
        self.height_m = height_m

        # Number of grid cells in X (columns) and Y (rows)
        self.cols = int(round(width_m / resolution))
        self.rows = int(round(height_m / resolution))

        # Origin is centered so (0, 0) meters is at the center of the grid matrix
        self.origin_x = -width_m / 2.0
        self.origin_y = -height_m / 2.0

        # Log-odds matrix: 0.0 corresponds to probability P(occ) = 0.5 (unknown)
        self.grid = np.zeros((self.rows, self.cols), dtype=np.float32)

        # Persistent obstacle coordinate list for fast point rendering
        self.obstacle_points: List[Tuple[float, float]] = []
        # Spatial set to prevent duplicate points in the same cell
        self.occupied_cells: Set[Tuple[int, int]] = set()

    def reset(self):
        """Clears the occupancy grid and all recorded obstacles."""
        self.grid.fill(0.0)
        self.obstacle_points.clear()
        self.occupied_cells.clear()

    def world_to_grid(self, gx: float, gy: float) -> Tuple[int, int]:
        """Converts world metric coordinates (meters) to grid indices (col, row)."""
        col = int(math.floor((gx - self.origin_x) / self.resolution))
        row = int(math.floor((gy - self.origin_y) / self.resolution))
        return col, row

    def grid_to_world(self, col: int, row: int) -> Tuple[float, float]:
        """Converts grid cell indices to center coordinates in world frame (meters)."""
        gx = self.origin_x + (col + 0.5) * self.resolution
        gy = self.origin_y + (row + 0.5) * self.resolution
        return gx, gy

    def is_inside(self, col: int, row: int) -> bool:
        """Checks if grid index falls within the bounded map matrix."""
        return 0 <= col < self.cols and 0 <= row < self.rows

    @staticmethod
    def bresenham_line(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """
        Standard integer Bresenham algorithm to compute all grid cells
        traversed by a sensor ray from origin (x0, y0) to target (x1, y1).
        """
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
        """
        Integrates one ultrasonic beam into the occupancy grid:
        - Carves all intermediate cells along the beam as FREE.
        - Marks the target endpoint as OCCUPIED if valid_hit is True.
        """
        # Convert world coordinates to grid indices
        c0, r0 = self.world_to_grid(origin_gx, origin_gy)
        c1, r1 = self.world_to_grid(target_gx, target_gy)

        if not self.is_inside(c0, r0):
            return

        # Trace all cells traversed by the beam
        ray_cells = self.bresenham_line(c0, r0, c1, r1)

        # Free space carving: all cells along the ray except the final endpoint
        # Only carve if distance is valid (> min range)
        if distance >= MIN_ULTRASONIC_DISTANCE:
            for c, r in ray_cells[:-1]:
                if self.is_inside(c, r):
                    # Decrement log-odds (more likely to be free space)
                    self.grid[r, c] = max(LOG_ODDS_MIN, self.grid[r, c] + LOG_ODDS_FREE)

        # Obstacle endpoint update
        if valid_hit and MIN_ULTRASONIC_DISTANCE <= distance <= MAX_ULTRASONIC_DISTANCE:
            if self.is_inside(c1, r1):
                # Increment log-odds (more likely to be occupied obstacle)
                self.grid[r1, c1] = min(LOG_ODDS_MAX, self.grid[r1, c1] + LOG_ODDS_OCCUPIED)

                # Store in vector obstacle point cloud if not already recorded
                cell_key = (c1, r1)
                if cell_key not in self.occupied_cells:
                    self.occupied_cells.add(cell_key)
                    self.obstacle_points.append((target_gx, target_gy))

    def get_probability_map(self) -> np.ndarray:
        """
        Converts log-odds matrix to occupancy probabilities [0.0, 1.0].
        Formula: P(m) = 1.0 - (1.0 / (1.0 + exp(grid)))
        """
        # np.exp with safe numerical clamping
        clamped_grid = np.clip(self.grid, -10.0, 10.0)
        return 1.0 - (1.0 / (1.0 + np.exp(clamped_grid)))
