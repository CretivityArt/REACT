"""
simulated_sensors.py
======================
Software simulation of an ultrasonic distance sensor array (front /
left / right), driven by rough visual geometry cues extracted from
each frame -- NOT real acoustic time-of-flight hardware.

SIMULATED SENSOR INFORMATION
------------------------------
Every number this module returns is an *approximation* built from:
  1. Edge/contour density in image regions (a crude proxy for "how
     close is something" -- more/stronger nearby edges within a region
     generally correlates with a nearer surface filling more of the
     frame), combined with
  2. A configurable baseline tunnel half-width (config.DEFAULT_TUNNEL_HALF_WIDTH)
     and the simulated forward scale (config.METERS_PER_FRAME), so that
     numbers stay in a plausible mine-tunnel range even when the visual
     signal is weak or ambiguous.

This is explicitly a *simulation*, not a physically grounded distance
measurement. Real deployment would replace this module with actual
ultrasonic/LiDAR hardware readings.
"""

import cv2
import numpy as np

import config


class SimulatedUltrasonicArray:
    def __init__(self):
        self.max_range = config.ULTRASONIC_MAX_RANGE
        self.baseline_half_width = config.DEFAULT_TUNNEL_HALF_WIDTH

    def _region_edge_density(self, edges: np.ndarray, x0, x1, y0, y1):
        region = edges[y0:y1, x0:x1]
        if region.size == 0:
            return 0.0
        return float(np.count_nonzero(region)) / float(region.size)

    def estimate(self, gray_frame: np.ndarray):
        """
        Returns a dict: {"front": meters, "left": meters, "right": meters}

        Method (simulation heuristic):
          - Run Canny edge detection on the frame.
          - Split the frame into left / center(front) / right vertical
            thirds.
          - Higher edge density near the bottom of a region (closer to
            camera / ground) is treated as "closer obstacle" -> shorter
            simulated distance. Lower density -> open space -> longer
            simulated distance, capped at max_range.
          - Values are smoothed toward the configured baseline so a
            single noisy frame does not produce wild spikes.
        """
        h, w = gray_frame.shape[:2]
        edges = cv2.Canny(gray_frame, 60, 150)

        third = w // 3
        regions = {
            "left": (0, third, h // 2, h),
            "front": (third, 2 * third, h // 2, h),
            "right": (2 * third, w, h // 2, h),
        }

        readings = {}
        for name, (x0, x1, y0, y1) in regions.items():
            density = self._region_edge_density(edges, x0, x1, y0, y1)
            # density in [0, 1]; invert & scale into a plausible range.
            # High density (busy, textured, close wall) -> short distance.
            # Low density (open/flat/dark tunnel) -> long distance.
            simulated = self.max_range * (1.0 - min(density * 4.0, 1.0))
            # Blend with baseline assumption to avoid unrealistic 0.0 / max jumps.
            baseline = self.baseline_half_width if name != "front" else self.max_range * 0.5
            blended = 0.6 * simulated + 0.4 * baseline
            readings[name] = round(float(np.clip(blended, 0.15, self.max_range)), 2)

        return readings


def format_sensor_block(readings: dict) -> str:
    return (
        f"Front: {readings.get('front', 0):.2f} m | "
        f"Left: {readings.get('left', 0):.2f} m | "
        f"Right: {readings.get('right', 0):.2f} m"
    )
