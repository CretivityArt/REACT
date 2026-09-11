"""
==============================================================================
REACT (Reactive Hazard Exploration/Response Rover)
Sensor Filtering Module: filters.py
==============================================================================
Provides robust digital signal processing filters:
1. Rolling Median Filter: Rejects impulse noise and specular ultrasonic reflections.
2. Exponential Moving Average (EMA) Filter: Smooths sensor measurements.
3. Outlier Gating: Discards physically impossible delta jumps.
"""

from collections import deque
from typing import Optional, List
import numpy as np


class MedianFilter:
    """
    Rolling window median filter.
    Highly effective against ultrasonic multi-path echoes and false early returns.
    """
    def __init__(self, window_size: int = 5):
        if window_size % 2 == 0:
            window_size += 1  # Window size should be odd for an exact median
        self.window_size = window_size
        self.buffer = deque(maxlen=window_size)

    def update(self, value: float) -> float:
        """Add new value and return filtered median."""
        self.buffer.append(value)
        sorted_vals = sorted(self.buffer)
        mid = len(sorted_vals) // 2
        return sorted_vals[mid]

    def reset(self):
        self.buffer.clear()


class MovingAverageFilter:
    """
    Exponential Moving Average (EMA) or Simple Moving Average (SMA).
    """
    def __init__(self, alpha: float = 0.25):
        """
        :param alpha: Smoothing factor between 0.0 and 1.0.
                      Higher alpha = faster response, less smoothing.
        """
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
    """
    Gating filter to reject readings that exceed physical rate of change limits.
    e.g., A rover cannot instantly move 3 meters in 50 milliseconds.
    """
    def __init__(self, max_delta: float = 1.5):
        """
        :param max_delta: Maximum permitted absolute change in one time step (meters).
        """
        self.max_delta = max_delta
        self.last_valid: Optional[float] = None
        self.consecutive_rejects = 0
        self.max_consecutive = 4  # Accept new value if persistently rejected

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
            # If the environment truly changed (e.g. rover turned a corner),
            # don't latch onto the old value indefinitely
            if self.consecutive_rejects >= self.max_consecutive:
                self.last_valid = value
                self.consecutive_rejects = 0
                return value
            return self.last_valid  # Hold last good value
