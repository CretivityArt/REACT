"""
coordinate_transform.py
========================
Helpers for converting between the rover's LOCAL (camera/body) frame and
the GLOBAL (map / world) frame.

Coordinate convention (matches the prompt's diagram):

              +Y
               |
               |
               |
               +---------> +X
             START (0,0)

- rover.x, rover.y  : rover position in the global map frame (meters)
- rover.theta       : rover heading in radians, measured counter-clockwise
                       from +X, i.e. theta=0 means facing along +X.
- A LOCAL point (forward, lateral) is expressed relative to the rover:
    forward = distance straight ahead of the rover (its facing direction)
    lateral = distance to the LEFT of the rover (positive = left)

HONESTY NOTE: these are pure 2D rigid-body transforms (rotation +
translation). They are exact *given* correct (x, y, theta) and correct
local (forward, lateral) estimates. The error in this system comes
entirely from upstream estimates (visual odometry scale/drift, and
monocular distance approximation for detected objects) -- not from the
transform math itself.
"""

import math
from dataclasses import dataclass


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # radians


def local_to_global(pose: Pose, forward: float, lateral: float):
    """
    Convert a point expressed relative to the rover (forward/lateral,
    meters) into global map (x, y) coordinates, given the rover's pose.

    forward: +distance ahead of the rover along its heading
    lateral: +distance to the LEFT of the rover
    """
    # Rotate the local (forward, lateral) vector by the rover heading.
    # Local "forward" axis aligns with heading theta.
    # Local "lateral" (left) axis is heading + 90 degrees.
    cos_t = math.cos(pose.theta)
    sin_t = math.sin(pose.theta)

    dx = forward * cos_t - lateral * sin_t
    dy = forward * sin_t + lateral * cos_t

    global_x = pose.x + dx
    global_y = pose.y + dy
    return global_x, global_y


def global_to_local(pose: Pose, global_x: float, global_y: float):
    """
    Convert a global map point into the rover's local (forward, lateral)
    frame, given the rover's pose. Inverse of local_to_global.
    """
    dx = global_x - pose.x
    dy = global_y - pose.y

    cos_t = math.cos(pose.theta)
    sin_t = math.sin(pose.theta)

    # Inverse rotation
    forward = dx * cos_t + dy * sin_t
    lateral = -dx * sin_t + dy * cos_t
    return forward, lateral


def bearing_distance_to_local(bearing_rad: float, distance: float):
    """
    Convert a (bearing, distance) pair -- e.g. from a detected object's
    horizontal offset in the camera frame plus an estimated distance --
    into local (forward, lateral) coordinates.

    bearing_rad: angle from the rover's forward axis, positive = left,
                 negative = right (radians). Small-angle camera bearings
                 typically come from horizontal pixel offset / focal length.
    distance: straight-line distance estimate (meters).
    """
    forward = distance * math.cos(bearing_rad)
    lateral = distance * math.sin(bearing_rad)
    return forward, lateral


def relative_direction_label(forward: float, lateral: float) -> str:
    """
    Turn a local (forward, lateral) offset into a human-readable label
    like 'front-right', used for dashboard/console text.
    """
    if forward >= 0:
        fb = "front"
    else:
        fb = "rear"

    if abs(lateral) < 0.3:
        lr = ""
    elif lateral > 0:
        lr = "-left"
    else:
        lr = "-right"

    return f"{fb}{lr}"
