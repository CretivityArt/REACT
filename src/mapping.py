"""
mapping.py
===========
Owns the live 2D map data structure: rover trajectory, estimated tunnel
boundary points, and all placed map_objects (people, hazards, other
detections), all expressed in the GLOBAL map coordinate frame (meters).

REAL VIDEO INFORMATION vs SIMULATED SENSOR INFORMATION
--------------------------------------------------------
- Trajectory points come from visual_odometry.py (real feature tracking
  + a simulated scale factor -- see that module's docstring).
- Tunnel boundary points come from simulated_sensors.py's ultrasonic
  simulation (partly real edge-density signal, partly simulated).
- Person / object positions come from object_detection.py's
  distance/bearing estimate transformed into global coordinates.
- Hazard positions come from hazard_simulation.py's predefined zones
  (fully simulated, not sensed).
"""

import math
from dataclasses import dataclass, field

from coordinate_transform import Pose, local_to_global


@dataclass
class MapObject:
    obj_type: str          # "person", "low_oxygen", "methane", "fire", "co", "obstacle", etc.
    x: float
    y: float
    label: str = ""         # e.g. "P1"
    confidence: float = 1.0
    frame_index: int = -1


class MineMap:
    def __init__(self):
        self.trajectory = []       # list of (x, y) rover positions
        self.boundary_points = []  # list of (x, y, side) tunnel wall estimates
        self.map_objects = []      # list of MapObject
        self._person_ids = {}      # simple track-id -> label memory
        self._next_person_id = 1

    # -- trajectory -----------------------------------------------------------
    def add_trajectory_point(self, pose: Pose):
        self.trajectory.append((pose.x, pose.y))

    # -- tunnel walls -----------------------------------------------------------
    def add_boundary_estimate(self, pose: Pose, left_dist: float, right_dist: float):
        """
        Given the rover's pose and simulated left/right ultrasonic
        readings, place approximate wall points to the left and right
        of the rover on the global map.
        """
        lx, ly = local_to_global(pose, forward=0.0, lateral=left_dist)
        rx, ry = local_to_global(pose, forward=0.0, lateral=-right_dist)
        self.boundary_points.append((lx, ly, "left"))
        self.boundary_points.append((rx, ry, "right"))

    # -- objects / hazards -----------------------------------------------------
    def add_person(self, pose: Pose, forward: float, lateral: float,
                    confidence: float, frame_index: int, track_key=None):
        gx, gy = local_to_global(pose, forward, lateral)

        # Very simple re-identification: if a similarly-positioned person
        # already exists nearby (within 2m), reuse its label instead of
        # minting a new one every frame. This is a heuristic, not real
        # multi-object tracking (e.g. no Kalman filter / re-ID embedding).
        for obj in self.map_objects:
            if obj.obj_type == "person":
                if math.hypot(obj.x - gx, obj.y - gy) < 2.0:
                    obj.x, obj.y = gx, gy
                    obj.confidence = confidence
                    obj.frame_index = frame_index
                    return obj

        label = f"P{self._next_person_id}"
        self._next_person_id += 1
        obj = MapObject("person", gx, gy, label=label,
                         confidence=confidence, frame_index=frame_index)
        self.map_objects.append(obj)
        return obj

    def add_detection(self, obj_type: str, pose: Pose, forward: float, lateral: float,
                       confidence: float, frame_index: int, label: str = ""):
        gx, gy = local_to_global(pose, forward, lateral)
        obj = MapObject(obj_type, gx, gy, label=label,
                         confidence=confidence, frame_index=frame_index)
        self.map_objects.append(obj)
        return obj

    def set_hazard_zones(self, zone_records):
        """Register static (simulated) hazard zones as map objects once."""
        for z in zone_records:
            self.map_objects.append(MapObject(
                obj_type=z["type"], x=z["x"], y=z["y"], label=z["id"],
                confidence=1.0, frame_index=-1,
            ))

    # -- summaries -----------------------------------------------------------
    def counts(self):
        people = sum(1 for o in self.map_objects if o.obj_type == "person")
        fire = sum(1 for o in self.map_objects if o.obj_type in ("fire", "fire_placeholder"))
        hazard_zones = sum(1 for o in self.map_objects
                            if o.obj_type in ("low_oxygen", "methane", "fire", "co"))
        return {"people": people, "fire": fire, "hazard_zones": hazard_zones}

    def as_object_dicts(self):
        return [
            {"type": o.obj_type, "x": round(o.x, 2), "y": round(o.y, 2),
             "label": o.label, "confidence": round(o.confidence, 2),
             "frame": o.frame_index}
            for o in self.map_objects
        ]
