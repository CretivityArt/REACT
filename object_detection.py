"""
object_detection.py
=====================
Wraps Ultralytics YOLO for object detection, and estimates each
detection's approximate distance/bearing from the rover using simple
pinhole-camera geometry.

IMPORTANT HONESTY NOTES
-------------------------
1. Standard pretrained YOLO models (e.g. yolov8n.pt trained on COCO) do
   NOT include "fire" or "smoke" as classes. This module therefore:
     - Uses real YOLO detections for classes it *does* know (person,
       vehicle-like classes, bags, etc).
     - Additionally runs a clearly-labeled PLACEHOLDER heuristic
       (color/brightness based) that flags regions which *might* be
       fire/smoke, tagged as "fire_placeholder" / "smoke_placeholder"
       so they are never confused with a real, trained detection.
     - If you provide a custom-trained mine-specific model at
       config.CUSTOM_MODEL_PATH (with real fire/smoke/person/obstacle
       classes), that model is used instead and placeholder detection
       is automatically disabled.

2. Distance-from-bounding-box-height is a classic monocular estimation
   trick: distance ≈ (real_height * focal_px) / bbox_height_px. It is
   only as good as the assumed real height and assumed focal length
   (both configurable in config.py) -- it is NOT a calibrated or
   depth-sensor measurement. Treat all reported distances as rough
   estimates, appropriate for a prototype/simulation only.
"""

import os
import cv2
import numpy as np

import config
from coordinate_transform import bearing_distance_to_local, relative_direction_label

_YOLO_AVAILABLE = False
try:
    from ultralytics import YOLO
    _YOLO_AVAILABLE = True
except ImportError:
    _YOLO_AVAILABLE = False


class Detection:
    def __init__(self, label, confidence, bbox, distance_m, bearing_rad,
                 forward, lateral, is_placeholder=False):
        self.label = label
        self.confidence = confidence
        self.bbox = bbox  # (x1, y1, x2, y2) in pixel coords
        self.distance_m = distance_m
        self.bearing_rad = bearing_rad
        self.forward = forward
        self.lateral = lateral
        self.is_placeholder = is_placeholder

    @property
    def direction_label(self):
        return relative_direction_label(self.forward, self.lateral)

    def __repr__(self):
        tag = " (PLACEHOLDER)" if self.is_placeholder else ""
        return (f"<{self.label}{tag} conf={self.confidence:.2f} "
                f"dist={self.distance_m:.1f}m dir={self.direction_label}>")


class ObjectDetector:
    def __init__(self):
        self.model = None
        self.using_custom_model = False
        self.enabled = config.ENABLE_YOLO and _YOLO_AVAILABLE

        if not _YOLO_AVAILABLE and config.ENABLE_YOLO:
            print("[object_detection] WARNING: ultralytics not installed. "
                  "YOLO detection disabled; only placeholder fire/smoke "
                  "heuristics will run. Install with: pip install ultralytics")

        if self.enabled:
            model_path = config.YOLO_MODEL_PATH
            if os.path.exists(config.CUSTOM_MODEL_PATH):
                model_path = config.CUSTOM_MODEL_PATH
                self.using_custom_model = True
                print(f"[object_detection] Using custom mine model: {model_path}")
            else:
                print(f"[object_detection] Using standard YOLO model: {model_path}")
                print("[object_detection] NOTE: standard model has no fire/smoke "
                      "classes -- those will come from the placeholder heuristic.")
            try:
                self.model = YOLO(model_path)
            except Exception as e:
                print(f"[object_detection] Failed to load YOLO model ({e}). "
                      "Falling back to placeholder-only detection.")
                self.model = None
                self.enabled = False

        self.focal_px = config.ASSUMED_FOCAL_PX
        self.person_height_m = config.ASSUMED_PERSON_HEIGHT_M

    # -- distance / bearing estimation -------------------------------------
    def _estimate_distance(self, label, bbox_height_px, frame_height_px):
        if bbox_height_px <= 0:
            return float("inf")
        if label == "person":
            real_h = self.person_height_m
        else:
            # Generic fallback assumed object height for non-person classes.
            real_h = 1.0
        distance = (real_h * self.focal_px) / bbox_height_px
        return float(np.clip(distance, 0.1, 100.0))

    def _estimate_bearing(self, bbox_center_x, frame_width_px):
        # Horizontal pixel offset from image center -> small-angle bearing.
        offset = (frame_width_px / 2.0) - bbox_center_x  # +left, -right
        bearing = np.arctan2(offset, self.focal_px)
        return float(bearing)

    # -- placeholder fire/smoke heuristic ------------------------------------
    def _placeholder_fire_smoke(self, frame_bgr):
        """
        Very crude color-based heuristic ONLY used when no trained
        fire/smoke model is available. Flags bright orange/red regions
        as possible "fire_placeholder" and gray/hazy low-saturation
        bright regions as possible "smoke_placeholder".

        This is explicitly NOT a trained detector and will produce
        false positives/negatives. It exists purely so the pipeline has
        a slot to visualize "what fire/smoke detection would plug into"
        until a real mine-specific model is trained and supplied at
        config.CUSTOM_MODEL_PATH.
        """
        detections = []
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        h, w = frame_bgr.shape[:2]

        # Fire-ish: high saturation, orange/red hue, high value.
        fire_mask = cv2.inRange(hsv, (0, 120, 150), (25, 255, 255))
        # Smoke-ish: low saturation, mid-high value (grayish/hazy).
        smoke_mask = cv2.inRange(hsv, (0, 0, 120), (180, 40, 220))

        for mask, label in ((fire_mask, "fire_placeholder"),
                             (smoke_mask, "smoke_placeholder")):
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area < (w * h * 0.002):  # ignore tiny specks / noise
                    continue
                x, y, bw, bh = cv2.boundingRect(c)
                distance = self._estimate_distance("generic", bh, h)
                bearing = self._estimate_bearing(x + bw / 2.0, w)
                forward, lateral = bearing_distance_to_local(bearing, distance)
                conf = float(np.clip(area / (w * h), 0.05, 0.6))
                detections.append(Detection(
                    label=label, confidence=conf, bbox=(x, y, x + bw, y + bh),
                    distance_m=distance, bearing_rad=bearing,
                    forward=forward, lateral=lateral, is_placeholder=True,
                ))
        return detections

    # -- main entry point -----------------------------------------------------
    def detect(self, frame_bgr):
        """
        Returns a list of Detection objects for this frame.
        """
        h, w = frame_bgr.shape[:2]
        results = []

        if self.enabled and self.model is not None:
            try:
                preds = self.model.predict(
                    frame_bgr, verbose=False,
                    conf=config.YOLO_CONFIDENCE_THRESHOLD,
                )
            except Exception as e:
                print(f"[object_detection] inference error: {e}")
                preds = []

            for r in preds:
                names = r.names
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    label = names.get(cls_id, str(cls_id)) if isinstance(names, dict) \
                        else names[cls_id]
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]

                    if not self.using_custom_model and label not in config.RELEVANT_COCO_CLASSES:
                        continue

                    bbox_h = y2 - y1
                    distance = self._estimate_distance(label, bbox_h, h)
                    bearing = self._estimate_bearing((x1 + x2) / 2.0, w)
                    forward, lateral = bearing_distance_to_local(bearing, distance)

                    results.append(Detection(
                        label=label, confidence=conf, bbox=(x1, y1, x2, y2),
                        distance_m=distance, bearing_rad=bearing,
                        forward=forward, lateral=lateral, is_placeholder=False,
                    ))

        # Only add color-heuristic fire/smoke if the loaded model is NOT a
        # custom model presumed to already know real fire/smoke classes.
        if not self.using_custom_model:
            results.extend(self._placeholder_fire_smoke(frame_bgr))

        return results
