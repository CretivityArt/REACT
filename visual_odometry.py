"""
visual_odometry.py
====================
A simplified monocular visual-odometry (VO) module.

REAL VIDEO INFORMATION vs SIMULATED SENSOR INFORMATION
--------------------------------------------------------
- REAL (derived from the actual video pixels): ORB keypoints, feature
  matches, and the *direction* of camera rotation/translation recovered
  from the Essential Matrix.
- SIMULATED / ASSUMED: the metric SCALE of translation. A single camera
  cannot recover true distance from an Essential Matrix decomposition --
  this is the well-known monocular scale ambiguity. We resolve it here
  by multiplying the unit translation vector by config.METERS_PER_FRAME,
  which is an explicit, configurable simulation constant, NOT a measured
  quantity.

Pipeline per frame pair:
  1. Detect ORB keypoints + descriptors in both frames.
  2. Match descriptors (BFMatcher + Lowe's ratio test).
  3. If enough good matches, estimate the Essential Matrix with RANSAC.
  4. Recover relative rotation R and translation direction t (unit vector)
     via cv2.recoverPose.
  5. Convert (R, t) into a 2D planar update (dx, dy, dtheta) applied to
     the rover's running pose, using the SIMULATED scale factor for
     translation magnitude.

Limitations (explicitly stated per the project's honesty requirement):
  - This is a *2D-projected* simplification of what is really 3D camera
    motion. We assume the rover moves roughly on a flat plane, which is
    reasonable for a wheeled mine rover but not exact.
  - No loop closure, no bundle adjustment, no keyframe management --
    small errors accumulate ("drift") over the length of a video, which
    is normal for a minimal VO pipeline like this one.
  - If the video contains rapid rotation, textureless walls (blank rock),
    or motion blur, feature matching quality drops and pose updates are
    held constant (skipped) for that frame rather than guessed.
"""

import math

import cv2
import numpy as np

import config
from coordinate_transform import Pose


class VisualOdometry:
    def __init__(self, camera_matrix: np.ndarray = None):
        self.orb = cv2.ORB_create(nfeatures=config.ORB_N_FEATURES)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

        self.prev_gray = None
        self.prev_kp = None
        self.prev_des = None

        self.pose = Pose(0.0, 0.0, 0.0)

        # Default intrinsic matrix guess if none supplied. This is a
        # rough approximation (no real calibration performed) -- see
        # config.ASSUMED_FOCAL_PX for the honesty note on focal length.
        if camera_matrix is None:
            f = config.ASSUMED_FOCAL_PX
            w = config.PROCESS_WIDTH
            h = int(w * 9 / 16)
            self.camera_matrix = np.array([
                [f, 0, w / 2],
                [0, f, h / 2],
                [0, 0, 1],
            ], dtype=np.float64)
        else:
            self.camera_matrix = camera_matrix

        self.last_match_count = 0
        self.last_status = "init"

    def process_frame(self, gray_frame: np.ndarray):
        """
        Feed the next grayscale frame. Updates self.pose in place.
        Returns (pose, status_string, num_matches, kp, matches_for_draw).
        """
        kp, des = self.orb.detectAndCompute(gray_frame, None)

        if self.prev_gray is None or des is None or self.prev_des is None:
            self.prev_gray, self.prev_kp, self.prev_des = gray_frame, kp, des
            self.last_status = "init"
            return self.pose, self.last_status, 0, kp, []

        if des is None or self.prev_des is None or len(des) < 2 or len(self.prev_des) < 2:
            self.prev_gray, self.prev_kp, self.prev_des = gray_frame, kp, des
            self.last_status = "insufficient_features"
            return self.pose, self.last_status, 0, kp, []

        # KNN match + Lowe's ratio test
        try:
            raw_matches = self.matcher.knnMatch(self.prev_des, des, k=2)
        except cv2.error:
            self.prev_gray, self.prev_kp, self.prev_des = gray_frame, kp, des
            self.last_status = "match_error"
            return self.pose, self.last_status, 0, kp, []

        good_matches = []
        for pair in raw_matches:
            if len(pair) != 2:
                continue
            m, n = pair
            if m.distance < 0.75 * n.distance:
                good_matches.append(m)

        self.last_match_count = len(good_matches)

        if len(good_matches) < config.MIN_MATCH_COUNT:
            # Not enough reliable motion signal -- hold previous pose.
            self.prev_gray, self.prev_kp, self.prev_des = gray_frame, kp, des
            self.last_status = "low_texture_hold_pose"
            return self.pose, self.last_status, len(good_matches), kp, good_matches

        pts_prev = np.float32([self.prev_kp[m.queryIdx].pt for m in good_matches])
        pts_curr = np.float32([kp[m.trainIdx].pt for m in good_matches])

        E, mask = cv2.findEssentialMat(
            pts_curr, pts_prev, self.camera_matrix,
            method=cv2.RANSAC, prob=0.999, threshold=1.0,
        )

        if E is None or E.shape != (3, 3):
            self.prev_gray, self.prev_kp, self.prev_des = gray_frame, kp, des
            self.last_status = "essential_matrix_failed"
            return self.pose, self.last_status, len(good_matches), kp, good_matches

        _, R, t, mask_pose = cv2.recoverPose(E, pts_curr, pts_prev, self.camera_matrix)

        # Project the 3D translation/rotation into a 2D planar update.
        # Camera/world convention here: x = right, y = down, z = forward.
        # We treat (z, x) as the ground-plane (forward, lateral) axes.
        forward_unit = float(t[2, 0])   # unit-scale forward component
        lateral_unit = float(t[0, 0])   # unit-scale lateral component

        # Yaw (rotation about the vertical/y axis) from the rotation matrix.
        # theta convention: positive = turning left (counter-clockwise).
        dtheta = -float(np.arctan2(R[0, 2], R[2, 2]))

        # Apply SIMULATED metric scale (see module docstring).
        # cv2.recoverPose already returns t as a UNIT vector (true scale is
        # lost -- that's the scale ambiguity). We take the ground-plane
        # (forward, lateral) sub-vector of that unit vector, re-normalize
        # IT to unit length (so a pure-sideways or pure-forward motion both
        # produce a full-magnitude step rather than a divide-by-near-zero
        # blow-up), and then scale by the configured simulated step size.
        planar_norm = math.hypot(forward_unit, lateral_unit)
        if planar_norm > 1e-6:
            forward_m = (forward_unit / planar_norm) * config.METERS_PER_FRAME
            lateral_m = (lateral_unit / planar_norm) * config.METERS_PER_FRAME
        else:
            forward_m = 0.0
            lateral_m = 0.0

        # Update global pose using the rover's current heading.
        cos_t = np.cos(self.pose.theta)
        sin_t = np.sin(self.pose.theta)
        dx = forward_m * cos_t - lateral_m * sin_t
        dy = forward_m * sin_t + lateral_m * cos_t

        self.pose.x += dx
        self.pose.y += dy
        self.pose.theta += dtheta

        self.prev_gray, self.prev_kp, self.prev_des = gray_frame, kp, des
        self.last_status = "ok"
        return self.pose, self.last_status, len(good_matches), kp, good_matches
