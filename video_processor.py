"""
video_processor.py
====================
Orchestrates the per-frame pipeline described in the project spec:

    Read frame
         |
    Detect features
         |
    Estimate rover motion         (visual_odometry.py)
         |
    Update rover position
         |
    Detect objects                (object_detection.py)
         |
    Estimate object positions     (coordinate_transform.py)
         |
    Update simulated sensors      (simulated_sensors.py)
         |
    Update hazard map             (hazard_simulation.py)
         |
    Render video + 2D map         (visualization.py)

Also responsible for writing the CSV logs and the final map PNG.
"""

import csv
import math
import os
import time

import cv2

import config
from visual_odometry import VisualOdometry
from simulated_sensors import SimulatedUltrasonicArray, format_sensor_block
from object_detection import ObjectDetector
from hazard_simulation import HazardSimulator
from mapping import MineMap
from coordinate_transform import Pose
import visualization as viz


class VideoProcessor:
    def __init__(self, video_path=None):
        self.video_path = video_path or config.VIDEO_PATH
        self.vo = VisualOdometry()
        self.ultrasonic = SimulatedUltrasonicArray()
        self.detector = ObjectDetector()
        self.hazards = HazardSimulator()
        self.mine_map = MineMap()
        self.map_canvas = viz.MapCanvas()

        self.mine_map.set_hazard_zones(self.hazards.zone_hazard_records())

        os.makedirs(config.OUTPUT_DIR, exist_ok=True)
        self._trajectory_rows = []
        self._detection_rows = []
        self._hazard_rows = []

        self.frame_index = 0

    # ------------------------------------------------------------------
    def _log_trajectory(self, pose: Pose, matches, vo_status):
        self._trajectory_rows.append({
            "frame": self.frame_index, "x": round(pose.x, 3), "y": round(pose.y, 3),
            "heading_deg": round(math.degrees(pose.theta), 2),
            "matches": matches, "vo_status": vo_status,
        })

    def _log_detections(self, detections):
        for d in detections:
            self._detection_rows.append({
                "frame": self.frame_index, "label": d.label,
                "confidence": round(d.confidence, 3),
                "distance_m": round(d.distance_m, 2),
                "direction": d.direction_label,
                "placeholder": d.is_placeholder,
            })

    def _log_hazards(self, env, warnings):
        for w in warnings:
            row = dict(w)
            row["frame"] = self.frame_index
            row.update({k: env[k] for k in ("oxygen", "methane", "co", "co2", "temperature")})
            self._hazard_rows.append(row)

    # ------------------------------------------------------------------
    def run(self):
        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            raise FileNotFoundError(
                f"Could not open video at '{self.video_path}'. "
                f"Place your tunnel video there or update config.VIDEO_PATH."
            )

        print(f"[video_processor] Processing: {self.video_path}")
        print(f"[video_processor] METERS_PER_FRAME (simulated scale) = {config.METERS_PER_FRAME}")

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            self.frame_index += 1
            if config.FRAME_STRIDE > 1 and (self.frame_index % config.FRAME_STRIDE != 0):
                continue

            # Resize for consistent processing speed / intrinsics assumption.
            h0, w0 = frame.shape[:2]
            scale = config.PROCESS_WIDTH / w0
            frame = cv2.resize(frame, (config.PROCESS_WIDTH, int(h0 * scale)))
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 1) Visual odometry -> rover motion / position (REAL + SIMULATED SCALE)
            pose, vo_status, matches, kp, good_matches = self.vo.process_frame(gray)
            self.mine_map.add_trajectory_point(pose)
            self._log_trajectory(pose, matches, vo_status)

            # 2) Simulated ultrasonic sensors (front/left/right)
            sensors = {"front": 0.0, "left": 0.0, "right": 0.0}
            if config.SIMULATE_ULTRASONIC:
                sensors = self.ultrasonic.estimate(gray)
                self.mine_map.add_boundary_estimate(pose, sensors["left"], sensors["right"])

            # 3) Object detection (YOLO + fire/smoke placeholder heuristic)
            detections = []
            if config.ENABLE_YOLO:
                detections = self.detector.detect(frame)
                self._log_detections(detections)
                for det in detections:
                    if det.label == "person":
                        self.mine_map.add_person(pose, det.forward, det.lateral,
                                                  det.confidence, self.frame_index)
                    else:
                        self.mine_map.add_detection(det.label, pose, det.forward, det.lateral,
                                                     det.confidence, self.frame_index, label=det.label)

            # 4) Simulated hazard environment for current position
            env, warnings = ({}, [])
            if config.SIMULATE_GAS:
                env, warnings = self.hazards.sample_environment(pose.x, pose.y)
                self._log_hazards(env, warnings)
            else:
                env = dict(config.BASELINE_ENVIRONMENT)

            counts = self.mine_map.counts()

            # 5) Render dashboard
            if config.SHOW_VIDEO or config.SHOW_MAP:
                annotated = viz.draw_detections_on_frame(frame, detections)
                map_img = self.map_canvas.render(self.mine_map, pose)
                status_panel = viz.build_status_panel(
                    width=annotated.shape[1] + map_img.shape[1], height=110,
                    rover_status={"x": pose.x, "y": pose.y,
                                  "heading_deg": math.degrees(pose.theta) % 360,
                                  "matches": matches},
                    sensors=sensors, environment=env, counts=counts,
                    warnings=warnings, vo_status=vo_status,
                )
                dashboard = viz.compose_dashboard(annotated, map_img, status_panel)
                cv2.imshow("REACT Mine Simulation Dashboard", dashboard)

                if config.SAVE_EVERY_N_FRAMES and self.frame_index % config.SAVE_EVERY_N_FRAMES == 0:
                    cv2.imwrite(os.path.join(config.OUTPUT_DIR, f"frame_{self.frame_index:05d}.png"), dashboard)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("[video_processor] Quit requested by user.")
                    break

        cap.release()
        cv2.destroyAllWindows()
        self._save_outputs()

    # ------------------------------------------------------------------
    def _save_outputs(self):
        viz.save_final_map_png(self.mine_map, config.FINAL_MAP_PATH)
        print(f"[video_processor] Saved final map -> {config.FINAL_MAP_PATH}")

        self._write_csv(config.TRAJECTORY_CSV, self._trajectory_rows,
                         ["frame", "x", "y", "heading_deg", "matches", "vo_status"])
        self._write_csv(config.DETECTIONS_CSV, self._detection_rows,
                         ["frame", "label", "confidence", "distance_m", "direction", "placeholder"])
        self._write_csv(config.HAZARDS_CSV, self._hazard_rows,
                         ["frame", "zone_id", "type", "level", "distance_to_center",
                          "oxygen", "methane", "co", "co2", "temperature"])

        print(f"[video_processor] Saved trajectory -> {config.TRAJECTORY_CSV}")
        print(f"[video_processor] Saved detections -> {config.DETECTIONS_CSV}")
        print(f"[video_processor] Saved hazards -> {config.HAZARDS_CSV}")

    @staticmethod
    def _write_csv(path, rows, fieldnames):
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow(row)
