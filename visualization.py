"""
visualization.py
==================
Builds the live dashboard: annotated video (left), 2D top-down map
(right), and a text sensor/status panel (bottom). Rendered every frame
with OpenCV for speed (fast enough to stay in sync with video playback);
a nicer static Matplotlib version of the final map is saved separately
at the end of the run (see save_final_map_png below).

Layout:
  +--------------------------+--------------------------+
  |                          |                          |
  |      TUNNEL VIDEO        |       2D MINE MAP        |
  |   (YOLO / placeholder    |  (rover, trajectory,     |
  |    bounding boxes)       |   walls, people, hazards)|
  |                          |                          |
  +--------------------------+--------------------------+
  |               SENSOR / STATUS TEXT PANEL             |
  +--------------------------------------------------------+
"""

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import config

# Color scheme (BGR for OpenCV)
COLOR_ROVER = (0, 165, 255)       # orange
COLOR_TRAJECTORY = (255, 200, 0)  # light blue
COLOR_WALL = (90, 90, 90)         # gray
COLOR_PERSON = (0, 255, 0)        # green
COLOR_FIRE = (0, 0, 255)          # red
COLOR_METHANE = (0, 200, 255)     # yellow-orange
COLOR_LOW_O2 = (255, 0, 255)      # magenta
COLOR_CO = (0, 100, 255)          # orange-red
COLOR_TEXT = (255, 255, 255)

HAZARD_COLOR_MAP = {
    "fire": COLOR_FIRE,
    "fire_placeholder": COLOR_FIRE,
    "smoke_placeholder": (180, 180, 180),
    "methane": COLOR_METHANE,
    "low_oxygen": COLOR_LOW_O2,
    "co": COLOR_CO,
    "person": COLOR_PERSON,
}


def draw_detections_on_frame(frame_bgr, detections):
    out = frame_bgr.copy()
    for det in detections:
        x1, y1, x2, y2 = [int(v) for v in det.bbox]
        color = HAZARD_COLOR_MAP.get(det.label, (255, 255, 255))
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        tag = f"{det.label} {det.confidence:.2f} ~{det.distance_m:.1f}m"
        if det.is_placeholder:
            tag += " [placeholder]"
        cv2.putText(out, tag, (x1, max(15, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    return out


class MapCanvas:
    """Maintains a fixed-size OpenCV canvas representing the 2D map and
    redraws it each frame from the current MineMap state."""

    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height

    def _world_to_pixel(self, x, y, x_min, x_max, y_min, y_max):
        # Map world (x,y) meters into pixel coords, y flipped (image
        # origin is top-left, map +Y should go "up" on screen).
        pad = config.MAP_PADDING
        x_min, x_max = x_min - pad, x_max + pad
        y_min, y_max = y_min - pad, y_max + pad
        span_x = max(x_max - x_min, 1e-3)
        span_y = max(y_max - y_min, 1e-3)
        px = int((x - x_min) / span_x * self.width)
        py = int(self.height - (y - y_min) / span_y * self.height)
        return px, py

    def render(self, mine_map, pose):
        canvas = np.full((self.height, self.width, 3), 30, dtype=np.uint8)

        xs = [p[0] for p in mine_map.trajectory] or [0.0]
        ys = [p[1] for p in mine_map.trajectory] or [0.0]
        xs += [b[0] for b in mine_map.boundary_points]
        ys += [b[1] for b in mine_map.boundary_points]
        xs += [o.x for o in mine_map.map_objects]
        ys += [o.y for o in mine_map.map_objects]

        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        if x_max - x_min < 5:
            x_min -= 2.5
            x_max += 2.5
        if y_max - y_min < 5:
            y_min -= 2.5
            y_max += 2.5

        def w2p(x, y):
            return self._world_to_pixel(x, y, x_min, x_max, y_min, y_max)

        # --- Title ---
        cv2.putText(canvas, "2D MINE MAP", (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_TEXT, 1, cv2.LINE_AA)

        # --- START marker ---
        sx, sy = w2p(0, 0)
        cv2.circle(canvas, (sx, sy), 4, (0, 255, 255), -1)
        cv2.putText(canvas, "START", (sx + 6, sy - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)

        # --- Tunnel walls ---
        for (bx, by, side) in mine_map.boundary_points:
            px, py = w2p(bx, by)
            cv2.circle(canvas, (px, py), 2, COLOR_WALL, -1)

        # --- Trajectory ---
        pts = [w2p(x, y) for (x, y) in mine_map.trajectory]
        for i in range(1, len(pts)):
            cv2.line(canvas, pts[i - 1], pts[i], COLOR_TRAJECTORY, 2)

        # --- Hazards / people / other objects ---
        for obj in mine_map.map_objects:
            px, py = w2p(obj.x, obj.y)
            color = HAZARD_COLOR_MAP.get(obj.obj_type, (200, 200, 200))
            if obj.obj_type == "person":
                cv2.circle(canvas, (px, py), 7, color, -1)
                cv2.putText(canvas, obj.label or "P?", (px + 8, py - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
            elif obj.obj_type in ("fire", "fire_placeholder"):
                cv2.drawMarker(canvas, (px, py), color, cv2.MARKER_STAR, 14, 2)
            elif obj.obj_type in ("methane", "low_oxygen", "co"):
                cv2.circle(canvas, (px, py), 10, color, 2)
                cv2.putText(canvas, obj.obj_type[:4], (px + 10, py + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)
            else:
                cv2.rectangle(canvas, (px - 4, py - 4), (px + 4, py + 4), color, 1)

        # --- Rover ---
        rx, ry = w2p(pose.x, pose.y)
        cv2.circle(canvas, (rx, ry), 8, COLOR_ROVER, -1)
        heading_len = 18
        hx = int(rx + heading_len * np.cos(pose.theta))
        hy = int(ry - heading_len * np.sin(pose.theta))
        cv2.arrowedLine(canvas, (rx, ry), (hx, hy), COLOR_ROVER, 2, tipLength=0.35)
        cv2.putText(canvas, "ROVER", (rx + 10, ry + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_ROVER, 1, cv2.LINE_AA)

        return canvas


def build_status_panel(width, height, rover_status, sensors, environment,
                        counts, warnings, vo_status):
    panel = np.full((height, width, 3), 15, dtype=np.uint8)
    x0 = 12
    y = 20
    line_h = 18

    def put(text, x=x0, color=COLOR_TEXT, scale=0.5):
        nonlocal y
        cv2.putText(panel, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)
        y += line_h

    col_w = width // 4

    y = 20
    put("ROVER STATUS", color=(0, 255, 255), scale=0.55)
    put(f"X: {rover_status['x']:.2f} m   Y: {rover_status['y']:.2f} m")
    put(f"Heading: {rover_status['heading_deg']:.1f} deg")
    put(f"VO status: {vo_status}  (matches: {rover_status['matches']})")

    y = 20
    x0 = col_w + 12
    put("SIMULATED SENSORS", x=x0, color=(0, 255, 255), scale=0.55)
    put(f"Front: {sensors['front']:.2f} m", x=x0)
    put(f"Left:  {sensors['left']:.2f} m", x=x0)
    put(f"Right: {sensors['right']:.2f} m", x=x0)

    y = 20
    x0 = 2 * col_w + 12
    put("ENVIRONMENT (VISION)", x=x0, color=(0, 255, 255), scale=0.55)
    if environment:
        put("O2: N/A   CH4: N/A", x=x0)
        put(f"CO: {environment.get('co', 0):.1f} ppm  CO2: {environment.get('co2', 0):.0f} ppm", x=x0)
        put(f"Temp: {environment.get('temperature', 0):.1f} C [EST.]", x=x0)
    else:
        put("O2: --   CH4: --", x=x0)
        put("CO: --     CO2: --", x=x0)
        put("Temp: --", x=x0)

    y = 20
    x0 = 3 * col_w + 12
    put("DETECTION", x=x0, color=(0, 255, 255), scale=0.55)
    put(f"People: {counts['people']}  Fire: {counts['fire']}", x=x0)
    put(f"Visual hazards: {len(warnings)}", x=x0)
    warn_text = ", ".join(f"{w['type']}:{w['level']}" for w in warnings[:2]) or "none"
    put(f"{warn_text}", x=x0, color=(0, 0, 255) if warnings else COLOR_TEXT, scale=0.45)

    return panel


def compose_dashboard(video_frame_bgr, map_canvas_bgr, status_panel_bgr):
    target_h = 480
    vf = cv2.resize(video_frame_bgr, (int(video_frame_bgr.shape[1] * target_h / video_frame_bgr.shape[0]), target_h))
    mc = cv2.resize(map_canvas_bgr, (vf.shape[0], target_h)) if False else map_canvas_bgr
    mc = cv2.resize(map_canvas_bgr, (mc.shape[1], target_h))

    top = np.hstack([vf, mc]) if vf.shape[0] == mc.shape[0] else _hstack_pad(vf, mc)
    panel = cv2.resize(status_panel_bgr, (top.shape[1], status_panel_bgr.shape[0]))
    full = np.vstack([top, panel])
    return full


def _hstack_pad(a, b):
    h = max(a.shape[0], b.shape[0])
    a2 = cv2.copyMakeBorder(a, 0, h - a.shape[0], 0, 0, cv2.BORDER_CONSTANT)
    b2 = cv2.copyMakeBorder(b, 0, h - b.shape[0], 0, 0, cv2.BORDER_CONSTANT)
    return np.hstack([a2, b2])


def save_final_map_png(mine_map, path):
    """Nicer static Matplotlib rendering of the final map, saved once
    at the end of the run per the project spec."""
    fig, ax = plt.subplots(figsize=(9, 7))

    if mine_map.boundary_points:
        bx = [p[0] for p in mine_map.boundary_points]
        by = [p[1] for p in mine_map.boundary_points]
        ax.scatter(bx, by, s=6, c="gray", label="tunnel wall (simulated)")

    if mine_map.trajectory:
        tx = [p[0] for p in mine_map.trajectory]
        ty = [p[1] for p in mine_map.trajectory]
        ax.plot(tx, ty, "-", color="tab:blue", linewidth=2, label="rover trajectory")
        ax.plot(tx[-1], ty[-1], "o", color="orange", markersize=10, label="rover (final)")

    ax.plot(0, 0, "*", color="gold", markersize=14, label="START")

    plotted_labels = set()
    for obj in mine_map.map_objects:
        color = {
            "person": "green", "fire": "red", "fire_placeholder": "red",
            "smoke_placeholder": "dimgray", "methane": "darkorange",
            "low_oxygen": "magenta", "co": "orangered",
        }.get(obj.obj_type, "black")
        lbl = obj.obj_type if obj.obj_type not in plotted_labels else None
        plotted_labels.add(obj.obj_type)
        marker = "^" if obj.obj_type == "person" else "X"
        ax.scatter(obj.x, obj.y, c=color, marker=marker, s=90, label=lbl, edgecolors="black")
        if obj.label:
            ax.annotate(obj.label, (obj.x, obj.y), textcoords="offset points",
                        xytext=(6, 6), fontsize=8)

    ax.set_title("REACT Mine Simulation -- Final 2D Map (SIMULATED)")
    ax.set_xlabel("X (meters, simulated scale)")
    ax.set_ylabel("Y (meters, simulated scale)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
