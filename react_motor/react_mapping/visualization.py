"""
==============================================================================
REACT (Reactive Hazard Exploration/Response Rover)
Real-Time 2D Robotics Visualization: visualization.py
==============================================================================
Professional 2D Ground Control Station GUI using Matplotlib:
- Large 2D Map Canvas with metric grid, coordinate axes, and distance scale.
- Persistent Occupancy Grid + obstacle point cloud.
- Rover polygon with heading arrow and historical trajectory breadcrumb trail.
- 4 Ultrasonic sensor rays with dynamic color-coded range alerts.
- Dedicated Right-Side Telemetry & Diagnostics HUD displaying:
  * Connection Status & Health Badges
  * Metric Coordinates (X, Y, Dist)
  * Heading (deg, rad, Compass Rose)
  * 4-Sensor Distance Bar Indicators
  * 9-DOF IMU (Accelerometer, Gyroscope, Magnetometer)
- Interactive buttons for [Reset Map], [Clear Path], and [Set Origin].
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle, FancyArrow
from matplotlib.widgets import Button
from typing import Dict, Any, List

from config import (
    ROVER_LENGTH, ROVER_WIDTH, SENSOR_CONFIG,
    MAP_WIDTH_METERS, MAP_HEIGHT_METERS, MAP_RESOLUTION,
    MIN_ULTRASONIC_DISTANCE, MAX_ULTRASONIC_DISTANCE
)
from coordinate_transform import CoordinateTransformer


class RoverVisualizer:
    """
    Manages the real-time Matplotlib interactive dashboard.
    Supports keyboard teleoperation (Arrow Keys: Up, Down, Left, Right, Space)
    and interactive real-time trajectory path tracing.
    """

    def __init__(self, on_reset_callback=None, on_clear_path_callback=None, on_drive_callback=None):
        self.on_reset_callback = on_reset_callback
        self.on_clear_path_callback = on_clear_path_callback
        self.on_drive_callback = on_drive_callback
        self.active_keys = set()

        # Setup modern dark aerospace theme
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(15, 8.5), facecolor='#0d1117')
        self.fig.canvas.manager.set_window_title("REACT Rover - Ground Control Station & 2D Mapping")

        # Connect keyboard events for arrow-key rover control & path tracing
        self.fig.canvas.mpl_connect('key_press_event', self._on_key_press)
        self.fig.canvas.mpl_connect('key_release_event', self._on_key_release)

        # Grid specification: 75% width for Map, 25% for Telemetry HUD
        gs = self.fig.add_gridspec(1, 2, width_ratios=[3.4, 1.3], wspace=0.12,
                                   left=0.04, right=0.97, top=0.93, bottom=0.08)

        self.ax_map = self.fig.add_subplot(gs[0, 0])
        self.ax_hud = self.fig.add_subplot(gs[0, 1])

        self._setup_map_axes()
        self._setup_hud_axes()
        self._setup_buttons()

        # Map drawing artists
        self.path_line, = self.ax_map.plot([], [], color='#00d4ff', linewidth=2.0, alpha=0.85, label='Trajectory')
        self.obstacle_scatter = self.ax_map.scatter([], [], s=20, c='#ff4757', marker='s', edgecolors='none', label='Obstacles')

        # Rover chassis polygon (in local frame, then rotated)
        # Rectangular body with front notch
        half_l = ROVER_LENGTH / 2.0
        half_w = ROVER_WIDTH / 2.0
        self.local_chassis = [
            (half_l, half_w * 0.5),
            (half_l * 1.2, 0.0),       # triangular nose
            (half_l, -half_w * 0.5),
            (half_l, -half_w),
            (-half_l, -half_w),
            (-half_l, half_w),
            (half_l, half_w)
        ]
        self.rover_patch = Polygon(self.local_chassis, closed=True, facecolor='#2ed573', edgecolor='#ffffff', linewidth=1.5, zorder=10)
        self.ax_map.add_patch(self.rover_patch)

        # Heading direction arrow
        self.heading_arrow = None

        # 3 Sensor Ray lines (FRONT, LEFT, RIGHT)
        self.ray_artists = {}
        colors = {
            "FRONT": "#ffa502",  # Amber for forward
            "LEFT":  "#2ed573",  # Green for left perpendicular
            "RIGHT": "#70a1ff",  # Blue for right perpendicular
            "FL": "#ffa502", "FR": "#2ed573", "RL": "#70a1ff"
        }
        for s_key in SENSOR_CONFIG:
            line, = self.ax_map.plot([], [], color=colors.get(s_key, '#eccc68'),
                                     linestyle='--', linewidth=1.5, alpha=0.9, zorder=8)
            contact = Circle((0, 0), radius=0.06, color=colors.get(s_key, '#eccc68'), zorder=9)
            self.ax_map.add_patch(contact)
            self.ray_artists[s_key] = {"line": line, "contact": contact}

        # Grid matrix raster image for occupancy map
        rows = int(round(MAP_HEIGHT_METERS / MAP_RESOLUTION))
        cols = int(round(MAP_WIDTH_METERS / MAP_RESOLUTION))
        empty_grid = np.full((rows, cols), 0.5)
        extent = [-MAP_WIDTH_METERS/2, MAP_WIDTH_METERS/2, -MAP_HEIGHT_METERS/2, MAP_HEIGHT_METERS/2]
        self.grid_im = self.ax_map.imshow(empty_grid, cmap='bone_r', vmin=0.0, vmax=1.0,
                                          extent=extent, origin='lower', alpha=0.45, zorder=1)

    def _setup_map_axes(self):
        self.ax_map.set_facecolor('#161b22')
        self.ax_map.set_title("REACT ROVER // 2D HAZARD OCCUPANCY MAP",
                              color='#58a6ff', fontsize=12, fontweight='bold', pad=10)
        self.ax_map.set_xlabel("Global X (East) [meters]", color='#8b949e', fontsize=10)
        self.ax_map.set_ylabel("Global Y (North) [meters]", color='#8b949e', fontsize=10)
        self.ax_map.set_xlim(-MAP_WIDTH_METERS/2, MAP_WIDTH_METERS/2)
        self.ax_map.set_ylim(-MAP_HEIGHT_METERS/2, MAP_HEIGHT_METERS/2)
        self.ax_map.set_aspect('equal', 'box')
        self.ax_map.grid(True, color='#30363d', linestyle=':', linewidth=0.8, alpha=0.7)
        self.ax_map.tick_params(colors='#8b949e')
        for spine in self.ax_map.spines.values():
            spine.set_color('#30363d')

        # On-map keyboard teleop overlay badge
        self.teleop_badge = self.ax_map.text(
            0.025, 0.035,
            "KEYBOARD TELEOP: [↑] Forward  [↓] Reverse  [←] Left  [→] Right  [Space] Stop",
            color='#58a6ff', fontsize=8.5, fontfamily='monospace', fontweight='bold',
            transform=self.ax_map.transAxes, zorder=25,
            bbox=dict(boxstyle="round,pad=0.4", facecolor="#0d1117", edgecolor="#30363d", alpha=0.92, lw=1)
        )

    def _setup_hud_axes(self):
        self.ax_hud.set_facecolor('#161b22')
        self.ax_hud.set_title("ROVER TELEMETRY HUD", color='#7ee787', fontsize=11, fontweight='bold', pad=10)
        self.ax_hud.set_xticks([])
        self.ax_hud.set_yticks([])
        for spine in self.ax_hud.spines.values():
            spine.set_color('#30363d')

        # Static placeholder text block that gets dynamically updated
        self.hud_text = self.ax_hud.text(
            0.05, 0.95, "Awaiting Telemetry...",
            color='#c9d1d9', fontsize=9.5, fontfamily='monospace',
            verticalalignment='top', transform=self.ax_hud.transAxes,
            linespacing=1.35
        )

    def _setup_buttons(self):
        # Interactive action buttons at bottom of window
        ax_btn_reset = self.fig.add_axes([0.68, 0.02, 0.13, 0.04])
        self.btn_reset = Button(ax_btn_reset, 'Reset Map & Origin', color='#21262d', hovercolor='#da3633')
        self.btn_reset.label.set_color('#f85149')
        self.btn_reset.label.set_fontsize(9)
        self.btn_reset.on_clicked(self._handle_reset)

        ax_btn_clear = self.fig.add_axes([0.83, 0.02, 0.13, 0.04])
        self.btn_clear = Button(ax_btn_clear, 'Clear Path Trail', color='#21262d', hovercolor='#1f6feb')
        self.btn_clear.label.set_color('#58a6ff')
        self.btn_clear.label.set_fontsize(9)
        self.btn_clear.on_clicked(self._handle_clear_path)

    def _handle_reset(self, event):
        if self.on_reset_callback:
            self.on_reset_callback()

    def _handle_clear_path(self, event):
        if self.on_clear_path_callback:
            self.on_clear_path_callback()

    def _on_key_press(self, event):
        """Processes keyboard presses for Arrow keys (Up, Down, Left, Right) and shortcuts."""
        if not event or not event.key:
            return
        k = event.key.lower()

        # Arrow keys or WASD equivalents
        if k in ('up', 'w'):
            self.active_keys.add('up')
            if self.on_drive_callback:
                self.on_drive_callback('up', is_press=True)
        elif k in ('down', 's'):
            self.active_keys.add('down')
            if self.on_drive_callback:
                self.on_drive_callback('down', is_press=True)
        elif k in ('left', 'a'):
            self.active_keys.add('left')
            if self.on_drive_callback:
                self.on_drive_callback('left', is_press=True)
        elif k in ('right', 'd'):
            self.active_keys.add('right')
            if self.on_drive_callback:
                self.on_drive_callback('right', is_press=True)
        elif k in (' ', 'space'):
            self.active_keys.clear()
            if self.on_drive_callback:
                self.on_drive_callback('stop', is_press=True)
        elif k == 'r':
            self._handle_reset(None)
        elif k == 'c':
            self._handle_clear_path(None)

    def _on_key_release(self, event):
        """Tracks key release to smoothly halt continuous motion."""
        if not event or not event.key:
            return
        k = event.key.lower()

        if k in ('up', 'w'):
            self.active_keys.discard('up')
            if self.on_drive_callback:
                self.on_drive_callback('up', is_press=False)
        elif k in ('down', 's'):
            self.active_keys.discard('down')
            if self.on_drive_callback:
                self.on_drive_callback('down', is_press=False)
        elif k in ('left', 'a'):
            self.active_keys.discard('left')
            if self.on_drive_callback:
                self.on_drive_callback('left', is_press=False)
        elif k in ('right', 'd'):
            self.active_keys.discard('right')
            if self.on_drive_callback:
                self.on_drive_callback('right', is_press=False)

    def update(self,
               rover_x: float,
               rover_y: float,
               heading_rad: float,
               path_x: List[float],
               path_y: List[float],
               obstacle_points: List[tuple],
               sensor_rays: Dict[str, dict],
               prob_grid: np.ndarray,
               telemetry: Dict[str, Any]):
        """
        Refreshes all visualization elements on the canvas.
        """
        # 1. Update Trajectory Path
        if path_x and path_y:
            self.path_line.set_data(path_x, path_y)

        # 2. Update Obstacle Scatter
        if obstacle_points:
            ox, oy = zip(*obstacle_points)
            self.obstacle_scatter.set_offsets(np.c_[ox, oy])

        # 3. Update Rover Chassis Polygon
        transformed_chassis = CoordinateTransformer.transform_polygon(
            self.local_chassis, rover_x, rover_y, heading_rad
        )
        self.rover_patch.set_xy(transformed_chassis)

        # 4. Update Heading Vector Arrow
        arrow_len = 0.5
        dx = arrow_len * math.cos(heading_rad)
        dy = arrow_len * math.sin(heading_rad)
        if self.heading_arrow:
            self.heading_arrow.remove()
        self.heading_arrow = self.ax_map.annotate(
            "", xy=(rover_x + dx, rover_y + dy), xytext=(rover_x, rover_y),
            arrowprops=dict(arrowstyle="->", color="#ffffff", lw=2.0)
        )

        # 5. Update Ultrasonic Rays & Contact Points
        for s_key, r_art in self.ray_artists.items():
            s_data = sensor_rays.get(s_key)
            if s_data and s_data.get("origin") and s_data.get("target"):
                ox, oy = s_data["origin"]
                tx, ty = s_data["target"]
                r_art["line"].set_data([ox, tx], [oy, ty])
                r_art["contact"].center = (tx, ty)
                r_art["contact"].set_visible(s_data.get("valid", False))
                # Alert color if obstacle is close (< 0.5 m)
                dist = s_data.get("dist", 99.0)
                if dist < 0.4:
                    r_art["line"].set_color("#ff4757")
                elif dist < 1.0:
                    r_art["line"].set_color("#ffa502")
                else:
                    r_art["line"].set_color("#2ed573")
            else:
                r_art["line"].set_data([], [])
                r_art["contact"].set_visible(False)

        # 6. Update Occupancy Grid Texture
        if prob_grid is not None:
            self.grid_im.set_data(prob_grid)

        # 7. Update Telemetry HUD
        hud_str = self._format_hud_text(rover_x, rover_y, heading_rad, telemetry)
        self.hud_text.set_text(hud_str)

    def _format_hud_text(self, rx: float, ry: float, heading_rad: float, t: Dict[str, Any]) -> str:
        heading_deg = t.get("HDG", (math.degrees(heading_rad)) % 360.0)
        conn_status = t.get("status", "STANDBY")
        mode = "SIMULATION" if t.get("is_sim", False) else "HARDWARE SERIAL"
        speed = t.get("speed", 0.0)
        dist_front = t.get("FRONT", t.get("FL", 0.0)) * 100.0  # cm
        dist_left = t.get("LEFT", t.get("FR", 0.0)) * 100.0
        dist_right = t.get("RIGHT", t.get("RL", 0.0)) * 100.0

        ax = t.get("ax", 0.0)
        ay = t.get("ay", 0.0)
        az = t.get("az", 1.0)
        gx = t.get("gx", 0.0)
        gy = t.get("gy", 0.0)
        gz = t.get("gz", 0.0)
        mx = t.get("mx", 0.0)
        my = t.get("my", 0.0)
        mz = t.get("mz", 0.0)

        lines = [
            f"SYSTEM: REACT ROVER v2.5",
            f"MODE  : {mode}",
            f"LINK  : {conn_status[:24]}",
            f"PKTS  : {t.get('packet_id', 0)}  [ERR: {t.get('errors', 0)}]",
            "─" * 28,
            "KEYBOARD TELEOP (ARROWS)",
            f"  State  : {t.get('drive_status', 'READY')}",
            f"  Keys   : [↑] Fwd  [↓] Rev",
            f"           [←] Left [→] Right",
            f"  Trail  : {t.get('path_len', 1)} breadcrumbs",
            "─" * 28,
            "GY-271 DIGITAL COMPASS",
            f"  Heading: {heading_deg:5.1f}° [{heading_rad:+4.2f} rad]",
            f"  Mag X  : {mx:+6.1f} uT",
            f"  Mag Y  : {my:+6.1f} uT",
            f"  Mag Z  : {mz:+6.1f} uT",
            "─" * 28,
            "POSITION & KINEMATICS",
            f"  X      : {rx:+6.2f} m",
            f"  Y      : {ry:+6.2f} m",
            f"  Speed  : {speed:5.2f} m/s",
            "─" * 28,
            "ULTRASONIC RANGES (cm)",
            f"  FRONT (0°) : {dist_front:5.1f} cm",
            f"  LEFT (+90°): {dist_left:5.1f} cm",
            f"  RIGHT(-90°): {dist_right:5.1f} cm",
            "─" * 28,
            f"MAPPING: ACTIVE (5cm res)",
            f"OBSTACLES LOGGED: {t.get('obstacle_count', 0)}"
        ]
        return "\n".join(lines)
