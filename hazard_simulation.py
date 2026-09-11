"""
hazard_simulation.py
======================
Purely software-simulated environmental hazard system.

THESE ARE SIMULATED VALUES ONLY -- NOT REAL SAFETY MEASUREMENTS.
This module does not read any physical gas/temperature sensor. It
generates plausible numbers based on:
  - a safe baseline environment (config.BASELINE_ENVIRONMENT)
  - predefined hazard zones with a location + radius + type
    (config.HAZARD_ZONES)
  - the rover's current simulated position relative to those zones

Do not use this module, or this project, as an actual mine-safety
instrument. It is a prototype/demo for perception & mapping software
architecture only.
"""

import math
import random

import config


class HazardSimulator:
    def __init__(self):
        self.zones = config.HAZARD_ZONES
        self.baseline = dict(config.BASELINE_ENVIRONMENT)
        self._rng = random.Random(42)  # deterministic-ish jitter

    def _distance(self, x1, y1, x2, y2):
        return math.hypot(x2 - x1, y2 - y1)

    def sample_environment(self, rover_x, rover_y):
        """
        Returns (environment_dict, active_warnings_list) for the rover's
        current position. environment_dict blends the baseline with the
        influence of any nearby hazard zone(s), weighted by proximity.
        """
        env = dict(self.baseline)
        warnings = []

        for zone in self.zones:
            dist = self._distance(rover_x, rover_y, zone["x"], zone["y"])
            radius = zone["radius"]
            warn_radius = radius * config.HAZARD_WARNING_RADIUS_MULTIPLIER

            if dist > warn_radius:
                continue  # too far, no influence

            # Influence strength: 1.0 at the zone center, fading to 0 at
            # warn_radius. Simple linear falloff -- a simulation choice,
            # not a physically modeled gas-diffusion equation.
            strength = max(0.0, 1.0 - dist / warn_radius)

            if zone["type"] == "low_oxygen":
                env["oxygen"] = env["oxygen"] - strength * 6.0  # can drop toward ~15%
            elif zone["type"] == "methane":
                env["methane"] = env["methane"] + strength * 4.5  # up toward ~4.5%
            elif zone["type"] == "fire":
                env["temperature"] = env["temperature"] + strength * 40.0
                env["co"] = env["co"] + strength * 150
                env["co2"] = env["co2"] + strength * 2000
            elif zone["type"] == "co":
                env["co"] = env["co"] + strength * 80

            if dist <= radius:
                warnings.append({
                    "zone_id": zone["id"],
                    "type": zone["type"],
                    "level": "HAZARD",
                    "distance_to_center": round(dist, 2),
                })
            elif dist <= warn_radius:
                warnings.append({
                    "zone_id": zone["id"],
                    "type": zone["type"],
                    "level": "WARNING",
                    "distance_to_center": round(dist, 2),
                })

        # Small random sensor jitter so the dashboard doesn't look static.
        env["oxygen"] = round(env["oxygen"] + self._rng.uniform(-0.1, 0.1), 2)
        env["methane"] = round(max(0.0, env["methane"] + self._rng.uniform(-0.02, 0.02)), 2)
        env["co"] = round(max(0.0, env["co"] + self._rng.uniform(-1, 1)), 1)
        env["co2"] = round(max(350, env["co2"] + self._rng.uniform(-5, 5)), 1)
        env["temperature"] = round(env["temperature"] + self._rng.uniform(-0.2, 0.2), 2)

        return env, warnings

    def classify_safety(self, env: dict) -> str:
        """Rough traffic-light classification for dashboard color-coding."""
        if env["oxygen"] < 17 or env["methane"] > 3.0 or env["temperature"] > 55 or env["co"] > 100:
            return "red"
        if env["oxygen"] < 19.5 or env["methane"] > 1.0 or env["temperature"] > 35 or env["co"] > 35:
            return "yellow"
        return "green"

    def zone_hazard_records(self):
        """Static list of hazard-zone records for map plotting/CSV export."""
        return [
            {
                "type": z["type"],
                "id": z["id"],
                "x": z["x"],
                "y": z["y"],
                "radius": z["radius"],
            }
            for z in self.zones
        ]
