"""
Vision-derived hazard state for the R.E.A.C.T. video simulation.

There are NO predetermined hazard zones in this version. Environmental
variables remain blank until the perception pipeline detects a visible fire.
When visual fire evidence is present, the module exposes indicative fire
conditions so the dashboard can demonstrate how perception would update the
R.E.A.C.T. environmental state. These values are simulated estimates, not
physical measurements.
"""

import config


class HazardSimulator:
    def __init__(self):
        self._last_fire = False

    def sample_from_detections(self, detections):
        """Return environment values and warnings derived ONLY from vision."""
        fire_detections = [
            d for d in detections
            if d.label in ("fire", "fire_placeholder")
        ]

        if not fire_detections:
            self._last_fire = False
            return {}, []

        self._last_fire = True
        strongest = max(fire_detections, key=lambda d: d.confidence)
        env = dict(config.FIRE_ENVIRONMENT_ESTIMATE)
        warnings = [{
            "zone_id": f"VISUAL-FIRE-{strongest.confidence:.2f}",
            "type": "fire",
            "level": "HAZARD",
            "distance_to_center": round(strongest.distance_m, 2),
            "source": "vision",
            "confidence": round(strongest.confidence, 2),
        }]
        return env, warnings

    def classify_safety(self, env: dict) -> str:
        if not env:
            return "unknown"
        if env.get("temperature", 0) > 55 or env.get("co", 0) > 100:
            return "red"
        return "yellow"

    def zone_hazard_records(self):
        """No static hazard records: hazards must come from video perception."""
        return []
