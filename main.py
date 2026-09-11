"""
main.py
========
Entry point for the REACT live-camera perception and mapping prototype.

Usage:
    python main.py

The default input is the computer's live camera. Press Q in the dashboard
window to stop.

This is a SIMULATION / PROTOTYPE of a disaster-response mine rover's
perception and mapping software. It does not use, and is not a
substitute for, certified mine-safety hardware (real gas sensors, real
LiDAR/ultrasonic, GPS, etc). See config.py and the module docstrings
for exactly which numbers are estimated from real video vs simulated.
"""

import sys
import config
from video_processor import VideoProcessor


DISCLAIMER = """
==============================================================================
 REACT Mine Simulation -- Prototype / Proof-of-Concept
==============================================================================
 This program is a SOFTWARE SIMULATION for architecture/demo purposes.
 It is NOT a certified mine-rescue or mine-safety instrument.

  - Rover position: estimated via monocular visual odometry (ORB feature
    matching + Essential Matrix). Monocular video has NO true metric
    scale; translation magnitude uses a SIMULATED constant
    (config.METERS_PER_FRAME = {mpf}), not a measured distance.

  - Ultrasonic sensor readings: SIMULATED from image edge-density
    heuristics, not real acoustic time-of-flight hardware.

  - Gas / oxygen / methane values are blank because no physical gas
    sensors are connected. Fire-related CO / CO2 / temperature values are
    indicative demo estimates populated only after visible fire detection.

  - Object distances: estimated from bounding-box height using an
    assumed real-world object height and an assumed focal length
    (config.py) -- a rough monocular approximation, not calibrated
    depth sensing.

 Treat every number this program prints as an approximation useful for
 demonstrating the mapping/perception pipeline, not as ground truth.
==============================================================================
""".format(mpf=config.METERS_PER_FRAME)


SUGGESTED_IMPROVEMENTS = """
Suggested next steps to move this from simulation toward a more accurate
real-world system (see README.md for more detail on each):
  1. Stereo camera (or RGB-D) for real, calibrated depth instead of
     monocular scale-ambiguous estimates.
  2. Real IMU integration for orientation/acceleration ground truth.
  3. Wheel encoder integration for real odometry scale.
  4. Real ultrasonic or LiDAR hardware for actual obstacle ranging.
  5. Real calibrated gas sensors (O2 / CH4 / CO / CO2) instead of
     vision-derived demo estimates.
  6. RTAB-Map or ORB-SLAM3 for a full, loop-closing visual SLAM backend.
  7. ROS 2 integration for a real robot middleware/sensor-fusion stack.
  8. Proper camera calibration (intrinsics/distortion) instead of the
     assumed focal length used here.
  9. Learned monocular or stereo depth estimation networks.
 10. A mine-specific YOLO model trained on real fire/smoke/obstacle data
     (drop the weights at models/mine_custom.pt to auto-enable it).
 11. Sensor fusion (VO + IMU + wheel encoders) via an EKF/UKF for a much
     more robust pose estimate.
 12. Loop closure detection to correct accumulated drift on longer runs.
 13. True probabilistic occupancy-grid mapping instead of scattered
     boundary point estimates.
"""


def main():
    print(DISCLAIMER)

    camera_index = config.CAMERA_INDEX
    if len(sys.argv) > 1:
        try:
            camera_index = int(sys.argv[1])
        except ValueError:
            print("[main] ERROR: optional camera index must be an integer, e.g. python main.py 1")
            sys.exit(1)

    processor = VideoProcessor(source=camera_index, source_name="camera")
    processor.run()

    print(SUGGESTED_IMPROVEMENTS)
    print("[main] Done. See the output/ folder for final_mine_map.png and CSV logs.")


if __name__ == "__main__":
    main()
