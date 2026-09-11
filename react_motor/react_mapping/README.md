# REACT (Reactive Hazard Exploration/Response Rover)
## Real-Time 2D Ultrasonic Mapping & IMU-Based Rover Tracking

Designed for underground coal-mine and disaster environment exploration.

### Project Structure
```
react_mapping/
├── main.py                  # Main system entry point & orchestration
├── config.py                # Hardware, geometry, filter & map settings
├── serial_reader.py         # Thread-safe USB serial reader & simulator
├── imu.py                   # MPU-6050/6500 & Magnetometer driver/calibration
├── sensor_fusion.py         # Tilt-compensated compass & Complementary filter
├── odometry.py              # Kinematic odometry & dead-reckoning engine
├── mapping.py               # 2D Bayesian occupancy grid & Bresenham ray-tracer
├── coordinate_transform.py  # SE(2) rigid-body kinematic projections
├── visualization.py         # Real-time Matplotlib Ground Control Station GUI
├── filters.py               # Median filter, EMA, and outlier rejection
├── requirements.txt         # Python dependencies
└── arduino/
    ├── react_sensor_node.ino    # Production Arduino firmware (with real HC-SR04, MPU-6050, HMC5883L)
    └── react_simulated_node.ino # Mock Arduino simulation firmware (Static rover rotating in place, NO sensors needed!)
```

### Quick Start (Windows / Linux)

1. Create a virtual environment and install dependencies:
   ```bash
   python -m venv venv
   # Windows:
   venv\Scripts\activate
   # Linux/Mac:
   source venv/bin/activate

   pip install -r requirements.txt
   ```

2. Run in Software Simulation Mode (No hardware at all):
   ```bash
   python main.py
   ```
   By default, `config.py` has `SIMULATION_MODE = True`. The rover will simulate navigating an underground mine tunnel, casting 4 ultrasonic rays against walls and pillars in real time.

3. Test with Arduino Board without Sensors (`react_simulated_node.ino`):
   To test that your physical Arduino and USB connection communicate properly with Python without wiring any sensors:
   - Flash `arduino/react_simulated_node.ino` to your Arduino board via Arduino IDE.
   - The vehicle stays static at `(0, 0)` but rotates continuously in place, simulating 360° ultrasonic ray casts and 9-DOF IMU data.
   - In `config.py`, set `SIMULATION_MODE = False` and `SERIAL_PORT = "COM3"` (your Arduino port).
   - Run `python main.py`.

4. Deploy on Real Hardware with Physical Sensors (`react_sensor_node.ino`):
   In `config.py`:
   ```python
   SIMULATION_MODE = False
   SERIAL_PORT = "COM3"  # Adjust to your Arduino's COM port
   BAUD_RATE = 115200
   ```
   Upload `arduino/react_sensor_node.ino` to the Arduino Uno, then run:
   ```bash
   python main.py
   ```
