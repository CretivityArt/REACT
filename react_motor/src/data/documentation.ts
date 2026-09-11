/**
 * REACT (Reactive Hazard Exploration/Response Rover)
 * 14 Engineering Deliverables & Technical Manual
 */

export interface DocSection {
  id: string;
  title: string;
  badge: string;
  summary: string;
  content: string;
}

export const ENGINEERING_DOCS: DocSection[] = [
  {
    id: "architecture",
    title: "1. Complete Project Architecture & Data Flow",
    badge: "Architecture",
    summary: "Decoupled hardware-agnostic pipeline separating serial acquisition, sensor fusion, odometry, coordinate projection, and occupancy mapping.",
    content: `### System Architecture Overview

The REACT Ground Control Station is organized around a strict **hardware-independent architecture**. The 2D mapping engine has zero direct dependencies on the underlying physical transducers, allowing ultrasonic rangefinders to be swapped for 2D LiDAR or depth cameras in the future without modifying coordinate transforms or visualization.

\`\`\`text
                 +--------------------------------+
                 | Arduino Uno / Sensor Node      |
                 | (HC-SR04 x4, MPU-6050, HMC5883)|
                 +---------------+----------------+
                                 | USB Serial (115200 baud)
                                 v
                 +---------------+----------------+
                 | serial_reader.py (Worker Thread)|
                 | Parses packets, validates CS,  |
                 | handles auto-reconnect/sim     |
                 +---------------+----------------+
                                 | Thread-safe Queue (queue.Queue)
                                 v
                 +---------------+----------------+
                 | filters.py                     |
                 | Rolling Median & Outlier Gate  |
                 +---------------+----------------+
                                 | Clean telemetry
                                 v
    +----------------------------+----------------------------+
    |                                                         |
    v                                                         v
+--------------------------+              +--------------------------+
| imu.py & sensor_fusion.py|              | odometry.py              |
| Tilt-compensated compass |              | Differential kinematic   |
| + Complementary Gyro-Yaw |              | dead-reckoning           |
+-------------+------------+              +------------+-------------+
              | Heading theta (rad)                    | Pose (X, Y)
              +-------------------+--------------------+
                                  |
                                  v
                  +---------------+----------------+
                  | coordinate_transform.py        |
                  | SE(2) Rigid Body Kinematics    |
                  | Sensor -> Rover -> Global Map  |
                  +---------------+----------------+
                                  |
                                  v
                  +---------------+----------------+
                  | mapping.py                     |
                  | 2D Bayesian Occupancy Grid     |
                  | Bresenham ray-tracer (Free/Occ)|
                  +---------------+----------------+
                                  |
                                  v
                  +---------------+----------------+
                  | visualization.py (Main Thread) |
                  | Real-Time Top-Down GCS Display |
                  +--------------------------------+
\`\`\`

### File Responsibilities
* **\`main.py\`**: Coordinates the lifecycle, drains queue, executes processing loop, and runs the interactive GUI.
* **\`config.py\`**: Single source of truth for ports, sensor angles, offsets, grid resolution, and filter gains.
* **\`serial_reader.py\`**: Dedicated worker thread isolating serial I/O so the GUI never drops frames or hangs.
* **\`filters.py\`**: Digital signal processing to reject multi-path acoustic reflections.
* **\`imu.py\`**: Calibration registers and raw unit normalization for MPU-6050 and external magnetometer.
* **\`sensor_fusion.py\`**: Accelerometer tilt compensation and drift-free yaw fusion.
* **\`odometry.py\`**: Robot pose tracking $(X, Y, \\theta)$ with wheel encoder expansion hooks.
* **\`coordinate_transform.py\`**: Forward kinematics mapping sensor polar rays to global Cartesian coordinates.
* **\`mapping.py\`**: Bayesian log-odds occupancy grid with free-space carving.
* **\`visualization.py\`**: Real-time Matplotlib dashboard with telemetry HUD and interactive controls.`
  },
  {
    id: "arduino",
    title: "2. Complete Schematic Diagram & Arduino Wiring",
    badge: "Hardware & Schematic",
    summary: "Full electrical schematic, pinout table, and wiring diagram for L298N Dual H-Bridge motor driver, 3x HC-SR04 ultrasonics, MPU-6050 IMU, and optional GY-271 Compass on Arduino Uno.",
    content: `### Electrical Schematic & Wiring Diagram

The diagram below details the exact connections between the **Arduino Uno**, the **L298N Dual H-Bridge Motor Driver**, the **3x HC-SR04 ultrasonic rangefinders**, the **MPU-6050 6-Axis MotionTracking IMU**, and the **optional GY-271 digital compass module**.

\`\`\`text
                 +-------------------------------------------------------------+
                 |                         ARDUINO UNO                         |
                 |                                                             |
                 |                                                  [USB/GCS]  |
                 |                                                   9600bd    |
                 |                                                             |
                 | [5V]   [GND]    [A4/SDA] [A5/SCL]  [D2-D7]    [D8-D12, A0]  |
                 +---+------+---------+--------+--------+-------------+--------+
                     |      |         |        |        |             |
    =================+      |         |        |        |             |  (5V Power Rail)
    |                       |         |        |        |             |
    |   ====================+         |        |        |             |  (Common Ground)
    |   |                             |        |        |             |
    |   |    MPU-6050 6-AXIS IMU      |        |        |             |
    |   |    +---------------------+  |        |        |             |
    +---+--->| VCC                 |  |        |        |             |
    |   +--->| GND                 |  |        |        |             |
    |        | SDA <---------------+--+        |        |             |
    |        | SCL <---------------------------+        |             |
    |        | AD0 -> GND          |                    |             |
    |        +---------------------+                    |             |
    |                                                   |             |
    |    3x HC-SR04 ULTRASONIC TRANSDUCERS              |             |
    |    +-------------------------------------------+  |             |
    +--->| FRONT VCC, LEFT VCC, RIGHT VCC            |  |             |
    | +->| FRONT GND, LEFT GND, RIGHT GND            |  |             |
    | |  | FRONT: TRIG -> D2,  ECHO -> D3            |<-+             |
    | |  | LEFT : TRIG -> D4,  ECHO -> D5            |<-+             |
    | |  | RIGHT: TRIG -> D6,  ECHO -> D7            |<-+             |
    | |  +-------------------------------------------+                |
    | |                                                               |
    | |  L298N DUAL H-BRIDGE MOTOR DRIVER                             |
    | |  +---------------------------------------------------------+  |
    | |  | LOGIC INPUTS (From Arduino):                            |  |
    | |  |   ENA (Left Motor PWM Speed)   <------------------------+--+-- Pin D9  (Timer 1)
    | |  |   IN1 (Left Motor Direction 1) <------------------------+--+-- Pin D8
    | |  |   IN2 (Left Motor Direction 2) <------------------------+--+-- Pin D10
    | |  |   ENB (Right Motor PWM Speed)  <------------------------+--+-- Pin D11 (Timer 2)
    | |  |   IN3 (Right Motor Direction 1)<------------------------+--+-- Pin D12
    | |  |   IN4 (Right Motor Direction 2)<------------------------+--+-- Pin A0  (Digital Out)
    | |  |                                                         |
    | +->| GND (Tied to Arduino GND AND Battery Negative (-))      |  (COMMON GROUND)
    |    |                                                         |
    |    | POWER INPUTS:                                           |
    |    |   12V / VMS <==== External Battery (+) (7.4V - 12V DC)  |
    |    |   GND       <==== External Battery (-)                  |
    |    |   5V        <---- 5V Logic (Keep jumper ON for <=12V)   |
    |    |                                                         |
    |    | MOTOR OUTPUTS:                                          |
    |    |   OUT1 & OUT2 ===> LEFT MOTOR (TT Gearmotor A)          |
    |    |   OUT3 & OUT4 ===> RIGHT MOTOR (TT Gearmotor B)         |
    |    +---------------------------------------------------------+
\`\`\`

---

### Complete Pinout Connection Table

| Device / Module | Module Pin | Arduino Uno Pin | Wire Color | Electrical Function |
| :--- | :--- | :--- | :--- | :--- |
| **L298N Motor Driver** | **ENA** | Digital Pin 9 (PWM) | Yellow | Left Motor Speed Control (PWM 0-255) |
| | **IN1** | Digital Pin 8 | Blue | Left Motor Direction Input 1 |
| | **IN2** | Digital Pin 10 | Green | Left Motor Direction Input 2 |
| | **ENB** | Digital Pin 11 (PWM) | Orange | Right Motor Speed Control (PWM 0-255) |
| | **IN3** | Digital Pin 12 | Purple | Right Motor Direction Input 1 |
| | **IN4** | Analog Pin A0 | Gray | Right Motor Direction Input 2 (GPIO Output) |
| | **12V / VMS** | Battery Pack (+) | Red (Heavy) | High-current motor supply (7.4V - 12V DC) |
| | **GND** | Battery (-) & Uno GND | Black (Heavy) | **Common Ground reference** (Crucial!) |
| | **5V Terminal** | Unconnected / Uno 5V | Red | Onboard 7805 5V regulator output |
| **Front HC-SR04** (0° Forward) | **VCC** | 5V Rail | Red | Regulated +5V DC Power |
| | **GND** | GND Rail | Black | Common Ground Reference |
| | **TRIG** | Digital Pin 2 | Yellow | 10 µs High Trigger Pulse Output |
| | **ECHO** | Digital Pin 3 | Green | Pulse duration return input (TTL 5V) |
| **Left HC-SR04** (+90° Perp) | **VCC** | 5V Rail | Red | Regulated +5V DC Power |
| | **GND** | GND Rail | Black | Common Ground Reference |
| | **TRIG** | Digital Pin 4 | Orange | 10 µs High Trigger Pulse Output |
| | **ECHO** | Digital Pin 5 | Blue | Pulse duration return input (TTL 5V) |
| **Right HC-SR04** (-90° Perp) | **VCC** | 5V Rail | Red | Regulated +5V DC Power |
| | **GND** | GND Rail | Black | Common Ground Reference |
| | **TRIG** | Digital Pin 6 | White | 10 µs High Trigger Pulse Output |
| | **ECHO** | Digital Pin 7 | Purple | Pulse duration return input (TTL 5V) |
| **MPU-6050 6-Axis IMU** | **VCC** | 5V Rail (or 3.3V) | Red | Power input (onboard 3.3V LDO) |
| | **GND** | GND Rail | Black | Common Ground Reference |
| | **SDA** | Analog Pin A4 | Cyan | I2C Serial Data line |
| | **SCL** | Analog Pin A5 | Gray | I2C Serial Clock line |
| | **AD0** | GND | Black | Set I2C slave address to 0x68 |

---

### Motor Power & Grounding Rules (L298N)

1. **Dual Power Source Isolation**:
   * **DO NOT** power the L298N motors directly from the Arduino 5V pin. TT DC gearmotors draw between 400mA and 1.5A stall current, which will cause the Arduino voltage regulator to overheat, brown out, and crash the MPU-6050 I2C bus.
   * Power the L298N 12V terminal using an external battery pack (e.g. 2S LiPo 7.4V, 3S 11.1V, or 6x AA batteries).
2. **Common Ground is Mandatory**:
   * Connect the L298N **GND terminal** to the battery negative (-) **AND** to an Arduino **GND pin**. Without a shared ground reference, logic signals to IN1-IN4 will float unpredictably.
3. **ENA / ENB Jumpers**:
   * If you leave the black 5V jumpers on ENA and ENB installed, motors will run at fixed maximum battery speed (4-wire control using IN1..IN4).
   * Removing the jumpers and connecting ENA -> D9 and ENB -> D11 enables full 0-255 8-bit hardware PWM speed control.

---

### Teleoperation Serial Command Protocol

When the user drives via keyboard arrows/WASD in Python or Web GCS, the computer transmits ASCII commands over USB serial:

| Command | Action | Left Motor (OUT1/2) | Right Motor (OUT3/4) | PWM Speed |
| :--- | :--- | :--- | :--- | :--- |
| **\`F\`** or **\`FORWARD\`** | Drive Forward | IN1=HIGH, IN2=LOW | IN3=HIGH, IN4=LOW | \`currentSpeed\` (Default 200) |
| **\`B\`** or **\`BACKWARD\`** | Reverse | IN1=LOW, IN2=HIGH | IN3=LOW, IN4=HIGH | \`currentSpeed\` (Default 200) |
| **\`L\`** or **\`LEFT\`** | Differential Spin Left | IN1=LOW, IN2=HIGH | IN3=HIGH, IN4=LOW | \`turnSpeed\` (Default 190) |
| **\`R\`** or **\`RIGHT\`** | Differential Spin Right | IN1=HIGH, IN2=LOW | IN3=LOW, IN4=HIGH | \`turnSpeed\` (Default 190) |
| **\`S\`** / **\` \`** / **\`STOP\`** | Emergency Halt / Brake | IN1=LOW, IN2=LOW | IN3=LOW, IN4=LOW | 0 (Off) |
| **\`1\`** - **\`5\`** | Speed Presets | - | - | 1=120, 2=160, 3=200, 4=230, 5=255 |
| **\`+\`** / **\`-\`** | Nudge Speed | - | - | Increases or decreases PWM by 20 |

**Safety Watchdog**:
The firmware includes a 2.0-second timeout watchdog. If the rover is driving forward and Python crashes or the serial connection drops, the Arduino automatically brakes to prevent runaway.

---

### Sensor Telemetry Protocol (Arduino -> Python)

\`\`\`text
DATA:#<seq>,DF=<cm>,DL=<cm>,DR=<cm>,HDG=<deg>,GZ=<dps>,AX=<g>,AY=<g>,AZ=<g>*
\`\`\`
Example:
\`\`\`text
DATA:#148,DF=123.4,DL=87.2,DR=91.5,HDG=12.35,GZ=1.24,AX=0.02,AY=-0.01,AZ=0.99*
\`\`\`
`
  },
  {
    id: "python_structure",
    title: "3. Modular Python Code Architecture",
    badge: "Software",
    summary: "Complete breakdown of the 10 Python modules, clean separation of concerns, and thread safety via queue.Queue.",
    content: `### Thread-Safe Communication via \`queue.Queue\`
A key architectural requirement in robotics visualization is avoiding UI thread freezes during serial timeouts.

\`\`\`python
# Worker Thread (serial_reader.py):
while self.running:
    packet = self.parse_packet(line)
    if packet:
        try:
            self.packet_queue.put_nowait(packet)
        except queue.Full:
            pass  # Drops oldest to maintain ultra-low latency

# GUI Thread (main.py):
def _animation_loop(self, frame_idx):
    while not self.packet_queue.empty():
        latest_packet = self.packet_queue.get_nowait()
    self._process_telemetry_packet(latest_packet, dt)
\`\`\`

This guarantees that even if the Arduino is unplugged or takes 2 seconds to reboot, the Matplotlib canvas continues redrawing smoothly at 20 FPS and immediately flags a **WARNING: Serial data timeout** in the HUD.`
  },
  {
    id: "requirements",
    title: "4. Dependencies & requirements.txt",
    badge: "Setup",
    summary: "Minimal, highly optimized Python dependencies: pyserial, numpy, and matplotlib.",
    content: `### Required Packages
\`\`\`text
pyserial>=3.5
numpy>=1.24.0
matplotlib>=3.7.0
\`\`\`

* **\`pyserial\`**: Provides high-performance binary and ASCII serial stream reading across Windows COM ports and Linux tty devices.
* **\`numpy\`**: Vectorized matrix transformations, occupancy grid log-odds calculations, and geometric rotations.
* **\`matplotlib\`**: High-performance interactive 2D Ground Control Station graphics with interactive buttons and real-time animation hooks.`
  },
  {
    id: "windows_setup",
    title: "5. Installation Instructions for Windows",
    badge: "Guide",
    summary: "Step-by-step setup guide for Windows 10/11 using PowerShell or Command Prompt.",
    content: `### Step 1: Install Python
1. Download Python 3.10, 3.11, or 3.12 from [python.org](https://www.python.org/downloads/).
2. During installation, **CHECK the box**: \`Add python.exe to PATH\`.

### Step 2: Create a Dedicated Virtual Environment
Open **PowerShell** or **Command Prompt**, navigate to the project directory:

\`\`\`powershell
cd C:\\path\\to\\react_mapping

# Create virtual environment
python -m venv venv

# Activate virtual environment in PowerShell:
.\\venv\\Scripts\\Activate.ps1
# (If execution policy restricts scripts, run: Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass)

# Or in standard Command Prompt (cmd.exe):
venv\\Scripts\\activate.bat
\`\`\`

### Step 3: Install Required Libraries
\`\`\`cmd
pip install --upgrade pip
pip install -r requirements.txt
\`\`\`

### Step 4: Verify Installation
\`\`\`cmd
python -c "import serial, numpy, matplotlib; print('All REACT dependencies successfully installed!')"
\`\`\``
  },
  {
    id: "com_port",
    title: "6. How to Identify the Arduino COM Port on Windows",
    badge: "Hardware",
    summary: "Discovering Arduino COM port via Windows Device Manager and automatic Python port enumeration.",
    content: `### Method A: Windows Device Manager
1. Press \`Win + X\` and select **Device Manager** (or run \`devmgmt.msc\`).
2. Expand the **Ports (COM & LPT)** branch.
3. Look for:
   * **Arduino Uno (COM3)**, or
   * **USB-SERIAL CH340 (COM4)** (common for Arduino clones), or
   * **USB Serial Device (COM5)**.
4. Note the port number (e.g. \`COM3\`).

### Method B: Python Port Scanner
Run this one-liner in your terminal:
\`\`\`cmd
python -c "import serial.tools.list_ports as lp; [print(f'Port: {p.device} | Description: {p.description}') for p in lp.comports()]"
\`\`\`

Update \`config.py\`:
\`\`\`python
SERIAL_PORT = "COM3"  # Replace with your identified port
\`\`\``
  },
  {
    id: "sim_mode",
    title: "7. Running in Simulation Mode (No Hardware Needed)",
    badge: "Testing",
    summary: "Simulated coal mine tunnel with perimeter walls, coal pillars, noisy IMU kinematics, and ray tracing.",
    content: `### Testing the Entire Mapping Stack Without Physical Hardware
You have two powerful simulation options:

#### Option A: Pure Python Software Simulator (Zero Hardware)
Simulation mode is enabled by default in \`config.py\`:

\`\`\`python
# In config.py:
SIMULATION_MODE = True
\`\`\`

Then launch the program:
\`\`\`cmd
python main.py
\`\`\`
* The \`SerialReaderThread\` spawns an internal physics simulator exploring a $12\\text{m} \\times 8\\text{m}$ tunnel with central support pillars and realistic sensor noise.

---

#### Option B: Real Arduino USB Simulation (\`react_simulated_node.ino\`)
If you want to verify that **the physical Arduino board and USB serial link talk properly to Python**, but you **have not wired the physical sensors yet**:

1. Open \`react_mapping/arduino/react_simulated_node.ino\` in the Arduino IDE.
2. Select your board (Arduino Uno / Nano / Mega / ESP32) and COM port, then click **Upload**.
3. **No breadboard or jumper wires required!** The Arduino generates synthetic mock telemetry internally:
   * The vehicle is **static at coordinate (0, 0)**.
   * The vehicle **rotates in place standing at a single point** around its vertical Z-axis (default ~18°/s, completing a 360° turn in 20 seconds).
   * Real-time geometric ray-tracing computes virtual distances from all 4 ultrasonic transducers to virtual walls and an obstacle pillar.
   * Realistic 6-DOF IMU acceleration, gyroscope rates, and 3-axis magnetometer readings with XOR checksums are streamed at 15 Hz over 115200 baud.
4. Set in \`config.py\`:
   \`\`\`python
   SIMULATION_MODE = False
   SERIAL_PORT = "COM3"   # Your Arduino USB port
   BAUD_RATE = 115200
   \`\`\`
5. Run \`python main.py\`. You will see the Python Ground Control Station receive real USB serial packets from the Arduino, track the rover rotating in place, and paint a 360° top-down 2D map!
6. **Interactive Controls via Serial**:
   * Press **Space** or **\`p\`** to pause/resume rotation.
   * Press **\`c\`** to change clockwise/counter-clockwise direction.
   * Press **\`+\`** / **\`-\`** to speed up or slow down.
   * Press **\`m\`** to switch between 360° spin and oscillating sweep (±90°).`
  },
  {
    id: "hardware_switch",
    title: "8. Switching from Simulation to Real Hardware",
    badge: "Deployment",
    summary: "Uploading Arduino sketch, setting COM port, and launching live serial acquisition.",
    content: `### Steps to Deploy on Real Rover:

1. **Connect Arduino Uno**:
   Plug your Arduino Uno into your computer via USB.

2. **Upload Arduino Code**:
   * Open the Arduino IDE.
   * Open \`react_mapping/arduino/react_sensor_node.ino\`.
   * Under **Tools > Board**, select **Arduino Uno**.
   * Under **Tools > Port**, select your detected COM port.
   * Click **Upload**.
   * Ensure Arduino Serial Monitor is **CLOSED** (otherwise Python cannot open the port).

3. **Configure Python**:
   Open \`react_mapping/config.py\` and change:
   \`\`\`python
   SIMULATION_MODE = False
   SERIAL_PORT = "COM3"   # Set to your Arduino COM port
   BAUD_RATE = 115200
   \`\`\`

4. **Run Ground Control Station**:
   \`\`\`cmd
   python main.py
   \`\`\`
   The application will connect to the Arduino, display **"Streaming hardware data (OK)"**, and start mapping your real room or test maze.`
  },
  {
    id: "calibration",
    title: "9. Sensor Calibration Procedures",
    badge: "Calibration",
    summary: "Stationary zero-rate bias calculation for Gyroscope and 360-degree hard-iron ellipse fitting for Magnetometer.",
    content: `### 1. Gyroscope Zero-Rate Bias Calibration
Even high-quality MEMS gyroscopes output a small non-zero reading when completely stationary (e.g. $+1.2^\\circ/\\text{s}$). Integrating this error causes orientation to drift indefinitely.

**Procedure:**
1. Place the rover on a completely flat, stationary surface.
2. Collect 200 consecutive readings over 4 seconds.
3. Compute the mean drift offset:
   $$\\text{Bias}_{\\text{gyro}} = \\frac{1}{N} \\sum_{i=1}^N \\omega_{z,i}$$
4. Enter these values into \`config.py\`:
   \`\`\`python
   GYRO_BIAS_Z = 1.25  # deg/s
   \`\`\`

---

### 2. Magnetometer Hard-Iron Calibration
Underground coal mines contain iron support arches, conveyor belts, and steel bolts. Furthermore, the rover's motors, batteries, and chassis create constant parasitic magnetic offsets known as **hard-iron distortion**.

Hard-iron shifts the center of the magnetic circle from $(0, 0)$ to $(X_{\\text{offset}}, Y_{\\text{offset}})$.

**Procedure:**
1. Rotate the rover slowly through two full $360^\\circ$ circles.
2. Record the minimum and maximum values:
   $$X_{\\text{offset}} = \\frac{X_{\\text{max}} + X_{\\text{min}}}{2}$$
   $$Y_{\\text{offset}} = \\frac{Y_{\\text{max}} + Y_{\\text{min}}}{2}$$
3. Corrected magnetometer reading:
   $$M_{x,\\text{clean}} = M_{x,\\text{raw}} - X_{\\text{offset}}$$
   $$M_{y,\\text{clean}} = M_{y,\\text{raw}} - Y_{\\text{offset}}$$`
  },
  {
    id: "coordinate_math",
    title: "10. Coordinate Transformation Mathematics",
    badge: "Kinematics",
    summary: "SE(2) rigid-body kinematic projections transforming sensor polar coordinates into global map coordinates.",
    content: `### Coordinate Conventions
* **Global Map Frame $\\{W\\}$**:
  * $+X_{\\text{global}}$ = Global East (Initial forward heading direction at startup)
  * $+Y_{\\text{global}}$ = Global North (Initial left direction)
  * Origin $(0, 0)$ is fixed at rover startup position.

* **Rover Body Frame $\\{B\\}$**:
  * $+X_{\\text{rover}}$ = Forward along vehicle longitudinal centerline.
  * $+Y_{\\text{rover}}$ = Left along vehicle lateral axle.
  * Orientation angle $\\theta$ is measured counter-clockwise from $+X_{\\text{global}}$.

### 2D Kinematic Projection Steps

#### Step 1: Transducer to Rover Body Frame
Each sensor is mounted at physical offset $(o_x, o_y)$ relative to the rover center and rotated at orientation angle $\\alpha$:
* **Front HC-SR04**: $o_x = +0.15\\text{ m}, o_y = 0.00\\text{ m}, \\alpha = 0^\\circ$ (facing forward centerline)
* **Left HC-SR04**: $o_x = 0.00\\text{ m}, o_y = +0.10\\text{ m}, \\alpha = +90^\\circ$ (perpendicular to left flank)
* **Right HC-SR04**: $o_x = 0.00\\text{ m}, o_y = -0.10\\text{ m}, \\alpha = -90^\\circ$ (perpendicular to right flank)

For measured distance $d$:
$$x_{\\text{rover}} = o_x + d \\cdot \\cos(\\alpha)$$
$$y_{\\text{rover}} = o_y + d \\cdot \\sin(\\alpha)$$

#### Step 2: Rover Body to Global World Frame
Using homogeneous transformation matrix in $SE(2)$:
$$\\begin{bmatrix} X_{\\text{global}} \\\\ Y_{\\text{global}} \\\\ 1 \\end{bmatrix} = \\begin{bmatrix} \\cos(\\theta) & -\\sin(\\theta) & X_{\\text{rover}} \\\\ \\sin(\\theta) & \\cos(\\theta) & Y_{\\text{rover}} \\\\ 0 & 0 & 1 \\end{bmatrix} \\begin{bmatrix} x_{\\text{rover}} \\\\ y_{\\text{rover}} \\\\ 1 \\end{bmatrix}$$

Expanded algebraic equations:
$$X_{\\text{global}} = X_{\\text{rover}} + x_{\\text{rover}} \\cos(\\theta) - y_{\\text{rover}} \\sin(\\theta)$$
$$Y_{\\text{global}} = Y_{\\text{rover}} + x_{\\text{rover}} \\sin(\\theta) + y_{\\text{rover}} \\cos(\\theta)$$`
  },
  {
    id: "mapping_algo",
    title: "11. 2D Occupancy Grid Generation & Ray Tracing",
    badge: "Mapping",
    summary: "Bresenham integer ray traversal, Bayesian log-odds occupancy updates, and noise suppression.",
    content: `### Bayesian Log-Odds Occupancy Grid
The underground environment is discretized into a 2D matrix of square cells of size $\\Delta = 0.05\\text{ m}$ ($5\\text{ cm}$).

Each cell $m_i$ stores the log-odds representation of its occupancy probability:
$$L(m_i) = \\log \\left( \\frac{P(m_i = \\text{occupied})}{1 - P(m_i = \\text{occupied})} \\right)$$

* When unvisited: $P(m_i) = 0.5 \\implies L(m_i) = 0.0$
* Along ultrasonic ray: Decrement by $l_{\\text{free}} = -0.4$ (cell is carved as free space)
* At detected obstacle endpoint: Increment by $l_{\\text{occupied}} = +0.85$ (cell is obstacle)

### Bresenham's Integer Line Algorithm
To avoid floating-point overhead, ray cells from the sensor chassis mount $(c_0, r_0)$ to the obstacle hit point $(c_1, r_1)$ are traversed using Bresenham's line algorithm:
\`\`\`text
Rover Transducer ───[ FREE ]───[ FREE ]───[ FREE ]───► [ OCCUPIED ]
\`\`\`

Conversion back to occupancy probability for rendering:
$$P(m_i) = 1.0 - \\frac{1.0}{1.0 + e^{L(m_i)}}$$`
  },
  {
    id: "heading_math",
    title: "12. Heading Estimation & Sensor Fusion",
    badge: "Sensor Fusion",
    summary: "Tilt compensation via accelerometer and Complementary filter combining Gyro dynamic rate and Magnetometer reference.",
    content: `### 1. Accelerometer Tilt Angles (Pitch & Roll)
When the rover climbs over loose rocks or coal dust, the vehicle is pitched or rolled. Accelerometer resolves the gravity vector:
$$\\text{roll} (\\phi) = \\text{atan2}(a_y, a_z)$$
$$\\text{pitch} (\\theta_p) = \\text{atan2}(-a_x, \\sqrt{a_y^2 + a_z^2})$$

### 2. Tilt-Compensated Magnetometer Yaw
The magnetic field is rotated into the horizontal ground plane:
$$X_h = M_x \\cos(\\theta_p) + M_z \\sin(\\theta_p)$$
$$Y_h = M_x \\sin(\\phi)\\sin(\\theta_p) + M_y \\cos(\\phi) - M_z \\sin(\\phi)\\cos(\\theta_p)$$
$$\\psi_{\\text{mag}} = \\text{atan2}(Y_h, X_h) + \\delta_{\\text{declination}}$$

### 3. Complementary Filter Fusion
Gyroscope integration provides rapid, jitter-free angle changes $\\omega_z \\Delta t$, but drifts over minutes. Magnetometer provides an absolute heading reference with high-frequency noise:
$$\\theta_{t} = \\text{normalize} \\left( \\theta_{\\text{gyro}} + (1 - \\alpha) \\cdot \\text{angle\\_diff}(\\psi_{\\text{mag}}, \\theta_{\\text{gyro}}) \\right)$$
With $\\alpha = 0.96$, 96% of short-term orientation comes from gyroscope integration, while 4% corrects drift toward the compass heading every frame.`
  },
  {
    id: "limitations",
    title: "13. Engineering Limitations & SLAM Roadmap",
    badge: "Robotics Theory",
    summary: "Honest evaluation of dead-reckoning drift, ultrasonic beam dispersion, and future upgrade path to wheel encoders, LiDAR, and EKF SLAM.",
    content: `### Important Engineering Disclaimer: This is NOT True SLAM
This system is an **Ultrasonic Occupancy Mapping and IMU Dead-Reckoning System**. In robotic literature, **SLAM (Simultaneous Localization and Mapping)** requires using observed environmental features to *correct* the robot's estimated position (closing loops).

### Physical Limitations of Current Setup:
1. **Ultrasonic Beam Spread ($\approx 15^\\circ$ cone)**:
   HC-SR04 transducers emit a wide acoustic cone. Any obstacle inside the $15^\\circ$ wedge triggers an echo return at the closest point, creating curved arc artifacts rather than razor-sharp wall lines.
2. **Specular Acoustic Reflection**:
   If an ultrasonic pulse hits a smooth mine wall at an angle greater than $45^\\circ$, sound bounces away instead of returning to the receiver (blind condition).
3. **Dead-Reckoning Position Drift**:
   Without wheel encoders or visual odometry, estimating position from velocity commands will slowly drift when wheels slip in loose coal debris.

### Modular Upgrade Roadmap (Zero Rewrite Needed!):
* **Step 1: Add Rotary Wheel Encoders**:
  Hook up encoder pins and call \`odometry.update_from_encoders(left_ticks, right_ticks, heading, dt)\`.
* **Step 2: Replace HC-SR04 with 2D LiDAR (RPLiDAR A1/A2)**:
  Replace the 4-sensor ray loop in \`main.py\` with 360-degree range rays; the \`mapping.py\` and \`coordinate_transform.py\` modules already accept arbitrary rays!
* **Step 3: Add Extended Kalman Filter (EKF) or FastSLAM**:
  Fuse encoder odometry, IMU, and scan-matching.`
  },
  {
    id: "troubleshooting",
    title: "14. Comprehensive Troubleshooting Guide",
    badge: "Support",
    summary: "Fast solutions for blank serial monitor, port lockouts, checksum errors, and I2C freezes.",
    content: `### Symptom 0: Blank Arduino Serial Monitor (No Output Appearing)
* **Cause 1 (Most Common - 90% of cases)**: **Baud Rate Mismatch**. Arduino IDE Serial Monitor defaults to **9600 baud**, but both \`react_sensor_node.ino\` and \`react_simulated_node.ino\` run at **115200 baud**.
  * **Fix**: In the bottom-right corner (or top-right in Arduino IDE 2.x) of the Serial Monitor tab, change the baud rate dropdown from \`9600 baud\` to **\`115200 baud\`**.
* **Cause 2: I2C Bus Freeze on \`Wire.endTransmission\`**. If GY-271 is not plugged into A4/A5 or has poor contact, AVR I2C hardware can lock up in \`setup()\`.
  * **Fix**: The latest firmware includes auto-timeout protection (\`Wire.setWireTimeout\`) and internal pull-ups on A4/A5. Re-flash the updated \`react_sensor_node.ino\` or \`react_simulated_node.ino\`.
* **Cause 3: COM Port Contention with Python**.
  * **Fix**: If \`python main.py\` is running in the background, close Python first. A COM port can only be accessed by one application at a time.
* **Cause 4: Physical Board Heartbeat Check**.
  * **Fix**: Check the yellow built-in LED on Pin 13 of the Arduino. In the updated firmware, the LED blinks continuously at 15 Hz to confirm active transmission.

### Symptom 1: \`serial.serialutil.SerialException: could not open port 'COM3': PermissionError\`
* **Cause**: Another program (such as the Arduino Serial Monitor or Cura/Slicer) is currently holding the COM port open.
* **Fix**: Close Arduino Serial Monitor, Arduino IDE, or any other serial terminal, then re-run \`python main.py\`.

### Symptom 2: GUI displays \`WARNING: Serial data timeout\`
* **Cause**: Python opened the port, but Arduino has stopped transmitting packets.
* **Fix**:
  1. Check baud rate matches: Both \`react_sensor_node.ino\` and \`config.py\` must use \`115200\`.
  2. Press the physical **RESET** button on the Arduino Uno.
  3. Verify the I2C bus hasn't frozen due to a loose wire on A4 (SDA) or A5 (SCL).

### Symptom 3: Ultrasonic readings randomly drop to 400 cm or read zero
* **Cause**: Ultrasonic acoustic crosstalk or missing echo pulse.
* **Fix**: The firmware already has a 500 µs guard delay between triggering adjacent sensors. Ensure power supply delivers solid 5V with adequate current (4 HC-SR04 sensors consume ~60 mA).

### Symptom 4: Heading spins continuously or points in wrong direction
* **Cause**: Magnetometer experiencing severe magnetic interference from motors or steel workbench.
* **Fix**: Mount the MPU-6050 and magnetometer on an elevated nylon/plastic standoff at least 10 cm above DC drive motors and battery packs.`
  }
];
