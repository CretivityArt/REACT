/*
 ==============================================================================
 REACT (Reactive Hazard Exploration/Response Rover)
 Simulated Sensor Node Firmware: react_simulated_node.ino
 Target Board: Arduino Uno / Nano / Mega / ESP32
 ==============================================================================
 PURPOSE:
 This sketch flashes directly to an Arduino board to test the complete Python
 Ground Control Station (GCS) and 2D mapping pipeline WITHOUT requiring ANY
 physical sensors to be connected.

 SENSOR LAYOUT:
 1. FRONT Ultrasonic : Mounted at front center, pointing straight forward (0°)
 2. LEFT Ultrasonic  : Mounted at left flank, pointing perpendicular to left (+90°)
 3. RIGHT Ultrasonic : Mounted at right flank, pointing perpendicular to right (-90°)
 4. GY-271 Compass   : Simulates 3-axis magnetometer & direct digital heading [0, 360)°

 SIMULATION BEHAVIOR:
 1. Stationarity: The rover vehicle is STATIC at fixed coordinate (X=0.0m, Y=0.0m).
 2. Rotation: The rover ROTATES IN PLACE standing at a single point around its
    vertical Z-axis at a configurable rate (default +18.0 deg/s -> 360° in 20s).
 3. Ray-Casting: Calculates geometric distance from the 3 transducers
    (FRONT=0°, LEFT=+90°, RIGHT=-90°) to virtual room walls and pillars.
 4. Output: Formats valid ASCII telemetry packets with XOR checksum identical to
    the real sensor node at 15 Hz (115200 baud).

 INTERACTIVE SERIAL COMMANDS (via Arduino Serial Monitor or Python):
  ' ' or 'p' -> Pause / Resume rotation
  'c'        -> Toggle rotation direction (Clockwise <-> Counter-Clockwise)
  '+'        -> Increase rotation speed (+5 deg/s)
  '-'        -> Decrease rotation speed (-5 deg/s)
  'r'        -> Reset heading to 0°
  'm'        -> Toggle Mode (Continuous 360° spin <-> Oscillating sweep +-90°)
  '?' or 'h' -> Print simulation status & help
 ==============================================================================
*/

#include <math.h>

// ----------------------------------------------------------------------------
// CONFIGURATION CONSTANTS
// ----------------------------------------------------------------------------
const unsigned long PACKET_INTERVAL_MS = 66; // ~15 Hz telemetry output rate
const long SERIAL_BAUD = 115200;

// Virtual Room Boundaries (meters)
const float ROOM_X_MAX = 2.5;  // East wall at +2.5m
const float ROOM_X_MIN = -2.5; // West wall at -2.5m
const float ROOM_Y_MAX = 2.0;  // North wall at +2.0m
const float ROOM_Y_MIN = -2.0; // South wall at -2.0m

// Virtual Obstacle Pillar (meters)
const float PILLAR_X = 1.0;
const float PILLAR_Y = 0.8;
const float PILLAR_RADIUS = 0.35; // 35cm radius cylindrical column

// Transducer mount offsets on chassis (matching config.py)
// 1. FRONT (0 deg): Mounted at front bumper, centerline
const float SENSOR_FRONT_OX = 0.175;
const float SENSOR_FRONT_OY = 0.000;
const float SENSOR_FRONT_ANGLE_RAD = 0.0; // 0 deg straight ahead

// 2. LEFT (+90 deg perpendicular): Mounted at left flank, vehicle center X
const float SENSOR_LEFT_OX = 0.000;
const float SENSOR_LEFT_OY = 0.125;
const float SENSOR_LEFT_ANGLE_RAD = 1.570796; // +90 deg (+PI/2)

// 3. RIGHT (-90 deg perpendicular): Mounted at right flank, vehicle center X
const float SENSOR_RIGHT_OX = 0.000;
const float SENSOR_RIGHT_OY = -0.125;
const float SENSOR_RIGHT_ANGLE_RAD = -1.570796; // -90 deg (-PI/2)

// ----------------------------------------------------------------------------
// SIMULATION STATE VARIABLES
// ----------------------------------------------------------------------------
unsigned long lastPacketTime = 0;
unsigned long lastTickTime = 0;
unsigned long packetCounter = 0;

// Rotation in place
float currentHeadingRad = 0.0;       // Radians (0 to 2*PI)
float rotationSpeedDegPerSec = 18.0; // Default: 18 deg/sec (20s per full rotation)
bool rotationPaused = false;
bool rotateClockwise = false;        // false = CCW (+Z), true = CW (-Z)
bool oscillateMode = false;          // true = sweep +-90 degrees back and forth
float sweepDirection = 1.0;

// Earth magnetic field constants (uT)
const float MAG_HORIZ = 32.0; // Horizontal field strength
const float MAG_VERT = 38.0;  // Vertical field strength

// ----------------------------------------------------------------------------
// HELPER: Pseudo-Random Jitter Generator (-range to +range)
// ----------------------------------------------------------------------------
float getNoise(float range) {
  int r = random(-1000, 1000);
  return ((float)r / 1000.0) * range;
}

// ----------------------------------------------------------------------------
// 2D RAY-CASTING ENGINE (Intersection with Walls and Cylinders)
// ----------------------------------------------------------------------------
float rayBoxIntersect(float ox, float oy, float dirX, float dirY,
                      float minX, float maxX, float minY, float maxY) {
  float tMin = 0.03;  // Minimum HC-SR04 blind zone (3 cm)
  float tMax = 4.00;  // Maximum reliable range (4 meters)

  // Test X bounds
  if (fabs(dirX) > 1e-6) {
    float t1 = (minX - ox) / dirX;
    float t2 = (maxX - ox) / dirX;
    if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
    if (t1 > tMin) tMin = t1;
    if (t2 < tMax) tMax = t2;
    if (tMin > tMax) return 4.0;
  } else {
    if (ox < minX || ox > maxX) return 4.0;
  }

  // Test Y bounds
  if (fabs(dirY) > 1e-6) {
    float t1 = (minY - oy) / dirY;
    float t2 = (maxY - oy) / dirY;
    if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
    if (t1 > tMin) tMin = t1;
    if (t2 < tMax) tMax = t2;
    if (tMin > tMax) return 4.0;
  } else {
    if (oy < minY || oy > maxY) return 4.0;
  }

  return (tMax > 0.03 && tMax <= 4.00) ? tMax : 4.0;
}

float rayCircleIntersect(float ox, float oy, float dirX, float dirY,
                         float cx, float cy, float radius) {
  float dx = ox - cx;
  float dy = oy - cy;
  float b = 2.0 * (dx * dirX + dy * dirY);
  float c = dx * dx + dy * dy - radius * radius;
  float disc = b * b - 4.0 * c;

  if (disc < 0.0) return 4.0; // No intersection

  float sqrtDisc = sqrt(disc);
  float t1 = (-b - sqrtDisc) / 2.0;
  float t2 = (-b + sqrtDisc) / 2.0;

  if (t1 > 0.03 && t1 <= 4.0) return t1;
  if (t2 > 0.03 && t2 <= 4.0) return t2;
  return 4.0;
}

float castUltrasonicRay(float originX, float originY, float rayAngleRad) {
  float dirX = cos(rayAngleRad);
  float dirY = sin(rayAngleRad);

  float distWall = rayBoxIntersect(originX, originY, dirX, dirY,
                                   ROOM_X_MIN, ROOM_X_MAX, ROOM_Y_MIN, ROOM_Y_MAX);
  float distPillar = rayCircleIntersect(originX, originY, dirX, dirY,
                                        PILLAR_X, PILLAR_Y, PILLAR_RADIUS);

  float closest = distWall;
  if (distPillar < closest) closest = distPillar;
  return closest;
}

// ----------------------------------------------------------------------------
// INTERACTIVE COMMAND PARSER
// ----------------------------------------------------------------------------
void processSerialInput() {
  while (Serial.available() > 0) {
    char cmd = (char)Serial.read();

    if (cmd == ' ' || cmd == 'p' || cmd == 'P') {
      rotationPaused = !rotationPaused;
      Serial.print(F("#MSG,Rotation "));
      Serial.println(rotationPaused ? F("PAUSED") : F("RESUMED"));
    } else if (cmd == 'c' || cmd == 'C') {
      rotateClockwise = !rotateClockwise;
      Serial.print(F("#MSG,Direction set to: "));
      Serial.println(rotateClockwise ? F("CLOCKWISE") : F("COUNTER-CLOCKWISE"));
    } else if (cmd == '+') {
      rotationSpeedDegPerSec += 5.0;
      if (rotationSpeedDegPerSec > 90.0) rotationSpeedDegPerSec = 90.0;
      Serial.print(F("#MSG,Rotation Speed: "));
      Serial.print(rotationSpeedDegPerSec);
      Serial.println(F(" deg/s"));
    } else if (cmd == '-') {
      rotationSpeedDegPerSec -= 5.0;
      if (rotationSpeedDegPerSec < 2.0) rotationSpeedDegPerSec = 2.0;
      Serial.print(F("#MSG,Rotation Speed: "));
      Serial.print(rotationSpeedDegPerSec);
      Serial.println(F(" deg/s"));
    } else if (cmd == 'r' || cmd == 'R') {
      currentHeadingRad = 0.0;
      Serial.println(F("#MSG,Heading Reset to 0.0 deg"));
    } else if (cmd == 'm' || cmd == 'M') {
      oscillateMode = !oscillateMode;
      Serial.print(F("#MSG,Mode: "));
      Serial.println(oscillateMode ? F("OSCILLATING (+-90 deg)") : F("CONTINUOUS 360"));
    } else if (cmd == '?' || cmd == 'h' || cmd == 'H') {
      Serial.println(F("#MSG,Commands: [p]=Pause [c]=Dir [+/-]=Speed [r]=Reset [m]=Mode"));
    }
  }
}

// ----------------------------------------------------------------------------
// SETUP
// ----------------------------------------------------------------------------
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(SERIAL_BAUD);
  delay(250);

  randomSeed(analogRead(A0) + analogRead(A1));

  Serial.println();
  Serial.println(F("=================================================="));
  Serial.println(F("REACT ROVER // SIMULATED SENSOR NODE READY"));
  Serial.println(F("BAUD RATE: 115200 (Set Serial Monitor to 115200)"));
  Serial.println(F("=================================================="));
  Serial.println(F("#0,STATUS=REACT_SIMULATOR_READY,SENSORS=FRONT_LEFT_RIGHT_GY271*00"));
  Serial.println(F("#INFO,Rover is STATIC at (0,0), ROTATING in place at 18 deg/s."));
  Serial.println(F("#INFO,Sensors: FRONT (0°), LEFT (+90° Perp), RIGHT (-90° Perp)"));
  Serial.println(F("#INFO,GY-271 Compass: Streaming absolute heading [0, 360)°"));
  Serial.println(F("#INFO,Commands: 'p'=Pause, 'c'=Reverse dir, 'r'=Reset heading"));
  Serial.println(F("=================================================="));

  lastPacketTime = millis();
  lastTickTime = millis();
  digitalWrite(LED_BUILTIN, LOW);
}

// ----------------------------------------------------------------------------
// MAIN EXECUTION LOOP
// ----------------------------------------------------------------------------
void loop() {
  unsigned long now = millis();

  // 1. Process any incoming interactive characters from Python or Serial Monitor
  processSerialInput();

  // 2. High-precision kinematics integration for rotation in place
  float dt = (float)(now - lastTickTime) / 1000.0;
  lastTickTime = now;

  float currentGyroZ_dps = 0.0;

  if (!rotationPaused) {
    float rateDps = rotationSpeedDegPerSec;

    if (oscillateMode) {
      // Oscillate between -90 and +90 degrees (-1.57 and +1.57 rad)
      float currentDeg = currentHeadingRad * (180.0 / M_PI);
      if (currentDeg > 90.0) sweepDirection = -1.0;
      if (currentDeg < -90.0) sweepDirection = 1.0;

      float dHeading = sweepDirection * rateDps * dt * (M_PI / 180.0);
      currentHeadingRad += dHeading;
      currentGyroZ_dps = sweepDirection * rateDps;
    } else {
      // Continuous 360 degree rotation
      float dirSign = rotateClockwise ? -1.0 : 1.0;
      float dHeading = dirSign * rateDps * dt * (M_PI / 180.0);
      currentHeadingRad += dHeading;

      // Wrap angle within [0, 2*PI)
      while (currentHeadingRad >= 2.0 * M_PI) currentHeadingRad -= 2.0 * M_PI;
      while (currentHeadingRad < 0.0) currentHeadingRad += 2.0 * M_PI;

      currentGyroZ_dps = dirSign * rateDps;
    }
  }

  // 3. Emit structured packet at ~15 Hz
  if (now - lastPacketTime >= PACKET_INTERVAL_MS) {
    lastPacketTime = now;
    packetCounter++;

    // Compute chassis orientation trigonometric factors
    float cosH = cos(currentHeadingRad);
    float sinH = sin(currentHeadingRad);

    // --- ULTRASONIC RANGE COMPUTATION (Geometric ray tracing) ---
    // Vehicle is static at (0.0, 0.0), but sensor positions rotate with chassis:
    // 1. FRONT (0 deg straight forward)
    float frontOrigX = SENSOR_FRONT_OX * cosH - SENSOR_FRONT_OY * sinH;
    float frontOrigY = SENSOR_FRONT_OX * sinH + SENSOR_FRONT_OY * cosH;
    float frontRayAngle = currentHeadingRad + SENSOR_FRONT_ANGLE_RAD;
    float distFront_m = castUltrasonicRay(frontOrigX, frontOrigY, frontRayAngle) + getNoise(0.015);

    // 2. LEFT (+90 deg perpendicular left)
    float leftOrigX = SENSOR_LEFT_OX * cosH - SENSOR_LEFT_OY * sinH;
    float leftOrigY = SENSOR_LEFT_OX * sinH + SENSOR_LEFT_OY * cosH;
    float leftRayAngle = currentHeadingRad + SENSOR_LEFT_ANGLE_RAD;
    float distLeft_m = castUltrasonicRay(leftOrigX, leftOrigY, leftRayAngle) + getNoise(0.015);

    // 3. RIGHT (-90 deg perpendicular right)
    float rightOrigX = SENSOR_RIGHT_OX * cosH - SENSOR_RIGHT_OY * sinH;
    float rightOrigY = SENSOR_RIGHT_OX * sinH + SENSOR_RIGHT_OY * cosH;
    float rightRayAngle = currentHeadingRad + SENSOR_RIGHT_ANGLE_RAD;
    float distRight_m = castUltrasonicRay(rightOrigX, rightOrigY, rightRayAngle) + getNoise(0.015);

    // Convert to centimeters and clamp to sensor limits (3cm to 400cm)
    float distFront_cm = constrain(distFront_m * 100.0, 3.0, 400.0);
    float distLeft_cm  = constrain(distLeft_m * 100.0, 3.0, 400.0);
    float distRight_cm = constrain(distRight_m * 100.0, 3.0, 400.0);

    // --- GY-271 DIGITAL COMPASS HEADING & MAGNETOMETER DATA ---
    float headingDeg = (currentHeadingRad * 180.0 / M_PI);
    while (headingDeg >= 360.0) headingDeg -= 360.0;
    while (headingDeg < 0.0) headingDeg += 360.0;

    // Projected magnetic field components in horizontal plane (uT)
    float mx = (MAG_HORIZ * cosH) + getNoise(0.4);
    float my = (MAG_HORIZ * sinH) + getNoise(0.4);
    float mz = MAG_VERT + getNoise(0.3);

    // Optional IMU components
    float ax = getNoise(0.02);
    float ay = getNoise(0.02);
    float az = 1.00 + getNoise(0.02);
    float gx = getNoise(0.2);
    float gy = getNoise(0.2);
    float gz = currentGyroZ_dps + getNoise(0.3);

    // --- TRANSMIT FORMATTED PACKET WITH XOR CHECKSUM ---
    transmitPacket(distFront_cm, distLeft_cm, distRight_cm, headingDeg,
                   mx, my, mz, ax, ay, az, gx, gy, gz);
  }
}

// ----------------------------------------------------------------------------
// PACKET TRANSMISSION
// ----------------------------------------------------------------------------
void transmitPacket(float dFront, float dLeft, float dRight, float hdg,
                    float mx, float my, float mz,
                    float ax, float ay, float az,
                    float gx, float gy, float gz) {
  char sDF[10], sDL[10], sDR[10], sHDG[10];
  char sMX[8],  sMY[8],  sMZ[8];
  char sAX[8],  sAY[8],  sAZ[8];
  char sGX[8],  sGY[8],  sGZ[8];

  dtostrf(dFront, 0, 1, sDF);
  dtostrf(dLeft,  0, 1, sDL);
  dtostrf(dRight, 0, 1, sDR);
  dtostrf(hdg,    0, 1, sHDG);

  dtostrf(mx, 0, 1, sMX);
  dtostrf(my, 0, 1, sMY);
  dtostrf(mz, 0, 1, sMZ);

  dtostrf(ax, 0, 2, sAX);
  dtostrf(ay, 0, 2, sAY);
  dtostrf(az, 0, 2, sAZ);

  dtostrf(gx, 0, 1, sGX);
  dtostrf(gy, 0, 1, sGY);
  dtostrf(gz, 0, 1, sGZ);

  char buffer[240];
  // Telemetry format supporting DF, DL, DR, GY-271 HDG, and MPU-6050 6-DOF:
  int len = snprintf(buffer, sizeof(buffer),
    "#%lu,DF=%s,DL=%s,DR=%s,HDG=%s,AX=%s,AY=%s,AZ=%s,GX=%s,GY=%s,GZ=%s,MX=%s,MY=%s,MZ=%s,D1=%s,D2=%s,D3=%s",
    packetCounter,
    sDF, sDL, sDR, sHDG,
    sAX, sAY, sAZ,
    sGX, sGY, sGZ,
    sMX, sMY, sMZ,
    sDF, sDL, sDR
  );

  // Compute 8-bit XOR checksum
  byte checksum = 0;
  for (int i = 0; i < len; i++) {
    checksum ^= (byte)buffer[i];
  }

  // Print over serial
  Serial.print(buffer);
  Serial.print('*');
  if (checksum < 0x10) Serial.print('0');
  Serial.println(checksum, HEX);

  // Toggle built-in LED (Pin 13) to indicate heartbeat
  digitalWrite(LED_BUILTIN, (packetCounter % 2 == 0) ? HIGH : LOW);
}
