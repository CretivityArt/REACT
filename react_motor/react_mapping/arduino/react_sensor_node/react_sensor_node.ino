/*
  ============================================================
                 R.E.A.C.T. ROVER
       Arduino UNO / Nano Sensor + Motor Firmware
  ============================================================

  Features:
  - L298N dual motor control
  - 3x HC-SR04 ultrasonic sensors
  - MPU6050 / MPU6500 accelerometer
  - MPU6050 / MPU6500 Z-axis gyro
  - Relative heading from gyro integration
  - Gyro startup calibration
  - Motor watchdog
  - Serial motor commands
  - Clean DATA telemetry for Python

  Serial:
  Baud rate = 9600

  Telemetry format:

  DATA:#<seq>,DF=<cm>,DL=<cm>,DR=<cm>,
  HDG=<deg>,GZ=<dps>,AX=<g>,AY=<g>,AZ=<g>*

  IMPORTANT:
  MPU6050/MPU6500 does NOT contain a magnetometer.
  HDG is therefore relative gyro-integrated heading,
  NOT magnetic North heading.

  ============================================================
*/

#include <Wire.h>
#include <stdlib.h>
#include <string.h>

// ============================================================
// MPU
// ============================================================

#define MPU_ADDR 0x68

// MPU6050 registers
#define MPU_PWR_MGMT_1 0x6B
#define MPU_GYRO_ZOUT_H 0x47
#define MPU_ACCEL_XOUT_H 0x3B

// ============================================================
// MOTOR PINS - L298N
// ============================================================

#define ENA 9
#define IN1 8
#define IN2 10

#define ENB 11
#define IN3 12
#define IN4 A0

// ============================================================
// ULTRASONIC PINS
// ============================================================

// Front
#define FRONT_TRIG 2
#define FRONT_ECHO 3

// Left
#define LEFT_TRIG 4
#define LEFT_ECHO 5

// Right
#define RIGHT_TRIG 6
#define RIGHT_ECHO 7

// ============================================================
// OTHER
// ============================================================

#define STATUS_LED 13

// ============================================================
// MOTOR SETTINGS
// ============================================================

int currentSpeed = 150;
int turnSpeed = 140;

// Change these if your motors are physically reversed
bool invertLeftMotor = false;
bool invertRightMotor = false;

// ============================================================
// WATCHDOG
// ============================================================

const unsigned long COMMAND_TIMEOUT = 2000;

unsigned long lastCommandTime = 0;

// ============================================================
// ULTRASONIC SETTINGS
// ============================================================

const unsigned long ULTRASONIC_TIMEOUT = 30000;

float distFront = 0.0;
float distLeft = 0.0;
float distRight = 0.0;

// ============================================================
// IMU VARIABLES
// ============================================================

float gyroZOffset = 0.0;

float gyroZ = 0.0;

float angleZ = 0.0;

float ax = 0.0;
float ay = 0.0;
float az = 0.0;

unsigned long previousTime = 0;

// ============================================================
// TELEMETRY
// ============================================================

unsigned long packetCounter = 0;

unsigned long lastTelemetryTime = 0;

const unsigned long TELEMETRY_INTERVAL = 100;

// 100 ms = approximately 10 Hz

// ============================================================
// SERIAL COMMAND BUFFER
// ============================================================

const int SERIAL_BUFFER_SIZE = 50;

char serialCmdBuffer[SERIAL_BUFFER_SIZE];

int serialCmdIndex = 0;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void setupMPU();

int16_t readGyroZRaw();

void readAccelerometer();

void calibrateGyro();

void updateHeading();

float measureDistance(int trigPin, int echoPin);

void readUltrasonicSensors();

void transmitTelemetry();

void processCommand(char command);

void processStringCommand(char *command);

void readSerialCommands();

void stopMotors();

void moveForward();

void moveBackward();

void turnLeft();

void turnRight();

void setLeftMotor(int speedValue);

void setRightMotor(int speedValue);

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(9600);

  Wire.begin();

  // ----------------------------------------------------------
  // Motor pins
  // ----------------------------------------------------------

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // ----------------------------------------------------------
  // Ultrasonic pins
  // ----------------------------------------------------------

  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);

  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(LEFT_ECHO, INPUT);

  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);

  // ----------------------------------------------------------
  // LED
  // ----------------------------------------------------------

  pinMode(STATUS_LED, OUTPUT);

  digitalWrite(STATUS_LED, LOW);

  // ----------------------------------------------------------
  // Make sure motors are stopped
  // ----------------------------------------------------------

  stopMotors();

  // ----------------------------------------------------------
  // Initialize MPU
  // ----------------------------------------------------------

  setupMPU();

  delay(500);

  // ----------------------------------------------------------
  // Gyro calibration
  // ----------------------------------------------------------

  calibrateGyro();

  angleZ = 0.0;

  previousTime = millis();

  lastCommandTime = millis();

  // ----------------------------------------------------------
  // Startup indication
  // ----------------------------------------------------------

  digitalWrite(STATUS_LED, HIGH);

  delay(300);

  digitalWrite(STATUS_LED, LOW);

  Serial.println(F("REACT READY"));

  Serial.println(F("MPU: 0x68"));

  Serial.println(F("Telemetry: DATA:"));

}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  // ----------------------------------------------------------
  // Read serial commands
  // ----------------------------------------------------------

  readSerialCommands();

  // ----------------------------------------------------------
  // Update gyro heading
  // ----------------------------------------------------------

  updateHeading();

  // ----------------------------------------------------------
  // Read accelerometer
  // ----------------------------------------------------------

  readAccelerometer();

  // ----------------------------------------------------------
  // Read ultrasonic sensors
  // ----------------------------------------------------------

  readUltrasonicSensors();

  // ----------------------------------------------------------
  // Motor watchdog
  // ----------------------------------------------------------

  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {

    stopMotors();

  }

  // ----------------------------------------------------------
  // Telemetry
  // ----------------------------------------------------------

  if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL) {

    lastTelemetryTime = millis();

    transmitTelemetry();

  }

}

// ============================================================
// MPU INITIALIZATION
// ============================================================

void setupMPU() {

  Wire.beginTransmission(MPU_ADDR);

  Wire.write(MPU_PWR_MGMT_1);

  Wire.write(0x00);

  Wire.endTransmission();

  delay(100);

}

// ============================================================
// READ GYRO Z
// ============================================================

int16_t readGyroZRaw() {

  Wire.beginTransmission(MPU_ADDR);

  Wire.write(MPU_GYRO_ZOUT_H);

  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 2, true);

  if (Wire.available() < 2) {

    return 0;

  }

  int16_t gz =

    ((int16_t)Wire.read() << 8) |

    Wire.read();

  return gz;

}

// ============================================================
// GYRO CALIBRATION
// ============================================================

void calibrateGyro() {

  Serial.println(F("Keep MPU still..."));

  delay(500);

  long sum = 0;

  const int samples = 500;

  for (int i = 0; i < samples; i++) {

    int16_t gz = readGyroZRaw();

    sum += gz;

    delay(2);

  }

  gyroZOffset =

    (float)sum / samples;

  Serial.println(F("Gyro calibration complete."));

}

// ============================================================
// UPDATE HEADING
// ============================================================

void updateHeading() {

  int16_t gzRaw = readGyroZRaw();

  // MPU6050 default gyro sensitivity:
  // ±250 deg/s = 131 LSB/(deg/s)

  gyroZ =

    ((float)gzRaw - gyroZOffset) / 131.0;

  unsigned long currentTime = millis();

  float dt =

    (currentTime - previousTime) / 1000.0;

  previousTime = currentTime;

  // Protect against abnormal timing
  if (dt <= 0.0 || dt > 1.0) {

    dt = 0.01;

  }

  // Integrate angular velocity

  angleZ += gyroZ * dt;

  // Normalize to 0-360 degrees

  while (angleZ >= 360.0) {

    angleZ -= 360.0;

  }

  while (angleZ < 0.0) {

    angleZ += 360.0;

  }

}

// ============================================================
// READ ACCELEROMETER
// ============================================================

void readAccelerometer() {

  Wire.beginTransmission(MPU_ADDR);

  Wire.write(MPU_ACCEL_XOUT_H);

  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 6, true);

  if (Wire.available() < 6) {

    return;

  }

  int16_t rawAX =

    ((int16_t)Wire.read() << 8) |

    Wire.read();

  int16_t rawAY =

    ((int16_t)Wire.read() << 8) |

    Wire.read();

  int16_t rawAZ =

    ((int16_t)Wire.read() << 8) |

    Wire.read();

  // Default MPU6050 accelerometer:
  // ±2g = 16384 LSB/g

  ax = rawAX / 16384.0;

  ay = rawAY / 16384.0;

  az = rawAZ / 16384.0;

}

// ============================================================
// HC-SR04 DISTANCE
// ============================================================

float measureDistance(int trigPin, int echoPin) {

  // Make sure trigger starts LOW

  digitalWrite(trigPin, LOW);

  delayMicroseconds(2);

  // 10 us trigger pulse

  digitalWrite(trigPin, HIGH);

  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  // Measure echo

  unsigned long duration =

    pulseIn(

      echoPin,

      HIGH,

      ULTRASONIC_TIMEOUT

    );

  // No echo

  if (duration == 0) {

    return -1.0;

  }

  // Speed of sound:
  // distance cm = time us / 58

  float distance =

    duration / 58.0;

  // Reject impossible readings

  if (distance < 2.0 || distance > 400.0) {

    return -1.0;

  }

  return distance;

}

// ============================================================
// READ ALL THREE ULTRASONIC SENSORS
// ============================================================

void readUltrasonicSensors() {

  distFront =

    measureDistance(

      FRONT_TRIG,

      FRONT_ECHO

    );

  delay(2);

  distLeft =

    measureDistance(

      LEFT_TRIG,

      LEFT_ECHO

    );

  delay(2);

  distRight =

    measureDistance(

      RIGHT_TRIG,

      RIGHT_ECHO

    );

}

// ============================================================
// TELEMETRY
// ============================================================

void transmitTelemetry() {

  /*
    IMPORTANT:

    Do NOT use sprintf/snprintf with %f on Arduino UNO/Nano.

    Direct Serial.print() is used so float values are printed
    correctly.
  */

  Serial.print(F("DATA:#"));

  Serial.print(packetCounter++);

  Serial.print(F(",DF="));

  Serial.print(distFront, 1);

  Serial.print(F(",DL="));

  Serial.print(distLeft, 1);

  Serial.print(F(",DR="));

  Serial.print(distRight, 1);

  Serial.print(F(",HDG="));

  Serial.print(angleZ, 2);

  Serial.print(F(",GZ="));

  Serial.print(gyroZ, 2);

  Serial.print(F(",AX="));

  Serial.print(ax, 2);

  Serial.print(F(",AY="));

  Serial.print(ay, 2);

  Serial.print(F(",AZ="));

  Serial.print(az, 2);

  Serial.println(F("*"));

}

// ============================================================
// MOTOR CONTROL
// ============================================================

// ------------------------------------------------------------
// LEFT MOTOR
// ------------------------------------------------------------

void setLeftMotor(int speedValue) {

  speedValue =

    constrain(speedValue, -255, 255);

  if (invertLeftMotor) {

    speedValue = -speedValue;

  }

  if (speedValue > 0) {

    digitalWrite(IN1, HIGH);

    digitalWrite(IN2, LOW);

    analogWrite(

      ENA,

      speedValue

    );

  }

  else if (speedValue < 0) {

    digitalWrite(IN1, LOW);

    digitalWrite(IN2, HIGH);

    analogWrite(

      ENA,

      -speedValue

    );

  }

  else {

    digitalWrite(IN1, LOW);

    digitalWrite(IN2, LOW);

    analogWrite(ENA, 0);

  }

}

// ------------------------------------------------------------
// RIGHT MOTOR
// ------------------------------------------------------------

void setRightMotor(int speedValue) {

  speedValue =

    constrain(speedValue, -255, 255);

  if (invertRightMotor) {

    speedValue = -speedValue;

  }

  if (speedValue > 0) {

    digitalWrite(IN3, HIGH);

    digitalWrite(IN4, LOW);

    analogWrite(

      ENB,

      speedValue

    );

  }

  else if (speedValue < 0) {

    digitalWrite(IN3, LOW);

    digitalWrite(IN4, HIGH);

    analogWrite(

      ENB,

      -speedValue

    );

  }

  else {

    digitalWrite(IN3, LOW);

    digitalWrite(IN4, LOW);

    analogWrite(ENB, 0);

  }

}

// ============================================================
// FORWARD
// ============================================================

void moveForward() {

  setLeftMotor(currentSpeed);

  setRightMotor(currentSpeed);

}

// ============================================================
// BACKWARD
// ============================================================

void moveBackward() {

  setLeftMotor(-currentSpeed);

  setRightMotor(-currentSpeed);

}

// ============================================================
// LEFT TURN
// ============================================================

void turnLeft() {

  setLeftMotor(-turnSpeed);

  setRightMotor(turnSpeed);

}

// ============================================================
// RIGHT TURN
// ============================================================

void turnRight() {

  setLeftMotor(turnSpeed);

  setRightMotor(-turnSpeed);

}

// ============================================================
// STOP
// ============================================================

void stopMotors() {

  setLeftMotor(0);

  setRightMotor(0);

}

// ============================================================
// SINGLE CHARACTER COMMAND
// ============================================================

void processCommand(char command) {

  switch (command) {

    // --------------------------------------------------------
    // Forward
    // --------------------------------------------------------

    case 'F':
    case 'f':

      moveForward();

      lastCommandTime = millis();

      break;

    // --------------------------------------------------------
    // Backward
    // --------------------------------------------------------

    case 'B':
    case 'b':

      moveBackward();

      lastCommandTime = millis();

      break;

    // --------------------------------------------------------
    // Left
    // --------------------------------------------------------

    case 'L':
    case 'l':

      turnLeft();

      lastCommandTime = millis();

      break;

    // --------------------------------------------------------
    // Right
    // --------------------------------------------------------

    case 'R':
    case 'r':

      turnRight();

      lastCommandTime = millis();

      break;

    // --------------------------------------------------------
    // Stop
    // --------------------------------------------------------

    case 'S':
    case 's':

      stopMotors();

      lastCommandTime = millis();

      break;

    // --------------------------------------------------------
    // Speed levels
    // --------------------------------------------------------

    case '1':

      currentSpeed = 80;

      break;

    case '2':

      currentSpeed = 110;

      break;

    case '3':

      currentSpeed = 150;

      break;

    case '4':

      currentSpeed = 190;

      break;

    case '5':

      currentSpeed = 230;

      break;

    // --------------------------------------------------------
    // Increase speed
    // --------------------------------------------------------

    case '+':

      currentSpeed += 10;

      currentSpeed =

        constrain(

          currentSpeed,

          0,

          255

        );

      break;

    // --------------------------------------------------------
    // Decrease speed
    // --------------------------------------------------------

    case '-':

      currentSpeed -= 10;

      currentSpeed =

        constrain(

          currentSpeed,

          0,

          255

        );

      break;

  }

}

// ============================================================
// STRING COMMAND PROCESSOR
// ============================================================

void processStringCommand(char *command) {

  // Convert command to uppercase

  for (int i = 0; command[i] != '\0'; i++) {

    if (

      command[i] >= 'a' &&

      command[i] <= 'z'

    ) {

      command[i] -= 32;

    }

  }

  // ----------------------------------------------------------
  // Forward
  // ----------------------------------------------------------

  if (

    strcmp(command, "FORWARD") == 0 ||

    strcmp(command, "FWD") == 0

  ) {

    moveForward();

    lastCommandTime = millis();

    return;

  }

  // ----------------------------------------------------------
  // Backward
  // ----------------------------------------------------------

  if (

    strcmp(command, "BACKWARD") == 0 ||

    strcmp(command, "REV") == 0 ||

    strcmp(command, "REVERSE") == 0

  ) {

    moveBackward();

    lastCommandTime = millis();

    return;

  }

  // ----------------------------------------------------------
  // Left
  // ----------------------------------------------------------

  if (

    strcmp(command, "LEFT") == 0

  ) {

    turnLeft();

    lastCommandTime = millis();

    return;

  }

  // ----------------------------------------------------------
  // Right
  // ----------------------------------------------------------

  if (

    strcmp(command, "RIGHT") == 0

  ) {

    turnRight();

    lastCommandTime = millis();

    return;

  }

  // ----------------------------------------------------------
  // Stop
  // ----------------------------------------------------------

  if (

    strcmp(command, "STOP") == 0 ||

    strcmp(command, "HALT") == 0

  ) {

    stopMotors();

    lastCommandTime = millis();

    return;

  }

  // ----------------------------------------------------------
  // SPEED command
  //
  // Examples:
  //
  // SPD:150
  // SPEED:200
  // ----------------------------------------------------------

  if (

    strncmp(

      command,

      "SPD:",

      4

    ) == 0

  ) {

    int newSpeed =

      atoi(command + 4);

    currentSpeed =

      constrain(

        newSpeed,

        0,

        255

      );

    return;

  }

  if (

    strncmp(

      command,

      "SPEED:",

      6

    ) == 0

  ) {

    int newSpeed =

      atoi(command + 6);

    currentSpeed =

      constrain(

        newSpeed,

        0,

        255

      );

    return;

  }

}

// ============================================================
// SERIAL COMMAND READER
// ============================================================

void readSerialCommands() {

  while (Serial.available() > 0) {

    char c = Serial.read();

    // --------------------------------------------------------
    // Ignore carriage return
    // --------------------------------------------------------

    if (c == '\r') {

      continue;

    }

    // --------------------------------------------------------
    // Newline = complete command
    // --------------------------------------------------------

    if (c == '\n') {

      if (serialCmdIndex > 0) {

        serialCmdBuffer[serialCmdIndex] = '\0';

        // If only one character was received,
        // treat it as a single-character command.

        if (serialCmdIndex == 1) {

          processCommand(

            serialCmdBuffer[0]

          );

        }

        else {

          processStringCommand(

            serialCmdBuffer

          );

        }

        serialCmdIndex = 0;

      }

      continue;

    }

    // --------------------------------------------------------
    // Ignore spaces when buffer is empty
    // --------------------------------------------------------

    if (

      c == ' ' &&

      serialCmdIndex == 0

    ) {

      continue;

    }

    // --------------------------------------------------------
    // Add character to command buffer
    // --------------------------------------------------------

    if (

      serialCmdIndex <

      SERIAL_BUFFER_SIZE - 1

    ) {

      serialCmdBuffer[serialCmdIndex++] = c;

    }

    else {

      // Buffer overflow protection

      serialCmdIndex = 0;

    }

  }

}