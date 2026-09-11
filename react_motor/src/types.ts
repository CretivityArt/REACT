/**
 * REACT (Reactive Hazard Exploration/Response Rover)
 * Shared TypeScript Definitions
 */

export interface SensorGeometry {
  id?: string;
  name: string;
  angleDeg: number;     // degrees relative to +X (forward)
  angleRad: number;
  offsetX: number;      // meters forward from center
  offsetY: number;      // meters left from center
  minDist: number;      // meters
  maxDist: number;      // meters
  beamAngleDeg: number; // degrees
}

export interface UltrasonicReadings {
  front: number; // meters
  left: number;
  right: number;
  // Aliases
  FL?: number;
  FR?: number;
  RL?: number;
  RR?: number;
}

export interface IMUTelemetry {
  ax: number; // g
  ay: number;
  az: number;
  gx: number; // deg/s
  gy: number;
  gz: number;
  mx: number; // uT
  my: number;
  mz: number;
  timestamp: number;
}

export interface TelemetryPacket {
  packetId: number;
  timestamp: number;
  // 3 Ultrasonic sensors
  distFront: number; // meters
  distLeft: number;
  distRight: number;
  // Backward compatibility
  distFL: number;
  distFR: number;
  distRL: number;
  distRR: number;
  // GY-271 Compass Heading
  gy271HeadingDeg: number;
  imu: IMUTelemetry;
  checksumValid: boolean;
  rawPacket: string;
}

export interface RoverPose {
  x: number;              // meters (East / Forward origin)
  y: number;              // meters (North / Left origin)
  headingRad: number;     // radians
  linearVelocity: number; // m/s
  angularVelocity: number;// rad/s
  totalDistance: number;  // meters
}

export interface SensorRayTrace {
  sensorId?: string;
  sensorKey?: string;
  originX: number;
  originY: number;
  targetX: number;
  targetY: number;
  distance: number;
  validHit: boolean;
}

export interface SerialPacketLog {
  id: number;
  timestamp: string;
  raw: string;
  checksumValid: boolean;
}

export interface MapObstacle {
  x: number;
  y: number;
  hits: number;
  confidence?: number;
  timestamp?: number;
}

export type OperationMode = "SIMULATION" | "WEB_SERIAL" | "STANDBY";

export type NavTab = "MONITOR" | "CODE_EXPLORER" | "ENGINEERING_DOCS" | "CALIBRATION";

export interface EnvironmentWall {
  id?: string;
  x1: number;
  y1: number;
  x2: number;
  y2: number;
  type: "wall" | "pillar" | "hazard" | "rubble";
}

export interface CalibrationOffsets {
  gyroBiasX: number;
  gyroBiasY: number;
  gyroBiasZ: number;
  magOffsetX: number;
  magOffsetY: number;
  magOffsetZ: number;
  magScaleX: number;
  magScaleY: number;
  magScaleZ: number;
}

export interface CodeFile {
  name: string;
  path: string;
  category: "python" | "arduino" | "config" | "docs";
  description: string;
  content: string;
  language: "python" | "cpp" | "plaintext" | "markdown";
}
