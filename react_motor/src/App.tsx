import React, { useState, useEffect, useRef, useCallback } from "react";
import {
  NavTab,
  OperationMode,
  RoverPose,
  TelemetryPacket,
  SensorRayTrace,
  SensorGeometry,
  MapObstacle,
  EnvironmentWall
} from "./types";
import { Navbar } from "./components/Navbar";
import { MappingCanvas } from "./components/MappingCanvas";
import { TelemetryHUD } from "./components/TelemetryHUD";
import { CodeExplorer } from "./components/CodeExplorer";
import { EngineeringDocs } from "./components/EngineeringDocs";
import { CalibrationStudio } from "./components/CalibrationStudio";
import { Usb, AlertCircle, Sparkles } from "lucide-react";

// Sensor mount geometry constants (3 Ultrasonic sensors: FRONT 0°, LEFT +90°, RIGHT -90°)
const SENSOR_CONFIGS: Record<string, SensorGeometry> = {
  FRONT: {
    id: "FRONT",
    name: "Front Center",
    angleDeg: 0.0,
    angleRad: 0.0,
    offsetX: 0.175,
    offsetY: 0.0,
    minDist: 0.03,
    maxDist: 4.0,
    beamAngleDeg: 15.0
  },
  LEFT: {
    id: "LEFT",
    name: "Left Flank",
    angleDeg: 90.0,
    angleRad: (90.0 * Math.PI) / 180,
    offsetX: 0.0,
    offsetY: 0.125,
    minDist: 0.03,
    maxDist: 4.0,
    beamAngleDeg: 15.0
  },
  RIGHT: {
    id: "RIGHT",
    name: "Right Flank",
    angleDeg: -90.0,
    angleRad: (-90.0 * Math.PI) / 180,
    offsetX: 0.0,
    offsetY: -0.125,
    minDist: 0.03,
    maxDist: 4.0,
    beamAngleDeg: 15.0
  }
};

// Initial simulated underground coal-mine boundaries
const INITIAL_MINE_WALLS: EnvironmentWall[] = [
  // Outer perimeter tunnel walls
  { id: "wall-s", x1: -5.5, y1: -3.5, x2: 5.5, y2: -3.5, type: "wall" },
  { id: "wall-e", x1: 5.5, y1: -3.5, x2: 5.5, y2: 3.5, type: "wall" },
  { id: "wall-n", x1: 5.5, y1: 3.5, x2: -5.5, y2: 3.5, type: "wall" },
  { id: "wall-w", x1: -5.5, y1: 3.5, x2: -5.5, y2: -3.5, type: "wall" },
  // Coal support pillar 1 (West)
  { id: "pil-1a", x1: -2.2, y1: -1.4, x2: -2.2, y2: -0.4, type: "pillar" },
  { id: "pil-1b", x1: -2.2, y1: -0.4, x2: -1.2, y2: -0.4, type: "pillar" },
  { id: "pil-1c", x1: -1.2, y1: -0.4, x2: -1.2, y2: -1.4, type: "pillar" },
  { id: "pil-1d", x1: -1.2, y1: -1.4, x2: -2.2, y2: -1.4, type: "pillar" },
  // Coal support pillar 2 (East)
  { id: "pil-2a", x1: 1.5, y1: 0.6, x2: 1.5, y2: 1.6, type: "pillar" },
  { id: "pil-2b", x1: 1.5, y1: 1.6, x2: 2.5, y2: 1.6, type: "pillar" },
  { id: "pil-2c", x1: 2.5, y1: 1.6, x2: 2.5, y2: 0.6, type: "pillar" },
  { id: "pil-2d", x1: 2.5, y1: 0.6, x2: 1.5, y2: 0.6, type: "pillar" }
];

export default function App() {
  const [currentTab, setCurrentTab] = useState<NavTab>("MONITOR");
  const [operationMode, setOperationMode] = useState<OperationMode>("SIMULATION");
  const [isStreaming, setIsStreaming] = useState<boolean>(true);
  const [isAutoPatrol, setIsAutoPatrol] = useState<boolean>(true);

  // Layer visibility toggles
  const [showRays, setShowRays] = useState<boolean>(true);
  const [showGrid, setShowGrid] = useState<boolean>(true);
  const [showTrail, setShowTrail] = useState<boolean>(true);
  const [showFreeSpace, setShowFreeSpace] = useState<boolean>(true);

  // Rover Kinematic State
  const [pose, setPose] = useState<RoverPose>({
    x: 0.0,
    y: 0.0,
    headingRad: 0.0,
    linearVelocity: 0.25,
    angularVelocity: 0.0,
    totalDistance: 0.0
  });

  const [path, setPath] = useState<Array<{ x: number; y: number }>>([{ x: 0, y: 0 }]);
  const [obstacles, setObstacles] = useState<MapObstacle[]>([]);
  const [freeSpaceCells, setFreeSpaceCells] = useState<Set<string>>(new Set());
  const [environmentWalls, setEnvironmentWalls] = useState<EnvironmentWall[]>(INITIAL_MINE_WALLS);

  // Ultrasonic rays state
  const [sensorRays, setSensorRays] = useState<Record<string, SensorRayTrace>>({});

  // Telemetry HUD state
  const [telemetry, setTelemetry] = useState<TelemetryPacket>({
    packetId: 0,
    timestamp: Date.now(),
    distFront: 1.25,
    distLeft: 0.95,
    distRight: 1.80,
    distFL: 1.25,
    distFR: 0.95,
    distRL: 1.80,
    distRR: 0.0,
    gy271HeadingDeg: 0.0,
    imu: {
      ax: 0.02,
      ay: -0.01,
      az: 1.0,
      gx: 0.4,
      gy: -0.2,
      gz: 0.8,
      mx: 31.5,
      my: -12.4,
      mz: 42.0,
      timestamp: Date.now()
    },
    checksumValid: true,
    rawPacket: "#0,DF=125.0,DL=95.0,DR=180.0,HDG=0.0,MX=31.5,MY=-12.4,MZ=42.0*58"
  });

  // Manual drive inputs
  const manualDriveRef = useRef<{ v: number; w: number }>({ v: 0.0, w: 0.0 });
  const serialPortRef = useRef<any>(null);
  const [baudRate, setBaudRate] = useState<number>(9600);
  const [activeDriveCommand, setActiveDriveCommand] = useState<string>("STOP");
  const [activeKeys, setActiveKeys] = useState<Set<string>>(new Set());

  // Bresenham 2D Line Algorithm to carve free space
  const carveFreeSpace = (
    x0: number,
    y0: number,
    x1: number,
    y1: number,
    res = 0.05,
    originOffset = 10.0
  ) => {
    const c0 = Math.floor((x0 + originOffset) / res);
    const r0 = Math.floor((y0 + originOffset) / res);
    const c1 = Math.floor((x1 + originOffset) / res);
    const r1 = Math.floor((y1 + originOffset) / res);

    const cells: string[] = [];
    const dx = Math.abs(c1 - c0);
    const dy = Math.abs(r1 - r0);
    const sx = c0 < c1 ? 1 : -1;
    const sy = r0 < r1 ? 1 : -1;
    let err = dx - dy;

    let currX = c0;
    let currY = r0;

    while (true) {
      cells.push(`${currX},${currY}`);
      if (currX === c1 && currY === r1) break;
      const e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        currX += sx;
      }
      if (e2 < dx) {
        err += dx;
        currY += sy;
      }
    }
    return cells;
  };

  // Ray-segment 2D intersection helper
  const rayIntersect = (
    ox: number,
    oy: number,
    angle: number,
    x1: number,
    y1: number,
    x2: number,
    y2: number,
    maxRange: number
  ): number | null => {
    const dx = Math.cos(angle);
    const dy = Math.sin(angle);
    const v1x = ox - x1;
    const v1y = oy - y1;
    const v2x = x2 - x1;
    const v2y = y2 - y1;
    const v3x = -dy;
    const v3y = dx;
    const dot = v2x * v3x + v2y * v3y;
    if (Math.abs(dot) < 1e-6) return null;
    const t1 = (v2x * v1y - v2y * v1x) / dot;
    const t2 = (v1x * v3x + v1y * v3y) / dot;
    if (t1 >= 0.0 && t1 <= maxRange && t2 >= 0.0 && t2 <= 1.0) {
      return t1;
    }
    return null;
  };

  // Reset origin & map
  const handleResetOrigin = () => {
    setPose({
      x: 0.0,
      y: 0.0,
      headingRad: 0.0,
      linearVelocity: 0.0,
      angularVelocity: 0.0,
      totalDistance: 0.0
    });
    setPath([{ x: 0, y: 0 }]);
    setObstacles([]);
    setFreeSpaceCells(new Set());
  };

  // Clear path trail
  const handleClearTrail = () => {
    setPath([{ x: pose.x, y: pose.y }]);
  };

  // Clear obstacle hits
  const handleClearObstacles = () => {
    setObstacles([]);
  };

  // Add custom obstacle by clicking on canvas
  const handleAddObstacle = (wx: number, wy: number) => {
    // Add small diamond rock obstacle
    const r = 0.35;
    const newWalls: EnvironmentWall[] = [
      { id: `obs-${Date.now()}-1`, x1: wx - r, y1: wy, x2: wx, y2: wy + r, type: "hazard" },
      { id: `obs-${Date.now()}-2`, x1: wx, y1: wy + r, x2: wx + r, y2: wy, type: "hazard" },
      { id: `obs-${Date.now()}-3`, x1: wx + r, y1: wy, x2: wx, y2: wy - r, type: "hazard" },
      { id: `obs-${Date.now()}-4`, x1: wx, y1: wy - r, x2: wx - r, y2: wy, type: "hazard" }
    ];
    setEnvironmentWalls((prev) => [...prev, ...newWalls]);
  };

  // Send command to physical rover over Web Serial if connected
  const sendSerialCommand = async (cmd: string) => {
    if (serialPortRef.current && serialPortRef.current.writable) {
      try {
        const textEncoder = new TextEncoder();
        const writer = serialPortRef.current.writable.getWriter();
        await writer.write(textEncoder.encode(`${cmd}\n`));
        writer.releaseLock();
      } catch (e) {
        console.warn("Serial write error:", e);
      }
    }
  };

  // Manual drive handler
  const handleDriveCommand = (cmd: "FORWARD" | "BACKWARD" | "LEFT" | "RIGHT" | "STOP") => {
    setIsAutoPatrol(false);
    setActiveDriveCommand(cmd);

    if (cmd === "FORWARD") {
      manualDriveRef.current = { v: 0.35, w: 0.0 };
      // Instant discrete tactile step for immediate path tracing on key press
      setPose((prev) => {
        const stepDist = 0.08;
        const newX = prev.x + stepDist * Math.cos(prev.headingRad);
        const newY = prev.y + stepDist * Math.sin(prev.headingRad);
        setPath((p) => [...p, { x: newX, y: newY }]);
        return {
          ...prev,
          x: newX,
          y: newY,
          linearVelocity: 0.35,
          totalDistance: prev.totalDistance + stepDist
        };
      });
      sendSerialCommand("F");
    } else if (cmd === "BACKWARD") {
      manualDriveRef.current = { v: -0.25, w: 0.0 };
      setPose((prev) => {
        const stepDist = -0.08;
        const newX = prev.x + stepDist * Math.cos(prev.headingRad);
        const newY = prev.y + stepDist * Math.sin(prev.headingRad);
        setPath((p) => [...p, { x: newX, y: newY }]);
        return {
          ...prev,
          x: newX,
          y: newY,
          linearVelocity: -0.25,
          totalDistance: prev.totalDistance + Math.abs(stepDist)
        };
      });
      sendSerialCommand("B");
    } else if (cmd === "LEFT") {
      manualDriveRef.current = { v: 0.08, w: 0.85 };
      setPose((prev) => {
        const newHeading = (prev.headingRad + 0.12) % (2 * Math.PI);
        return {
          ...prev,
          headingRad: newHeading,
          angularVelocity: 0.85
        };
      });
      sendSerialCommand("L");
    } else if (cmd === "RIGHT") {
      manualDriveRef.current = { v: 0.08, w: -0.85 };
      setPose((prev) => {
        const newHeading = (prev.headingRad - 0.12) % (2 * Math.PI);
        return {
          ...prev,
          headingRad: newHeading,
          angularVelocity: -0.85
        };
      });
      sendSerialCommand("R");
    } else {
      manualDriveRef.current = { v: 0.0, w: 0.0 };
      sendSerialCommand("S");
    }
  };

  // Keyboard listener for Arrow Keys and WASD navigation & path tracing
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      // Avoid capturing when user is typing in inputs or textareas
      if (e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement) {
        return;
      }

      const key = e.key;
      const code = e.code;

      if (key === "ArrowUp" || code === "KeyW") {
        e.preventDefault();
        setActiveKeys((prev) => new Set(prev).add("ArrowUp"));
        handleDriveCommand("FORWARD");
      } else if (key === "ArrowDown" || code === "KeyS") {
        e.preventDefault();
        setActiveKeys((prev) => new Set(prev).add("ArrowDown"));
        handleDriveCommand("BACKWARD");
      } else if (key === "ArrowLeft" || code === "KeyA") {
        e.preventDefault();
        setActiveKeys((prev) => new Set(prev).add("ArrowLeft"));
        handleDriveCommand("LEFT");
      } else if (key === "ArrowRight" || code === "KeyD") {
        e.preventDefault();
        setActiveKeys((prev) => new Set(prev).add("ArrowRight"));
        handleDriveCommand("RIGHT");
      } else if (code === "Space") {
        e.preventDefault();
        setActiveKeys(new Set());
        handleDriveCommand("STOP");
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      if (e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement) {
        return;
      }

      const key = e.key;
      const code = e.code;

      let keyToRemove = "";
      if (key === "ArrowUp" || code === "KeyW") keyToRemove = "ArrowUp";
      else if (key === "ArrowDown" || code === "KeyS") keyToRemove = "ArrowDown";
      else if (key === "ArrowLeft" || code === "KeyA") keyToRemove = "ArrowLeft";
      else if (key === "ArrowRight" || code === "KeyD") keyToRemove = "ArrowRight";

      if (keyToRemove) {
        setActiveKeys((prev) => {
          const next = new Set(prev);
          next.delete(keyToRemove);
          if (next.size === 0) {
            manualDriveRef.current = { v: 0.0, w: 0.0 };
            setActiveDriveCommand("STOP");
            sendSerialCommand("S");
          }
          return next;
        });
      }
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);
    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
    };
  }, []);

  // Emergency stop
  const handleEmergencyStop = () => {
    setIsAutoPatrol(false);
    setActiveDriveCommand("STOP");
    setActiveKeys(new Set());
    manualDriveRef.current = { v: 0.0, w: 0.0 };
    sendSerialCommand("S");
  };

  // Web Serial API connection to real Arduino
  const handleConnectWebSerial = async () => {
    if (!("serial" in navigator)) {
      alert(
        "Web Serial API is not supported in this browser. Please use Google Chrome or Microsoft Edge to connect to an Arduino Uno over USB."
      );
      return;
    }

    try {
      const navSerial = (navigator as any).serial;
      const port = await navSerial.requestPort();
      await port.open({ baudRate });
      serialPortRef.current = port;
      setOperationMode("WEB_SERIAL");

      const textDecoder = new TextDecoderStream();
      port.readable.pipeTo(textDecoder.writable);
      const reader = textDecoder.readable.getReader();

      let serialBuffer = "";
      while (true) {
        const { value, done } = await reader.read();
        if (done) break;
        if (value) {
          serialBuffer += value;
          const lines = serialBuffer.split("\n");
          serialBuffer = lines.pop() || "";
          for (const line of lines) {
            parseHardwarePacket(line.trim());
          }
        }
      }
    } catch (err) {
      console.warn("Web Serial error or cancelled:", err);
      setOperationMode("SIMULATION");
    }
  };

  const parseHardwarePacket = (line: string) => {
    if (!line) return;
    let clean = line.trim();
    let checksumValid = true;

    // Remove DATA: prefix if present (e.g. DATA:#148,...)
    if (clean.startsWith("DATA:")) {
      clean = clean.slice(5).trim();
    }

    // Strip trailing asterisk if present (e.g. DATA:#148,...*)
    if (clean.endsWith("*")) {
      clean = clean.slice(0, -1).trim();
    } else if (clean.includes("*")) {
      // Optional XOR checksum (*XX) if present
      const [data, csStr] = clean.split("*");
      if (csStr && csStr.length > 0) {
        const expected = parseInt(csStr, 16);
        if (!isNaN(expected)) {
          let calc = 0;
          for (let i = 0; i < data.length; i++) calc ^= data.charCodeAt(i);
          checksumValid = calc === expected;
        }
      }
      clean = data;
    }

    const tokens = clean.split(",");
    let packetId = telemetry.packetId;
    let dFront = telemetry.distFront;
    let dLeft = telemetry.distLeft;
    let dRight = telemetry.distRight;
    let hdg = telemetry.gy271HeadingDeg;
    let imuData = { ...telemetry.imu };
    let foundFields = 0;
    let hasDistances = false;

    for (const token of tokens) {
      const trimmed = token.trim();
      if (!trimmed) continue;

      // Packet counter token (e.g. #148 or DATA:#148)
      if (trimmed.startsWith("#") || trimmed.startsWith("DATA:#")) {
        const num = parseInt(trimmed.split("#")[1]);
        if (!isNaN(num)) {
          packetId = num;
          foundFields++;
        }
        continue;
      }

      const sep = trimmed.includes("=") ? "=" : trimmed.includes(":") ? ":" : null;
      if (!sep) continue;
      const [k, vStr] = trimmed.split(sep);
      const key = k.trim().toUpperCase();
      const val = parseFloat(vStr.trim());
      if (isNaN(val)) continue;

      // DF = Front distance in cm (TRIG D2 / ECHO D3)
      if (key === "DF" || key === "FRONT" || key === "D1" || key === "FL") {
        dFront = val > 10 ? val / 100 : val;
        hasDistances = true;
        foundFields++;
      }
      // DL = Left distance in cm (TRIG D4 / ECHO D5)
      else if (key === "DL" || key === "LEFT" || key === "D2") {
        dLeft = val > 10 ? val / 100 : val;
        hasDistances = true;
        foundFields++;
      }
      // DR = Right distance in cm (TRIG D6 / ECHO D7)
      else if (key === "DR" || key === "RIGHT" || key === "D3") {
        dRight = val > 10 ? val / 100 : val;
        hasDistances = true;
        foundFields++;
      }
      // HDG = Relative heading from gyro Z
      else if (key === "HDG" || key === "HEADING" || key === "ANGLE" || key === "ANGLEZ" || key === "YAW") {
        let h = val % 360;
        if (h < 0) h += 360;
        hdg = h;
        foundFields++;
      }
      // GZ = Gyroscope Z in deg/sec
      else if (key === "GZ") {
        imuData.gz = val;
        foundFields++;
      }
      // AX / AY / AZ = Accelerometer values
      else if (key === "AX") { imuData.ax = val; foundFields++; }
      else if (key === "AY") { imuData.ay = val; foundFields++; }
      else if (key === "AZ") { imuData.az = val; foundFields++; }
      else if (key === "GX") { imuData.gx = val; foundFields++; }
      else if (key === "GY") { imuData.gy = val; foundFields++; }
    }

    if (foundFields === 0) {
      // Fallback regex for "Angle: XX.XX" or "Angle = XX.XX"
      const match = clean.match(/Angle\s*[:=]\s*([-\d.]+)/i);
      if (match) {
        let rawAngle = parseFloat(match[1]);
        if (!isNaN(rawAngle)) {
          while (rawAngle < 0) rawAngle += 360;
          while (rawAngle >= 360) rawAngle -= 360;
          hdg = rawAngle;
          foundFields++;
        }
      }
    }

    // If hardware sketch provided heading without physical ultrasonics, generate pseudo distances
    if (!hasDistances && foundFields >= 1) {
      const rad = (hdg * Math.PI) / 180.0;
      const cosA = Math.cos(rad);
      const sinA = Math.sin(rad);
      const dx = Math.abs(cosA) > 0.01 ? 1.8 / Math.abs(cosA) : 3.5;
      const dy = Math.abs(sinA) > 0.01 ? 0.95 / Math.abs(sinA) : 3.5;
      dFront = Math.max(0.1, Math.min(Math.min(dx, dy), 3.8));

      const radL = ((hdg + 90.0) * Math.PI) / 180.0;
      const cosL = Math.cos(radL);
      const sinL = Math.sin(radL);
      const dxL = Math.abs(cosL) > 0.01 ? 1.8 / Math.abs(cosL) : 3.5;
      const dyL = Math.abs(sinL) > 0.01 ? 0.95 / Math.abs(sinL) : 3.5;
      dLeft = Math.max(0.1, Math.min(Math.min(dxL, dyL), 3.8));

      const radR = ((hdg - 90.0) * Math.PI) / 180.0;
      const cosR = Math.cos(radR);
      const sinR = Math.sin(radR);
      const dxR = Math.abs(cosR) > 0.01 ? 1.8 / Math.abs(cosR) : 3.5;
      const dyR = Math.abs(sinR) > 0.01 ? 0.95 / Math.abs(sinR) : 3.5;
      dRight = Math.max(0.1, Math.min(Math.min(dxR, dyR), 3.8));
    }

    setTelemetry((prev) => ({
      ...prev,
      packetId: packetId > 0 ? packetId : prev.packetId + 1,
      distFront: dFront,
      distLeft: dLeft,
      distRight: dRight,
      distFL: dFront,
      distFR: dLeft,
      distRL: dRight,
      distRR: 0.0,
      gy271HeadingDeg: parseFloat(hdg.toFixed(2)),
      imu: imuData,
      checksumValid,
      rawPacket: line
    }));

    // Update real-time rover orientation from physical MPU gyro heading
    if (foundFields >= 1) {
      setPose((prev) => ({
        ...prev,
        headingRad: (hdg * Math.PI) / 180.0
      }));
    }
  };

  // Main High-Performance Physics & Mapping Simulation Loop (runs at ~20 Hz)
  useEffect(() => {
    if (!isStreaming) return;

    const dt = 0.05; // 50 ms = 20 Hz
    let seq = 0;

    const interval = setInterval(() => {
      seq++;

      setPose((prevPose) => {
        let linearV = prevPose.linearVelocity;
        let angularW = prevPose.angularVelocity;

        if (isAutoPatrol) {
          linearV = 0.26;
          // Smooth coal-mine tunnel path exploration
          angularW = 0.35 * Math.sin(seq * 0.035);

          // Boundary turn avoidance
          if (
            Math.abs(prevPose.x) > 4.2 ||
            Math.abs(prevPose.y) > 2.6 ||
            (prevPose.x > 0.8 && prevPose.x < 3.2 && prevPose.y > 0.0 && prevPose.y < 2.0)
          ) {
            angularW = 0.75;
          }
        } else {
          linearV = manualDriveRef.current.v;
          angularW = manualDriveRef.current.w;
        }

        const newHeading = prevPose.headingRad + angularW * dt;
        const stepDist = linearV * dt;
        const newX = prevPose.x + stepDist * Math.cos(newHeading);
        const newY = prevPose.y + stepDist * Math.sin(newHeading);
        const newTotalDist = prevPose.totalDistance + Math.abs(stepDist);

        // Update breadcrumb path every 5cm
        setPath((prevPath) => {
          const last = prevPath[prevPath.length - 1];
          const distFromLast = Math.hypot(newX - last.x, newY - last.y);
          if (distFromLast > 0.05) {
            return [...prevPath.slice(-600), { x: newX, y: newY }];
          }
          return prevPath;
        });

        // 2. Perform 4 Ultrasonic Ray-Casting against environment walls & pillars
        const newRays: Record<string, SensorRayTrace> = {};
        const simulatedDistances: Record<string, number> = {};
        const newlyCarvedCells: string[] = [];
        const newObstacleHits: MapObstacle[] = [];

        for (const [key, cfg] of Object.entries(SENSOR_CONFIGS)) {
          // SE(2) Forward Kinematics: Sensor origin in global world frame
          const cosH = Math.cos(newHeading);
          const sinH = Math.sin(newHeading);
          const originGx = newX + (cfg.offsetX * cosH - cfg.offsetY * sinH);
          const originGy = newY + (cfg.offsetX * sinH + cfg.offsetY * cosH);

          const globalBeamAngle = newHeading + cfg.angleRad;

          // Ray cast against all walls
          let minDistance = cfg.maxDist;
          for (const wall of environmentWalls) {
            const hitDist = rayIntersect(
              originGx,
              originGy,
              globalBeamAngle,
              wall.x1,
              wall.y1,
              wall.x2,
              wall.y2,
              cfg.maxDist
            );
            if (hitDist !== null && hitDist < minDistance) {
              minDistance = hitDist;
            }
          }

          // Add realistic sensor noise (±1.5cm)
          const noisyDist = Math.max(
            cfg.minDist,
            Math.min(cfg.maxDist, minDistance + (Math.random() * 0.03 - 0.015))
          );
          simulatedDistances[key] = noisyDist;

          const validHit = noisyDist >= cfg.minDist && noisyDist <= cfg.maxDist - 0.05;
          const targetGx = originGx + noisyDist * Math.cos(globalBeamAngle);
          const targetGy = originGy + noisyDist * Math.sin(globalBeamAngle);

          newRays[key] = {
            sensorId: key,
            originX: originGx,
            originY: originGy,
            targetX: targetGx,
            targetY: targetGy,
            distance: noisyDist,
            validHit
          };

          // Carve free space
          const rayCells = carveFreeSpace(originGx, originGy, targetGx, targetGy);
          for (const c of rayCells) newlyCarvedCells.push(c);

          // Add obstacle hit
          if (validHit) {
            newObstacleHits.push({
              x: targetGx,
              y: targetGy,
              confidence: 0.85,
              timestamp: Date.now(),
              hits: 1
            });
          }
        }

        setSensorRays(newRays);

        // Update free space cells set
        setFreeSpaceCells((prev) => {
          const next = new Set(prev);
          for (let i = 0; i < Math.min(newlyCarvedCells.length, 60); i++) {
            next.add(newlyCarvedCells[i]);
          }
          return next;
        });

        // Add persistent obstacles
        if (newObstacleHits.length > 0) {
          setObstacles((prev) => {
            const updated = [...prev];
            for (const hit of newObstacleHits) {
              const existing = updated.find(
                (o) => Math.hypot(o.x - hit.x, o.y - hit.y) < 0.12
              );
              if (existing) {
                existing.hits = Math.min(10, existing.hits + 1);
              } else if (updated.length < 500) {
                updated.push(hit);
              }
            }
            return updated;
          });
        }

        // Update Telemetry HUD packet (FRONT, LEFT, RIGHT, and GY-271 HDG)
        const headingDeg = ((newHeading * 180 / Math.PI) % 360 + 360) % 360;

        setTelemetry((prev) => {
          const dF = simulatedDistances.FRONT || prev.distFront || 0;
          const dL = simulatedDistances.LEFT  || prev.distLeft || 0;
          const dR = simulatedDistances.RIGHT || prev.distRight || 0;

          return {
            packetId: seq,
            timestamp: Date.now(),
            distFront: dF,
            distLeft: dL,
            distRight: dR,
            distFL: dF,
            distFR: dL,
            distRL: dR,
            distRR: 0.0,
            gy271HeadingDeg: parseFloat(headingDeg.toFixed(1)),
            imu: {
              ax: parseFloat((Math.sin(newHeading) * 0.05).toFixed(2)),
              ay: parseFloat(((linearV * angularW) / 9.8).toFixed(2)),
              az: 1.0,
              gx: 0.1,
              gy: 0.1,
              gz: parseFloat(((angularW * 180) / Math.PI).toFixed(1)),
              mx: parseFloat((32.0 * Math.cos(newHeading)).toFixed(1)),
              my: parseFloat((32.0 * Math.sin(newHeading)).toFixed(1)),
              mz: 38.0,
              timestamp: Date.now()
            },
            checksumValid: true,
            rawPacket: `DATA:#${seq},DF=${(dF * 100).toFixed(1)},DL=${(dL * 100).toFixed(1)},DR=${(dR * 100).toFixed(1)},HDG=${headingDeg.toFixed(2)},GZ=${((angularW * 180) / Math.PI).toFixed(2)},AX=${(Math.sin(newHeading) * 0.05).toFixed(2)},AY=${((linearV * angularW) / 9.8).toFixed(2)},AZ=1.00*`
          };
        });

        return {
          x: newX,
          y: newY,
          headingRad: newHeading,
          linearVelocity: linearV,
          angularVelocity: angularW,
          totalDistance: newTotalDist
        };
      });
    }, 50);

    return () => clearInterval(interval);
  }, [isStreaming, isAutoPatrol, environmentWalls]);

  return (
    <div className="min-h-screen bg-neutral-950 text-neutral-100 flex flex-col selection:bg-cyan-500 selection:text-neutral-950">
      {/* 1. Global Navigation Bar */}
      <Navbar
        currentTab={currentTab}
        setTab={setCurrentTab}
        operationMode={operationMode}
        packetCount={telemetry.packetId}
        isStreaming={isStreaming}
        onEmergencyStop={handleEmergencyStop}
        onResetOrigin={handleResetOrigin}
      />

      {/* 2. Main Workspace Content according to active tab */}
      <main className="flex-1 w-full pb-8">
        {currentTab === "MONITOR" && (
          <div className="max-w-7xl mx-auto px-4 py-4 flex flex-col gap-4">
            {/* Real Hardware Connect Banner */}
            <div className="flex flex-wrap items-center justify-between gap-3 bg-neutral-900/90 border border-neutral-800 p-3 rounded-xl shadow-xs text-xs font-mono">
              <div className="flex items-center gap-2">
                <Usb className="w-4 h-4 text-cyan-400" />
                <span>HARDWARE SERIAL LINK:</span>
                <span className="text-neutral-400">
                  {operationMode === "WEB_SERIAL"
                    ? `Connected to Live USB Serial Arduino (${baudRate} baud) — Heading & Ultrasonics Active`
                    : "Underground Exploration Simulator (Ready for USB Hardware Hookup)"}
                </span>
              </div>

              <div className="flex items-center gap-2">
                {operationMode === "SIMULATION" ? (
                  <>
                    <div className="flex items-center bg-neutral-950 border border-neutral-800 rounded-lg p-0.5 text-[11px]">
                      <button
                        onClick={() => setBaudRate(9600)}
                        className={`px-2 py-1 rounded transition-colors ${
                          baudRate === 9600
                            ? "bg-cyan-900/60 text-cyan-300 font-semibold border border-cyan-700/50"
                            : "text-neutral-400 hover:text-neutral-200"
                        }`}
                        title="Standard baud rate matching user MPU-6050 sketch"
                      >
                        9600 Baud (Default)
                      </button>
                      <button
                        onClick={() => setBaudRate(115200)}
                        className={`px-2 py-1 rounded transition-colors ${
                          baudRate === 115200
                            ? "bg-cyan-900/60 text-cyan-300 font-semibold border border-cyan-700/50"
                            : "text-neutral-400 hover:text-neutral-200"
                        }`}
                        title="High-speed baud rate"
                      >
                        115200 Baud
                      </button>
                    </div>

                    <button
                      onClick={handleConnectWebSerial}
                      className="flex items-center gap-1.5 px-3 py-1.5 rounded-lg bg-cyan-950 hover:bg-cyan-900 text-cyan-300 border border-cyan-700/60 font-semibold text-xs transition-colors"
                    >
                      <Usb className="w-3.5 h-3.5" />
                      <span>Connect Real Arduino via USB</span>
                    </button>
                  </>
                ) : (
                  <button
                    onClick={() => setOperationMode("SIMULATION")}
                    className="flex items-center gap-1 px-3 py-1.5 rounded-lg bg-neutral-800 hover:bg-neutral-700 text-neutral-300 border border-neutral-700 font-semibold text-xs transition-colors"
                  >
                    <span>Switch Back to Simulator</span>
                  </button>
                )}
              </div>
            </div>

            {/* Split Screen: 2D Occupancy Map Canvas (Left) + Telemetry HUD (Right) */}
            <div className="grid grid-cols-1 lg:grid-cols-12 gap-4">
              <div className="lg:col-span-8 flex flex-col gap-3">
                <MappingCanvas
                  pose={pose}
                  path={path}
                  sensorRays={sensorRays}
                  sensorConfigs={SENSOR_CONFIGS}
                  obstacles={obstacles}
                  environmentWalls={environmentWalls}
                  onAddObstacle={handleAddObstacle}
                  showRays={showRays}
                  showGrid={showGrid}
                  showTrail={showTrail}
                  showFreeSpace={showFreeSpace}
                  freeSpaceCells={freeSpaceCells}
                />
              </div>

              <div className="lg:col-span-4">
                <TelemetryHUD
                  pose={pose}
                  telemetry={telemetry}
                  sensorRays={sensorRays}
                  isAutoPatrol={isAutoPatrol}
                  onToggleAutoPatrol={() => setIsAutoPatrol((v) => !v)}
                  onDriveCommand={handleDriveCommand}
                  onClearObstacles={handleClearObstacles}
                  onClearTrail={handleClearTrail}
                  showRays={showRays}
                  setShowRays={setShowRays}
                  showFreeSpace={showFreeSpace}
                  setShowFreeSpace={setShowFreeSpace}
                  showTrail={showTrail}
                  setShowTrail={setShowTrail}
                  obstacleCount={obstacles.length}
                  activeDriveCommand={activeDriveCommand}
                  activeKeys={Array.from(activeKeys)}
                />
              </div>
            </div>
          </div>
        )}

        {currentTab === "CODE_EXPLORER" && <CodeExplorer />}

        {currentTab === "ENGINEERING_DOCS" && <EngineeringDocs />}

        {currentTab === "CALIBRATION" && <CalibrationStudio />}
      </main>

      {/* 3. Footer */}
      <footer className="border-t border-neutral-800/80 bg-neutral-950 py-3 text-center text-xs text-neutral-500 font-mono">
        REACT Rover Exploration System &bull; 2D Hazard Occupancy Mapping &bull; Arduino Uno Serial &bull; Matplotlib Ground Control Station
      </footer>
    </div>
  );
}
