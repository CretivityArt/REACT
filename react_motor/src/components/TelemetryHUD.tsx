import React from "react";
import {
  Compass,
  Activity,
  ShieldAlert,
  Navigation,
  Cpu,
  ArrowUp,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  Square,
  Play,
  Pause,
  Layers,
  Sparkles
} from "lucide-react";
import { RoverPose, TelemetryPacket, SensorRayTrace } from "../types";

interface TelemetryHUDProps {
  pose: RoverPose;
  telemetry: TelemetryPacket;
  sensorRays: Record<string, SensorRayTrace>;
  isAutoPatrol: boolean;
  onToggleAutoPatrol: () => void;
  onDriveCommand: (cmd: "FORWARD" | "BACKWARD" | "LEFT" | "RIGHT" | "STOP") => void;
  onClearObstacles: () => void;
  onClearTrail: () => void;
  showRays: boolean;
  setShowRays: (val: boolean) => void;
  showFreeSpace: boolean;
  setShowFreeSpace: (val: boolean) => void;
  showTrail: boolean;
  setShowTrail: (val: boolean) => void;
  obstacleCount: number;
  activeDriveCommand?: string;
  activeKeys?: string[];
}

export const TelemetryHUD: React.FC<TelemetryHUDProps> = ({
  pose,
  telemetry,
  sensorRays,
  isAutoPatrol,
  onToggleAutoPatrol,
  onDriveCommand,
  onClearObstacles,
  onClearTrail,
  showRays,
  setShowRays,
  showFreeSpace,
  setShowFreeSpace,
  showTrail,
  setShowTrail,
  obstacleCount,
  activeDriveCommand = "STOP",
  activeKeys = []
}) => {
  const headingDeg = telemetry.gy271HeadingDeg !== undefined && telemetry.gy271HeadingDeg !== 0
    ? telemetry.gy271HeadingDeg
    : ((pose.headingRad * 180) / Math.PI + 360) % 360;

  // Cardinal direction calculator
  const getCardinal = (deg: number) => {
    const directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];
    const idx = Math.round(deg / 45) % 8;
    return directions[idx];
  };

  // Helper for sensor range bar percentage & color
  const getRangeMeta = (distMeters: number) => {
    const cm = distMeters * 100;
    const pct = Math.min(100, Math.max(0, (distMeters / 4.0) * 100));
    let colorClass = "bg-emerald-500 text-emerald-400";
    if (distMeters < 0.35) {
      colorClass = "bg-red-500 text-red-400";
    } else if (distMeters < 0.9) {
      colorClass = "bg-amber-500 text-amber-400";
    }
    return { cm: cm.toFixed(1), pct, colorClass };
  };

  const dfDist = telemetry.distFront ?? telemetry.distFL ?? 0;
  const dlDist = telemetry.distLeft ?? telemetry.distFR ?? 0;
  const drDist = telemetry.distRight ?? telemetry.distRL ?? 0;

  const dfMeta = getRangeMeta(dfDist);
  const dlMeta = getRangeMeta(dlDist);
  const drMeta = getRangeMeta(drDist);

  return (
    <div className="flex flex-col gap-3 w-full text-neutral-200 text-xs font-mono select-none">
      {/* 1. Rover Pose & Heading Card */}
      <div className="bg-neutral-900/90 border border-neutral-800 rounded-xl p-3.5 flex flex-col gap-2.5 shadow-sm">
        <div className="flex items-center justify-between pb-2 border-b border-neutral-800">
          <div className="flex items-center gap-1.5 text-cyan-400 font-semibold text-xs">
            <Navigation className="w-3.5 h-3.5" />
            <span>ROVER POSE & KINEMATICS</span>
          </div>
          <span className="text-[10px] text-neutral-400 font-normal">Global World Frame</span>
        </div>

        <div className="grid grid-cols-3 gap-2 text-center">
          <div className="bg-neutral-950 p-2 rounded-lg border border-neutral-800">
            <span className="text-[10px] text-neutral-500 uppercase block">Global X</span>
            <span className="text-sm font-bold text-neutral-100 font-mono">
              {pose.x >= 0 ? `+${pose.x.toFixed(2)}` : pose.x.toFixed(2)}
              <span className="text-[10px] font-normal text-neutral-500 ml-0.5">m</span>
            </span>
          </div>

          <div className="bg-neutral-950 p-2 rounded-lg border border-neutral-800">
            <span className="text-[10px] text-neutral-500 uppercase block">Global Y</span>
            <span className="text-sm font-bold text-neutral-100 font-mono">
              {pose.y >= 0 ? `+${pose.y.toFixed(2)}` : pose.y.toFixed(2)}
              <span className="text-[10px] font-normal text-neutral-500 ml-0.5">m</span>
            </span>
          </div>

          <div className="bg-neutral-950 p-2 rounded-lg border border-neutral-800">
            <span className="text-[10px] text-neutral-500 uppercase block">MPU Gyro Heading</span>
            <span className="text-sm font-bold text-cyan-400 font-mono">
              {headingDeg.toFixed(1)}° <span className="text-xs text-neutral-400">{getCardinal(headingDeg)}</span>
            </span>
          </div>
        </div>

        <div className="flex items-center justify-between text-[11px] pt-1 text-neutral-400">
          <span>Speed: <strong className="text-neutral-200">{pose.linearVelocity.toFixed(2)} m/s</strong></span>
          <span>Distance: <strong className="text-neutral-200">{pose.totalDistance.toFixed(2)} m</strong></span>
          <span>Obstacles: <strong className="text-red-400">{obstacleCount}</strong></span>
        </div>
      </div>

      {/* 2. Ultrasonic Transducers Layout (DF, DL, DR) */}
      <div className="bg-neutral-900/90 border border-neutral-800 rounded-xl p-3.5 flex flex-col gap-2.5 shadow-sm">
        <div className="flex items-center justify-between pb-2 border-b border-neutral-800">
          <div className="flex items-center gap-1.5 text-amber-400 font-semibold text-xs">
            <Activity className="w-3.5 h-3.5" />
            <span>ULTRASONIC SENSORS (HC-SR04)</span>
          </div>
          <span className="text-[10px] text-emerald-400 font-medium">DF (D2/D3) &bull; DL (D4/D5) &bull; DR (D6/D7)</span>
        </div>

        <div className="grid grid-cols-3 gap-2">
          {/* Front (DF) */}
          <div className="bg-neutral-950 p-2.5 rounded-lg border border-neutral-800 flex flex-col gap-1">
            <div className="flex justify-between items-center text-[10px]">
              <span className="text-amber-300 font-semibold flex items-center gap-1">
                <span className="w-2 h-2 rounded-full bg-amber-400 inline-block"></span>
                DF (Front)
              </span>
              <span className={dfMeta.colorClass}>{dfMeta.cm} cm</span>
            </div>
            <div className="text-[9px] text-neutral-500">Trig D2 / Echo D3</div>
            <div className="w-full h-1.5 bg-neutral-800 rounded-full overflow-hidden mt-0.5">
              <div
                className={`h-full transition-all duration-150 ${
                  dfDist < 0.35 ? "bg-red-500" : dfDist < 0.9 ? "bg-amber-500" : "bg-emerald-500"
                }`}
                style={{ width: `${dfMeta.pct}%` }}
              />
            </div>
          </div>

          {/* Left (DL) */}
          <div className="bg-neutral-950 p-2.5 rounded-lg border border-neutral-800 flex flex-col gap-1">
            <div className="flex justify-between items-center text-[10px]">
              <span className="text-sky-400 font-semibold flex items-center gap-1">
                <span className="w-2 h-2 rounded-full bg-sky-400 inline-block"></span>
                DL (Left)
              </span>
              <span className={dlMeta.colorClass}>{dlMeta.cm} cm</span>
            </div>
            <div className="text-[9px] text-neutral-500">Trig D4 / Echo D5</div>
            <div className="w-full h-1.5 bg-neutral-800 rounded-full overflow-hidden mt-0.5">
              <div
                className={`h-full transition-all duration-150 ${
                  dlDist < 0.35 ? "bg-red-500" : dlDist < 0.9 ? "bg-amber-500" : "bg-emerald-500"
                }`}
                style={{ width: `${dlMeta.pct}%` }}
              />
            </div>
          </div>

          {/* Right (DR) */}
          <div className="bg-neutral-950 p-2.5 rounded-lg border border-neutral-800 flex flex-col gap-1">
            <div className="flex justify-between items-center text-[10px]">
              <span className="text-emerald-400 font-semibold flex items-center gap-1">
                <span className="w-2 h-2 rounded-full bg-emerald-400 inline-block"></span>
                DR (Right)
              </span>
              <span className={drMeta.colorClass}>{drMeta.cm} cm</span>
            </div>
            <div className="text-[9px] text-neutral-500">Trig D6 / Echo D7</div>
            <div className="w-full h-1.5 bg-neutral-800 rounded-full overflow-hidden mt-0.5">
              <div
                className={`h-full transition-all duration-150 ${
                  drDist < 0.35 ? "bg-red-500" : drDist < 0.9 ? "bg-amber-500" : "bg-emerald-500"
                }`}
                style={{ width: `${drMeta.pct}%` }}
              />
            </div>
          </div>
        </div>

        {/* Proximity Warning Banner */}
        {(dfDist < 0.35 || dlDist < 0.35 || drDist < 0.35) && (
          <div className="flex items-center gap-2 bg-red-950/80 border border-red-800/80 text-red-300 px-2.5 py-1.5 rounded-lg animate-pulse text-[10px]">
            <ShieldAlert className="w-3.5 h-3.5 shrink-0 text-red-400" />
            <span>OBSTACLE PROXIMITY ALERT (&lt;35cm)</span>
          </div>
        )}
      </div>

      {/* 3. Live Serial Line Stream Card */}
      <div className="bg-neutral-900/90 border border-neutral-800 rounded-xl p-3 flex flex-col gap-1.5 shadow-sm">
        <div className="flex items-center justify-between text-[10px]">
          <div className="flex items-center gap-1.5 text-cyan-300 font-semibold">
            <span className="relative flex h-2 w-2">
              <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-emerald-400 opacity-75"></span>
              <span className="relative inline-flex rounded-full h-2 w-2 bg-emerald-500"></span>
            </span>
            <span>LIVE SERIAL DATA (DATA: FORMAT)</span>
          </div>
          <span className="text-neutral-500">9600 Baud</span>
        </div>
        <div className="bg-neutral-950 p-2 rounded-lg border border-neutral-800/80 font-mono text-[11px] text-cyan-400/90 truncate select-all">
          {telemetry.rawPacket || `DATA:#${telemetry.packetId},DF=${dfMeta.cm},DL=${dlMeta.cm},DR=${drMeta.cm},HDG=${headingDeg.toFixed(2)},GZ=${(telemetry.imu.gz ?? 0).toFixed(2)},AX=${(telemetry.imu.ax ?? 0).toFixed(2)},AY=${(telemetry.imu.ay ?? 0).toFixed(2)},AZ=${(telemetry.imu.az ?? 1.0).toFixed(2)}*`}
        </div>
      </div>

      {/* 3. MPU-6050 6-Axis MotionTracking IMU */}
      <div className="bg-neutral-900/90 border border-neutral-800 rounded-xl p-3.5 flex flex-col gap-2.5 shadow-sm">
        <div className="flex items-center justify-between pb-2 border-b border-neutral-800">
          <div className="flex items-center gap-1.5 text-violet-400 font-semibold text-xs">
            <Cpu className="w-3.5 h-3.5" />
            <span>MPU-6050 6-AXIS IMU</span>
          </div>
          <span className="text-[10px] text-emerald-400 font-semibold bg-emerald-950/60 px-1.5 py-0.5 rounded border border-emerald-800/40">
            I2C 0x68 ONLINE
          </span>
        </div>

        {/* Accelerometer (3-Axis) & Gyroscope (3-Axis) */}
        <div className="grid grid-cols-2 gap-2 text-[10px]">
          {/* Accelerometer (g) */}
          <div className="bg-neutral-950 p-2 rounded-lg border border-neutral-800">
            <div className="text-neutral-400 font-medium mb-1 flex items-center justify-between">
              <span>ACCEL (g)</span>
              <span className="text-[9px] text-neutral-500">±2g</span>
            </div>
            <div className="space-y-0.5 font-mono">
              <div className="flex justify-between">
                <span className="text-neutral-500">Ax:</span>
                <span className="text-neutral-200">{(telemetry.imu.ax ?? 0) >= 0 ? `+${(telemetry.imu.ax ?? 0).toFixed(2)}` : (telemetry.imu.ax ?? 0).toFixed(2)}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-neutral-500">Ay:</span>
                <span className="text-neutral-200">{(telemetry.imu.ay ?? 0) >= 0 ? `+${(telemetry.imu.ay ?? 0).toFixed(2)}` : (telemetry.imu.ay ?? 0).toFixed(2)}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-neutral-500">Az:</span>
                <span className="text-neutral-200">{(telemetry.imu.az ?? 1.0) >= 0 ? `+${(telemetry.imu.az ?? 1.0).toFixed(2)}` : (telemetry.imu.az ?? 1.0).toFixed(2)}</span>
              </div>
            </div>
          </div>

          {/* Gyroscope (deg/s) */}
          <div className="bg-neutral-950 p-2 rounded-lg border border-neutral-800">
            <div className="text-neutral-400 font-medium mb-1 flex items-center justify-between">
              <span>GYRO (°/s)</span>
              <span className="text-[9px] text-neutral-500">±250°/s</span>
            </div>
            <div className="space-y-0.5 font-mono">
              <div className="flex justify-between">
                <span className="text-neutral-500">Gx:</span>
                <span className="text-neutral-200">{(telemetry.imu.gx ?? 0) >= 0 ? `+${(telemetry.imu.gx ?? 0).toFixed(1)}` : (telemetry.imu.gx ?? 0).toFixed(1)}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-neutral-500">Gy:</span>
                <span className="text-neutral-200">{(telemetry.imu.gy ?? 0) >= 0 ? `+${(telemetry.imu.gy ?? 0).toFixed(1)}` : (telemetry.imu.gy ?? 0).toFixed(1)}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-neutral-500">Gz (Yaw):</span>
                <span className="text-violet-300 font-bold">{(telemetry.imu.gz ?? 0) >= 0 ? `+${(telemetry.imu.gz ?? 0).toFixed(1)}` : (telemetry.imu.gz ?? 0).toFixed(1)}</span>
              </div>
            </div>
          </div>
        </div>

        {/* Calculated Pitch & Roll from Gravity Vector */}
        <div className="flex items-center justify-between bg-neutral-950 p-2 rounded-lg border border-neutral-800 text-[11px]">
          <div className="flex items-center gap-1.5">
            <span className="text-neutral-400">Pitch (Tilt):</span>
            <span className="font-mono font-semibold text-amber-400">
              {((Math.atan2(-(telemetry.imu.ax ?? 0), Math.sqrt((telemetry.imu.ay ?? 0) ** 2 + (telemetry.imu.az ?? 1.0) ** 2)) * 180) / Math.PI).toFixed(1)}°
            </span>
          </div>
          <div className="flex items-center gap-1.5">
            <span className="text-neutral-400">Roll:</span>
            <span className="font-mono font-semibold text-sky-400">
              {((Math.atan2(telemetry.imu.ay ?? 0, telemetry.imu.az ?? 1.0) * 180) / Math.PI).toFixed(1)}°
            </span>
          </div>
        </div>
      </div>

      {/* 4. Heading & Compass Display */}
      <div className="bg-neutral-900/90 border border-neutral-800 rounded-xl p-3.5 flex flex-col gap-2.5 shadow-sm">
        <div className="flex items-center justify-between pb-2 border-b border-neutral-800">
          <div className="flex items-center gap-1.5 text-cyan-400 font-semibold text-xs">
            <Compass className="w-3.5 h-3.5" />
            <span>HEADING & ORIENTATION</span>
          </div>
          <span className="text-[10px] text-neutral-400 font-normal">MPU-6050 / GY-271</span>
        </div>

        {/* Compass Visual Rose & Heading Banner */}
        <div className="flex items-center gap-3 bg-neutral-950 p-2.5 rounded-lg border border-neutral-800">
          <div className="relative w-12 h-12 rounded-full border border-neutral-700 bg-neutral-900 flex items-center justify-center shrink-0">
            <div
              className="w-1 h-9 bg-gradient-to-t from-transparent via-red-500 to-red-400 rounded-full transition-transform duration-150"
              style={{ transform: `rotate(${headingDeg}deg)` }}
            />
            <span className="absolute text-[8px] font-bold text-neutral-400 top-0.5">N</span>
            <span className="absolute text-[7px] text-neutral-600 right-0.5">E</span>
            <span className="absolute text-[7px] text-neutral-600 bottom-0.5">S</span>
            <span className="absolute text-[7px] text-neutral-600 left-0.5">W</span>
            <div className="w-2 h-2 rounded-full bg-cyan-400 z-10"></div>
          </div>

          <div className="flex flex-col flex-1">
            <div className="flex justify-between items-baseline">
              <span className="text-neutral-400 text-[11px]">Azimuth Yaw</span>
              <span className="text-cyan-400 font-bold text-base font-mono">
                {headingDeg.toFixed(1)}° <span className="text-xs text-amber-300 font-semibold">({getCardinal(headingDeg)})</span>
              </span>
            </div>
            <span className="text-[10px] text-neutral-500">
              Integrated MPU-6050 Gyro + Tilt-Compensated Vector
            </span>
          </div>
        </div>

        {(telemetry.imu.mx !== 0 || telemetry.imu.my !== 0) && (
          <div className="flex justify-between items-center py-0.5 text-[11px]">
            <span className="text-neutral-400">Magnetic Field (µT):</span>
            <span className="font-mono text-neutral-200">
              Mx:{telemetry.imu.mx >= 0 ? `+${telemetry.imu.mx.toFixed(1)}` : telemetry.imu.mx.toFixed(1)}{" "}
              My:{telemetry.imu.my >= 0 ? `+${telemetry.imu.my.toFixed(1)}` : telemetry.imu.my.toFixed(1)}{" "}
              Mz:{telemetry.imu.mz >= 0 ? `+${telemetry.imu.mz.toFixed(1)}` : telemetry.imu.mz.toFixed(1)}
            </span>
          </div>
        )}
      </div>

      {/* 4. Drive Controls & Autonomous Patrol */}
      <div className="bg-neutral-900/90 border border-neutral-800 rounded-xl p-3.5 flex flex-col gap-2.5 shadow-sm">
        <div className="flex items-center justify-between pb-2 border-b border-neutral-800">
          <span className="text-xs font-semibold text-neutral-300">NAVIGATION CONTROLS</span>
          <button
            onClick={onToggleAutoPatrol}
            className={`flex items-center gap-1 text-[10px] font-semibold px-2 py-1 rounded border transition-colors ${
              isAutoPatrol
                ? "bg-amber-950/80 text-amber-300 border-amber-500/50"
                : "bg-neutral-800 text-neutral-300 border-neutral-700 hover:bg-neutral-700"
            }`}
          >
            {isAutoPatrol ? <Pause className="w-3 h-3" /> : <Play className="w-3 h-3" />}
            <span>{isAutoPatrol ? "Autonomous Exploration ON" : "Start Auto-Patrol"}</span>
          </button>
        </div>

        {/* Direction Keypad & Keyboard Teleoperation */}
        <div className="flex flex-col items-center gap-1.5 pt-1">
          <div className="w-full flex items-center justify-between px-1 text-[10px] text-neutral-400 font-mono">
            <span>KEYBOARD TELEOP</span>
            <span className={`px-1.5 py-0.5 rounded text-[9px] font-bold ${
              activeDriveCommand !== "STOP" && activeDriveCommand !== "IDLE"
                ? "bg-cyan-950 text-cyan-300 border border-cyan-700"
                : "bg-neutral-800 text-neutral-400"
            }`}>
              {activeDriveCommand}
            </span>
          </div>

          <button
            onClick={() => onDriveCommand("FORWARD")}
            className={`w-14 h-9 rounded-lg flex flex-col items-center justify-center border transition-all shadow-xs ${
              activeDriveCommand === "FORWARD" || activeKeys.includes("ArrowUp") || activeKeys.includes("KeyW")
                ? "bg-cyan-500 text-neutral-950 border-cyan-300 ring-2 ring-cyan-400/40"
                : "bg-neutral-800 hover:bg-neutral-700 text-neutral-200 border-neutral-700"
            }`}
            title="Drive Forward (Arrow Up / W)"
          >
            <ArrowUp className="w-4 h-4" />
            <span className="text-[8px] font-mono leading-none opacity-80">↑ / W</span>
          </button>

          <div className="flex items-center gap-2">
            <button
              onClick={() => onDriveCommand("LEFT")}
              className={`w-14 h-9 rounded-lg flex flex-col items-center justify-center border transition-all shadow-xs ${
                activeDriveCommand === "LEFT" || activeKeys.includes("ArrowLeft") || activeKeys.includes("KeyA")
                  ? "bg-cyan-500 text-neutral-950 border-cyan-300 ring-2 ring-cyan-400/40"
                  : "bg-neutral-800 hover:bg-neutral-700 text-neutral-200 border-neutral-700"
              }`}
              title="Turn Left (Arrow Left / A)"
            >
              <ArrowLeft className="w-4 h-4" />
              <span className="text-[8px] font-mono leading-none opacity-80">← / A</span>
            </button>

            <button
              onClick={() => onDriveCommand("STOP")}
              className={`w-12 h-9 rounded-lg flex flex-col items-center justify-center border transition-all shadow-xs ${
                activeDriveCommand === "STOP"
                  ? "bg-red-900/90 text-red-200 border-red-500"
                  : "bg-red-950 hover:bg-red-900 text-red-300 border-red-800"
              }`}
              title="Emergency Stop (Space)"
            >
              <Square className="w-3.5 h-3.5" />
              <span className="text-[7.5px] font-mono leading-none opacity-80">SPACE</span>
            </button>

            <button
              onClick={() => onDriveCommand("RIGHT")}
              className={`w-14 h-9 rounded-lg flex flex-col items-center justify-center border transition-all shadow-xs ${
                activeDriveCommand === "RIGHT" || activeKeys.includes("ArrowRight") || activeKeys.includes("KeyD")
                  ? "bg-cyan-500 text-neutral-950 border-cyan-300 ring-2 ring-cyan-400/40"
                  : "bg-neutral-800 hover:bg-neutral-700 text-neutral-200 border-neutral-700"
              }`}
              title="Turn Right (Arrow Right / D)"
            >
              <ArrowRight className="w-4 h-4" />
              <span className="text-[8px] font-mono leading-none opacity-80">→ / D</span>
            </button>
          </div>

          <button
            onClick={() => onDriveCommand("BACKWARD")}
            className={`w-14 h-9 rounded-lg flex flex-col items-center justify-center border transition-all shadow-xs ${
              activeDriveCommand === "BACKWARD" || activeKeys.includes("ArrowDown") || activeKeys.includes("KeyS")
                ? "bg-cyan-500 text-neutral-950 border-cyan-300 ring-2 ring-cyan-400/40"
                : "bg-neutral-800 hover:bg-neutral-700 text-neutral-200 border-neutral-700"
            }`}
            title="Drive Backward (Arrow Down / S)"
          >
            <ArrowDown className="w-4 h-4" />
            <span className="text-[8px] font-mono leading-none opacity-80">↓ / S</span>
          </button>

          <p className="text-[9px] text-neutral-400 font-mono text-center pt-1">
            Press Arrow Keys on keyboard to steer rover & trace path
          </p>
        </div>

        {/* Layer Toggles & Clear */}
        <div className="pt-2 border-t border-neutral-800 flex flex-wrap gap-2 text-[10px]">
          <label className="flex items-center gap-1.5 cursor-pointer text-neutral-300">
            <input
              type="checkbox"
              checked={showRays}
              onChange={(e) => setShowRays(e.target.checked)}
              className="rounded bg-neutral-800 border-neutral-700 text-cyan-500"
            />
            <span>Sensor Rays</span>
          </label>

          <label className="flex items-center gap-1.5 cursor-pointer text-neutral-300">
            <input
              type="checkbox"
              checked={showFreeSpace}
              onChange={(e) => setShowFreeSpace(e.target.checked)}
              className="rounded bg-neutral-800 border-neutral-700 text-cyan-500"
            />
            <span>Free Space</span>
          </label>

          <label className="flex items-center gap-1.5 cursor-pointer text-neutral-300">
            <input
              type="checkbox"
              checked={showTrail}
              onChange={(e) => setShowTrail(e.target.checked)}
              className="rounded bg-neutral-800 border-neutral-700 text-cyan-500"
            />
            <span>Trajectory Trail</span>
          </label>
        </div>

        <div className="flex gap-2 pt-1 text-[10px]">
          <button
            onClick={onClearObstacles}
            className="flex-1 py-1 rounded bg-neutral-800 hover:bg-neutral-700 text-neutral-300 border border-neutral-700 transition-colors"
          >
            Clear Obstacles
          </button>
          <button
            onClick={onClearTrail}
            className="flex-1 py-1 rounded bg-neutral-800 hover:bg-neutral-700 text-neutral-300 border border-neutral-700 transition-colors"
          >
            Clear Trail
          </button>
        </div>
      </div>
    </div>
  );
};
