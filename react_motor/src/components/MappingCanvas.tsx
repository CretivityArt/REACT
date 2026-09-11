import React, { useRef, useEffect, useState, useCallback } from "react";
import {
  ZoomIn,
  ZoomOut,
  Crosshair,
  Layers,
  MapPin,
  Compass,
  AlertTriangle
} from "lucide-react";
import {
  RoverPose,
  SensorRayTrace,
  SensorGeometry,
  MapObstacle,
  EnvironmentWall
} from "../types";

interface MappingCanvasProps {
  pose: RoverPose;
  path: Array<{ x: number; y: number }>;
  sensorRays: Record<string, SensorRayTrace>;
  sensorConfigs: Record<string, SensorGeometry>;
  obstacles: MapObstacle[];
  environmentWalls: EnvironmentWall[];
  onAddObstacle: (x: number, y: number) => void;
  showRays: boolean;
  showGrid: boolean;
  showTrail: boolean;
  showFreeSpace: boolean;
  freeSpaceCells: Set<string>; // "col,row"
}

export const MappingCanvas: React.FC<MappingCanvasProps> = ({
  pose,
  path,
  sensorRays,
  sensorConfigs,
  obstacles,
  environmentWalls,
  onAddObstacle,
  showRays,
  showGrid,
  showTrail,
  showFreeSpace,
  freeSpaceCells
}) => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const containerRef = useRef<HTMLDivElement | null>(null);

  // Viewport transforms (pan & zoom)
  const [scale, setScale] = useState<number>(45); // pixels per meter (default 45px = 1m)
  const [pan, setPan] = useState<{ x: number; y: number }>({ x: 0, y: 0 });
  const [isPanning, setIsPanning] = useState<boolean>(false);
  const [dragStart, setDragStart] = useState<{ x: number; y: number }>({ x: 0, y: 0 });
  const [followRover, setFollowRover] = useState<boolean>(true);
  const [cursorWorldCoords, setCursorWorldCoords] = useState<{ x: number; y: number } | null>(null);

  // Reset view to center on rover
  const centerOnRover = useCallback(() => {
    if (!containerRef.current) return;
    const width = containerRef.current.clientWidth;
    const height = containerRef.current.clientHeight;
    // Map coords: +X is East (right), +Y is North (up)
    setPan({
      x: width / 2 - pose.x * scale,
      y: height / 2 + pose.y * scale
    });
    setFollowRover(true);
  }, [pose.x, pose.y, scale]);

  // Keep centered on rover if followRover is active
  useEffect(() => {
    if (followRover && containerRef.current) {
      const width = containerRef.current.clientWidth;
      const height = containerRef.current.clientHeight;
      setPan({
        x: width / 2 - pose.x * scale,
        y: height / 2 + pose.y * scale
      });
    }
  }, [pose.x, pose.y, followRover, scale]);

  // Handle Canvas Resize
  useEffect(() => {
    const handleResize = () => {
      if (!canvasRef.current || !containerRef.current) return;
      canvasRef.current.width = containerRef.current.clientWidth;
      canvasRef.current.height = containerRef.current.clientHeight;
    };
    handleResize();
    window.addEventListener("resize", handleResize);
    return () => window.removeEventListener("resize", handleResize);
  }, []);

  // Main Canvas Rendering Loop
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const width = canvas.width;
    const height = canvas.height;

    // Helper: World (meters) to Screen (pixels)
    // In World: +X is East (right), +Y is North (up)
    // In Screen: (0,0) is top-left, +Y is down
    const toScreen = (wx: number, wy: number): [number, number] => {
      const sx = pan.x + wx * scale;
      const sy = pan.y - wy * scale;
      return [sx, sy];
    };

    // Helper: Screen to World
    const toWorld = (sx: number, sy: number): [number, number] => {
      const wx = (sx - pan.x) / scale;
      const wy = (pan.y - sy) / scale;
      return [wx, wy];
    };

    // 1. Clear background (Deep dark coal-mine basalt theme)
    ctx.fillStyle = "#0a0d13";
    ctx.fillRect(0, 0, width, height);

    // 2. Render Metric Grid & Coordinates
    if (showGrid) {
      const minWorld = toWorld(0, height);
      const maxWorld = toWorld(width, 0);

      const startX = Math.floor(minWorld[0]);
      const endX = Math.ceil(maxWorld[0]);
      const startY = Math.floor(minWorld[1]);
      const endY = Math.ceil(maxWorld[1]);

      // Minor grid lines (0.5m)
      ctx.lineWidth = 0.5;
      ctx.strokeStyle = "#161d28";
      ctx.beginPath();
      for (let x = startX * 2; x <= endX * 2; x++) {
        const [sx] = toScreen(x * 0.5, 0);
        ctx.moveTo(sx, 0);
        ctx.lineTo(sx, height);
      }
      for (let y = startY * 2; y <= endY * 2; y++) {
        const [, sy] = toScreen(0, y * 0.5);
        ctx.moveTo(0, sy);
        ctx.lineTo(width, sy);
      }
      ctx.stroke();

      // Major grid lines (1.0m)
      ctx.lineWidth = 1;
      ctx.strokeStyle = "#222d3d";
      ctx.beginPath();
      for (let x = startX; x <= endX; x++) {
        const [sx] = toScreen(x, 0);
        ctx.moveTo(sx, 0);
        ctx.lineTo(sx, height);
      }
      for (let y = startY; y <= endY; y++) {
        const [, sy] = toScreen(0, y);
        ctx.moveTo(0, sy);
        ctx.lineTo(width, sy);
      }
      ctx.stroke();

      // World Origin Axes (X=0 and Y=0)
      const [originSx, originSy] = toScreen(0, 0);
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = "rgba(74, 222, 128, 0.4)"; // Green North axis (+Y)
      ctx.beginPath();
      ctx.moveTo(originSx, originSy);
      ctx.lineTo(originSx, 0);
      ctx.stroke();

      ctx.strokeStyle = "rgba(56, 189, 248, 0.4)"; // Cyan East axis (+X)
      ctx.beginPath();
      ctx.moveTo(originSx, originSy);
      ctx.lineTo(width, originSy);
      ctx.stroke();

      // Metric labels
      ctx.font = "10px monospace";
      ctx.fillStyle = "#4b5563";
      for (let x = startX; x <= endX; x++) {
        if (x % 2 === 0 && x !== 0) {
          const [sx, sy] = toScreen(x, 0);
          ctx.fillText(`${x}m`, sx + 4, Math.min(height - 10, Math.max(20, originSy - 6)));
        }
      }
      for (let y = startY; y <= endY; y++) {
        if (y % 2 === 0 && y !== 0) {
          const [sx, sy] = toScreen(0, y);
          ctx.fillText(`${y}m`, Math.min(width - 30, Math.max(10, originSx + 6)), sy - 4);
        }
      }
    }

    // 3. Render Free Space Occupancy Cells (Carved by Bresenham algorithm)
    if (showFreeSpace && freeSpaceCells.size > 0) {
      const resolution = 0.05; // 5cm
      const cellPx = Math.max(2, resolution * scale);
      ctx.fillStyle = "rgba(14, 165, 233, 0.07)"; // subtle blue/cyan for carved free path
      for (const key of freeSpaceCells) {
        const [cStr, rStr] = key.split(",");
        const c = parseInt(cStr, 10);
        const r = parseInt(rStr, 10);
        const wx = (c + 0.5) * resolution - 10.0;
        const wy = (r + 0.5) * resolution - 10.0;
        const [sx, sy] = toScreen(wx, wy);
        ctx.fillRect(sx - cellPx / 2, sy - cellPx / 2, cellPx, cellPx);
      }
    }

    // 4. Render Simulated Environment Walls & Mine Pillars
    if (environmentWalls.length > 0) {
      ctx.lineWidth = 3;
      ctx.strokeStyle = "#374151";
      ctx.fillStyle = "rgba(55, 65, 81, 0.25)";
      for (const w of environmentWalls) {
        const [s1x, s1y] = toScreen(w.x1, w.y1);
        const [s2x, s2y] = toScreen(w.x2, w.y2);
        ctx.beginPath();
        ctx.moveTo(s1x, s1y);
        ctx.lineTo(s2x, s2y);
        ctx.stroke();

        // If pillar or hazard, highlight with hazard stripe or border
        if (w.type === "hazard" || w.type === "pillar") {
          ctx.strokeStyle = "#b45309";
          ctx.lineWidth = 1.5;
          ctx.stroke();
        }
      }
    }

    // 5. Render Trajectory Breadcrumb Trail
    if (showTrail && path.length > 1) {
      ctx.beginPath();
      const [firstSx, firstSy] = toScreen(path[0].x, path[0].y);
      ctx.moveTo(firstSx, firstSy);

      for (let i = 1; i < path.length; i++) {
        const [sx, sy] = toScreen(path[i].x, path[i].y);
        ctx.lineTo(sx, sy);
      }
      ctx.strokeStyle = "#00d4ff";
      ctx.lineWidth = 2;
      ctx.lineCap = "round";
      ctx.lineJoin = "round";
      ctx.stroke();

      // Origin start marker
      ctx.fillStyle = "#10b981";
      ctx.beginPath();
      ctx.arc(firstSx, firstSy, 4, 0, Math.PI * 2);
      ctx.fill();
    }

    // 6. Render Persistent Obstacle Points & Occupancy Hits
    for (const obs of obstacles) {
      const [sx, sy] = toScreen(obs.x, obs.y);
      const hitRadius = Math.min(8, Math.max(3, obs.hits * 1.2));

      // Glow circle
      const grad = ctx.createRadialGradient(sx, sy, 1, sx, sy, hitRadius * 2);
      grad.addColorStop(0, "rgba(239, 68, 68, 0.9)");
      grad.addColorStop(1, "rgba(239, 68, 68, 0)");
      ctx.fillStyle = grad;
      ctx.beginPath();
      ctx.arc(sx, sy, hitRadius * 2, 0, Math.PI * 2);
      ctx.fill();

      // Core obstacle point
      ctx.fillStyle = "#ef4444";
      ctx.fillRect(sx - 2, sy - 2, 4, 4);
    }

    // 7. Render 4 Ultrasonic Sensor Rays (FL, FR, RL, RR)
    if (showRays) {
      for (const [key, rVal] of Object.entries(sensorRays)) {
        const ray = rVal as SensorRayTrace;
        if (!ray) continue;
        const [ox, oy] = toScreen(ray.originX, ray.originY);
        const [tx, ty] = toScreen(ray.targetX, ray.targetY);

        // Ray line color according to range alert thresholds
        let strokeColor = "#22c55e"; // Green (> 1.0m)
        let alertGlow = "rgba(34, 197, 94, 0.3)";
        if (ray.distance < 0.35) {
          strokeColor = "#ef4444"; // Danger Red (< 35cm)
          alertGlow = "rgba(239, 68, 68, 0.5)";
        } else if (ray.distance < 0.9) {
          strokeColor = "#f59e0b"; // Warning Amber (< 90cm)
          alertGlow = "rgba(245, 158, 11, 0.4)";
        }

        // Draw dashed beam ray
        ctx.save();
        ctx.setLineDash([4, 4]);
        ctx.strokeStyle = strokeColor;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(ox, oy);
        ctx.lineTo(tx, ty);
        ctx.stroke();
        ctx.restore();

        // Draw obstacle contact point if valid hit
        if (ray.validHit) {
          ctx.fillStyle = strokeColor;
          ctx.beginPath();
          ctx.arc(tx, ty, 4, 0, Math.PI * 2);
          ctx.fill();

          // Outer pulse circle
          ctx.strokeStyle = alertGlow;
          ctx.lineWidth = 2;
          ctx.beginPath();
          ctx.arc(tx, ty, 8, 0, Math.PI * 2);
          ctx.stroke();

          // Distance label in cm
          ctx.font = "bold 9px monospace";
          ctx.fillStyle = "#f3f4f6";
          const distCm = (ray.distance * 100).toFixed(0);
          ctx.fillText(`${key}:${distCm}cm`, tx + 6, ty - 4);
        }
      }
    }

    // 8. Render Rover Chassis & Direction Triangle
    const [roverSx, roverSy] = toScreen(pose.x, pose.y);
    const roverLengthPx = 0.35 * scale; // 35 cm length
    const roverWidthPx = 0.25 * scale;  // 25 cm width

    ctx.save();
    ctx.translate(roverSx, roverSy);
    // In standard math: theta counter-clockwise from East (+X).
    // In canvas: positive rotation is clockwise, and Y is inverted, so rotate(-headingRad)
    ctx.rotate(-pose.headingRad);

    // Rover Footprint (Chassis base)
    ctx.fillStyle = "#1e293b";
    ctx.strokeStyle = "#38bdf8";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.roundRect(-roverLengthPx / 2, -roverWidthPx / 2, roverLengthPx, roverWidthPx, 4);
    ctx.fill();
    ctx.stroke();

    // Wheel tracks (Left and Right treads)
    ctx.fillStyle = "#0f172a";
    ctx.strokeStyle = "#64748b";
    ctx.lineWidth = 1;
    // Left tread (+Y)
    ctx.fillRect(-roverLengthPx / 2 - 2, -roverWidthPx / 2 - 4, roverLengthPx + 4, 5);
    ctx.strokeRect(-roverLengthPx / 2 - 2, -roverWidthPx / 2 - 4, roverLengthPx + 4, 5);
    // Right tread (-Y)
    ctx.fillRect(-roverLengthPx / 2 - 2, roverWidthPx / 2 - 1, roverLengthPx + 4, 5);
    ctx.strokeRect(-roverLengthPx / 2 - 2, roverWidthPx / 2 - 1, roverLengthPx + 4, 5);

    // Front Direction Pointer (Arrow / Triangle pointing forward +X)
    ctx.fillStyle = "#22c55e";
    ctx.beginPath();
    ctx.moveTo(roverLengthPx / 2 + 6, 0);
    ctx.lineTo(roverLengthPx / 2 - 6, -roverWidthPx / 4);
    ctx.lineTo(roverLengthPx / 2 - 6, roverWidthPx / 4);
    ctx.closePath();
    ctx.fill();

    // Rover center marker
    ctx.fillStyle = "#38bdf8";
    ctx.beginPath();
    ctx.arc(0, 0, 3, 0, Math.PI * 2);
    ctx.fill();

    // 3 Ultrasonic Sensor Transducers (FRONT, LEFT perpendicular, RIGHT perpendicular)
    const sensorPlacements = [
      { name: "FRONT", x: roverLengthPx / 2, y: 0, color: "#f59e0b", label: "F" },
      { name: "LEFT",  x: 0, y: -roverWidthPx / 2, color: "#22c55e", label: "L" },
      { name: "RIGHT", x: 0, y: roverWidthPx / 2,  color: "#38bdf8", label: "R" }
    ];

    for (const s of sensorPlacements) {
      ctx.fillStyle = s.color;
      ctx.beginPath();
      ctx.arc(s.x, s.y, 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 1;
      ctx.stroke();

      // Transducer mini label
      ctx.font = "bold 8px monospace";
      ctx.fillStyle = "#ffffff";
      ctx.fillText(s.label, s.x - 2.5, s.y + (s.y < 0 ? -6 : s.y > 0 ? 12 : 11));
    }

    // MPU-6050 6-DOF IMU chip representation at chassis center
    ctx.fillStyle = "#7c3aed"; // Violet MPU-6050
    ctx.fillRect(-6, -6, 12, 12);
    ctx.strokeStyle = "#c084fc";
    ctx.lineWidth = 1;
    ctx.strokeRect(-6, -6, 12, 12);
    ctx.fillStyle = "#ffffff";
    ctx.font = "bold 6.5px sans-serif";
    ctx.fillText("MPU", -6, 3);

    ctx.restore();

    // 9. Render Heading Vector Line from Rover Center
    const headingVectorLen = 45;
    const hx = roverSx + headingVectorLen * Math.cos(pose.headingRad);
    const hy = roverSy - headingVectorLen * Math.sin(pose.headingRad);

    ctx.save();
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    ctx.moveTo(roverSx, roverSy);
    ctx.lineTo(hx, hy);
    ctx.stroke();

    // Arrowhead
    const headAngle = Math.atan2(roverSy - hy, hx - roverSx);
    ctx.fillStyle = "#ffffff";
    ctx.beginPath();
    ctx.moveTo(hx, hy);
    ctx.lineTo(
      hx - 8 * Math.cos(headAngle - Math.PI / 6),
      hy + 8 * Math.sin(headAngle - Math.PI / 6)
    );
    ctx.lineTo(
      hx - 8 * Math.cos(headAngle + Math.PI / 6),
      hy + 8 * Math.sin(headAngle + Math.PI / 6)
    );
    ctx.closePath();
    ctx.fill();
    ctx.restore();

  }, [
    scale,
    pan,
    pose,
    path,
    sensorRays,
    obstacles,
    environmentWalls,
    showRays,
    showGrid,
    showTrail,
    showFreeSpace,
    freeSpaceCells
  ]);

  // Mouse pan & zoom handlers
  const handleMouseDown = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (e.button === 0) {
      // Left-click pan
      setIsPanning(true);
      setDragStart({ x: e.clientX - pan.x, y: e.clientY - pan.y });
      setFollowRover(false);
    }
  };

  const handleMouseMove = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (isPanning) {
      setPan({
        x: e.clientX - dragStart.x,
        y: e.clientY - dragStart.y
      });
    }

    // Update cursor metric coordinate display
    const rect = canvasRef.current?.getBoundingClientRect();
    if (rect) {
      const sx = e.clientX - rect.left;
      const sy = e.clientY - rect.top;
      const wx = (sx - pan.x) / scale;
      const wy = (pan.y - sy) / scale;
      setCursorWorldCoords({ x: wx, y: wy });
    }
  };

  const handleMouseUp = () => {
    setIsPanning(false);
  };

  const handleWheel = (e: React.WheelEvent<HTMLCanvasElement>) => {
    e.preventDefault();
    const zoomFactor = e.deltaY < 0 ? 1.15 : 0.87;
    const newScale = Math.max(15, Math.min(120, scale * zoomFactor));

    // Zoom centered at cursor
    const rect = canvasRef.current?.getBoundingClientRect();
    if (rect) {
      const mouseSx = e.clientX - rect.left;
      const mouseSy = e.clientY - rect.top;

      const newPanX = mouseSx - (mouseSx - pan.x) * (newScale / scale);
      const newPanY = mouseSy - (mouseSy - pan.y) * (newScale / scale);

      setScale(newScale);
      setPan({ x: newPanX, y: newPanY });
    }
  };

  // Double-click to place a custom obstacle in the mine
  const handleDoubleClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const rect = canvasRef.current?.getBoundingClientRect();
    if (rect) {
      const sx = e.clientX - rect.left;
      const sy = e.clientY - rect.top;
      const wx = (sx - pan.x) / scale;
      const wy = (pan.y - sy) / scale;
      onAddObstacle(wx, wy);
    }
  };

  return (
    <div
      ref={containerRef}
      className="relative w-full h-[520px] lg:h-[620px] bg-neutral-950 rounded-xl overflow-hidden border border-neutral-800 shadow-inner select-none"
    >
      <canvas
        ref={canvasRef}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseUp}
        onWheel={handleWheel}
        onDoubleClick={handleDoubleClick}
        className={`w-full h-full block ${isPanning ? "cursor-grabbing" : "cursor-crosshair"}`}
      />

      {/* Top Left: Canvas Header & Metric Coordinates */}
      <div className="absolute top-3 left-3 flex flex-col gap-1 bg-neutral-900/85 backdrop-blur-md px-3 py-2 rounded-lg border border-neutral-800 text-xs font-mono shadow-md">
        <div className="flex items-center gap-2">
          <span className="w-2 h-2 rounded-full bg-cyan-400 animate-pulse" />
          <span className="text-cyan-300 font-semibold uppercase tracking-wider">
            2D Occupancy Grid Map
          </span>
          <span className="text-[10px] text-neutral-400">5cm/cell</span>
        </div>
        <div className="text-neutral-400 text-[11px] flex gap-3">
          <span>Rover: ({pose.x.toFixed(2)}m, {pose.y.toFixed(2)}m)</span>
          {cursorWorldCoords && (
            <span className="text-neutral-300">
              Cursor: ({cursorWorldCoords.x.toFixed(2)}m, {cursorWorldCoords.y.toFixed(2)}m)
            </span>
          )}
        </div>
      </div>

      {/* Top Right: View Controls */}
      <div className="absolute top-3 right-3 flex items-center gap-1.5 bg-neutral-900/85 backdrop-blur-md p-1.5 rounded-lg border border-neutral-800 shadow-md">
        <button
          onClick={() => setScale((s) => Math.min(120, s * 1.2))}
          title="Zoom In"
          className="p-1.5 rounded-md hover:bg-neutral-800 text-neutral-300 hover:text-white transition-colors"
        >
          <ZoomIn className="w-4 h-4" />
        </button>
        <button
          onClick={() => setScale((s) => Math.max(15, s * 0.8))}
          title="Zoom Out"
          className="p-1.5 rounded-md hover:bg-neutral-800 text-neutral-300 hover:text-white transition-colors"
        >
          <ZoomOut className="w-4 h-4" />
        </button>
        <button
          onClick={centerOnRover}
          title={followRover ? "Centering active (Lock on Rover)" : "Center on Rover"}
          className={`p-1.5 rounded-md transition-colors ${
            followRover
              ? "bg-cyan-950 text-cyan-400 border border-cyan-500/40"
              : "hover:bg-neutral-800 text-neutral-400 hover:text-white"
          }`}
        >
          <Crosshair className="w-4 h-4" />
        </button>
      </div>

      {/* Bottom Left: Distance Scale Legend */}
      <div className="absolute bottom-3 left-3 bg-neutral-900/85 backdrop-blur-md px-3 py-1.5 rounded-lg border border-neutral-800 text-[11px] font-mono text-neutral-300 shadow-md flex items-center gap-3">
        <span>Scale:</span>
        <div className="flex flex-col items-center">
          <div
            className="h-1.5 bg-cyan-400 border-x-2 border-white"
            style={{ width: `${scale}px` }}
          />
          <span className="text-[10px] text-neutral-400">1.0 Meter</span>
        </div>
        <span className="text-[10px] text-neutral-500 hidden sm:inline">
          (Double-click canvas to spawn obstacle rock)
        </span>
      </div>

      {/* Bottom Right: Sensor Ray Legend */}
      <div className="absolute bottom-3 right-3 hidden sm:flex items-center gap-2.5 bg-neutral-900/85 backdrop-blur-md px-3 py-1.5 rounded-lg border border-neutral-800 text-[10px] font-mono shadow-md">
        <span className="flex items-center gap-1 text-emerald-400">
          <span className="w-2 h-2 rounded-full bg-emerald-500" /> &gt;1.0m
        </span>
        <span className="flex items-center gap-1 text-amber-400">
          <span className="w-2 h-2 rounded-full bg-amber-500" /> 0.4-1.0m
        </span>
        <span className="flex items-center gap-1 text-red-400">
          <span className="w-2 h-2 rounded-full bg-red-500 animate-pulse" /> &lt;0.4m Hazard
        </span>
      </div>
    </div>
  );
};
