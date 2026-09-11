import React, { useState } from "react";
import {
  Sliders,
  RotateCw,
  Compass,
  CheckCircle2,
  Copy,
  Check,
  RefreshCw,
  Info
} from "lucide-react";
import { CalibrationOffsets } from "../types";

export const CalibrationStudio: React.FC = () => {
  const [offsets, setOffsets] = useState<CalibrationOffsets>({
    gyroBiasX: 0.12,
    gyroBiasY: -0.08,
    gyroBiasZ: 1.25,
    magOffsetX: 14.2,
    magOffsetY: -8.7,
    magOffsetZ: 5.1,
    magScaleX: 1.02,
    magScaleY: 0.98,
    magScaleZ: 1.00
  });

  const [isSamplingGyro, setIsSamplingGyro] = useState<boolean>(false);
  const [gyroSamplesCount, setGyroSamplesCount] = useState<number>(0);
  const [copied, setCopied] = useState<boolean>(false);

  const startGyroSampling = () => {
    setIsSamplingGyro(true);
    setGyroSamplesCount(0);

    let count = 0;
    const interval = setInterval(() => {
      count += 20;
      setGyroSamplesCount(count);
      if (count >= 200) {
        clearInterval(interval);
        setIsSamplingGyro(false);
        // Simulate sampled zero-rate mean
        setOffsets((prev) => ({
          ...prev,
          gyroBiasX: parseFloat((Math.random() * 0.3 - 0.15).toFixed(2)),
          gyroBiasY: parseFloat((Math.random() * 0.3 - 0.15).toFixed(2)),
          gyroBiasZ: parseFloat((1.1 + Math.random() * 0.3).toFixed(2))
        }));
      }
    }, 100);
  };

  const generatedPythonSnippet = `# Copy & paste into react_mapping/config.py
GYRO_BIAS_X = ${offsets.gyroBiasX}   # deg/s
GYRO_BIAS_Y = ${offsets.gyroBiasY}   # deg/s
GYRO_BIAS_Z = ${offsets.gyroBiasZ}   # deg/s

MAG_OFFSET_X = ${offsets.magOffsetX} # uT
MAG_OFFSET_Y = ${offsets.magOffsetY} # uT
MAG_OFFSET_Z = ${offsets.magOffsetZ} # uT

MAG_SCALE_X = ${offsets.magScaleX}
MAG_SCALE_Y = ${offsets.magScaleY}
MAG_SCALE_Z = ${offsets.magScaleZ}
`;

  const copyConfigSnippet = () => {
    navigator.clipboard.writeText(generatedPythonSnippet);
    setCopied(true);
    setTimeout(() => setCopied(false), 2000);
  };

  return (
    <div className="max-w-7xl mx-auto px-4 py-4 flex flex-col gap-4 text-neutral-100 font-sans">
      {/* Header */}
      <div className="bg-neutral-900 border border-neutral-800 p-4 rounded-xl shadow-sm flex items-center justify-between">
        <div>
          <h2 className="text-lg font-bold text-neutral-100 flex items-center gap-2">
            <Sliders className="w-5 h-5 text-cyan-400" />
            <span>Sensor Calibration & Hard-Iron Studio</span>
          </h2>
          <p className="text-xs text-neutral-400 mt-0.5">
            Zero-rate gyro bias nulling and magnetometer hard-iron offset compensation.
          </p>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-4">
        {/* 1. Gyroscope Zero-Rate Bias Calibration */}
        <div className="bg-neutral-900 border border-neutral-800 rounded-xl p-4 flex flex-col gap-3 shadow-sm">
          <div className="flex items-center justify-between pb-2 border-b border-neutral-800">
            <span className="font-bold text-sm text-neutral-100">
              1. Gyroscope Zero-Rate Bias Sampling
            </span>
            <span className="text-[10px] font-mono px-2 py-0.5 rounded bg-neutral-800 text-cyan-400">
              Stationary Test
            </span>
          </div>

          <p className="text-xs text-neutral-300 leading-relaxed">
            Ensure the rover is completely stationary on a flat floor. The system records 200 consecutive packets to determine constant angular drift rate:
          </p>

          <div className="grid grid-cols-3 gap-2 text-center text-xs font-mono">
            <div className="bg-neutral-950 p-2.5 rounded-lg border border-neutral-800">
              <span className="text-neutral-500 text-[10px] block">BIAS X</span>
              <span className="font-bold text-sm text-cyan-400">{offsets.gyroBiasX} °/s</span>
            </div>
            <div className="bg-neutral-950 p-2.5 rounded-lg border border-neutral-800">
              <span className="text-neutral-500 text-[10px] block">BIAS Y</span>
              <span className="font-bold text-sm text-cyan-400">{offsets.gyroBiasY} °/s</span>
            </div>
            <div className="bg-neutral-950 p-2.5 rounded-lg border border-neutral-800">
              <span className="text-neutral-500 text-[10px] block">BIAS Z</span>
              <span className="font-bold text-sm text-emerald-400">{offsets.gyroBiasZ} °/s</span>
            </div>
          </div>

          <button
            onClick={startGyroSampling}
            disabled={isSamplingGyro}
            className="w-full py-2 px-3 rounded-lg bg-cyan-600 hover:bg-cyan-500 disabled:bg-neutral-800 text-neutral-950 disabled:text-neutral-500 font-bold text-xs transition-colors flex items-center justify-center gap-2"
          >
            <RefreshCw className={`w-3.5 h-3.5 ${isSamplingGyro ? "animate-spin" : ""}`} />
            <span>
              {isSamplingGyro
                ? `Sampling stationary gyro (${gyroSamplesCount}/200 samples)...`
                : "Record Stationary Gyro Bias (200 Samples)"}
            </span>
          </button>
        </div>

        {/* 2. Magnetometer Hard-Iron Calibration */}
        <div className="bg-neutral-900 border border-neutral-800 rounded-xl p-4 flex flex-col gap-3 shadow-sm">
          <div className="flex items-center justify-between pb-2 border-b border-neutral-800">
            <span className="font-bold text-sm text-neutral-100">
              2. Magnetometer Hard-Iron Offsets
            </span>
            <span className="text-[10px] font-mono px-2 py-0.5 rounded bg-neutral-800 text-amber-400">
              360° Spin Method
            </span>
          </div>

          <p className="text-xs text-neutral-300 leading-relaxed">
            Motors and batteries generate stray DC magnetic fields. Rotate the rover 360° to find the circle center shift:
          </p>

          <div className="grid grid-cols-3 gap-2 text-center text-xs font-mono">
            <div className="bg-neutral-950 p-2.5 rounded-lg border border-neutral-800">
              <span className="text-neutral-500 text-[10px] block">OFFSET X</span>
              <span className="font-bold text-sm text-amber-400">{offsets.magOffsetX} µT</span>
            </div>
            <div className="bg-neutral-950 p-2.5 rounded-lg border border-neutral-800">
              <span className="text-neutral-500 text-[10px] block">OFFSET Y</span>
              <span className="font-bold text-sm text-amber-400">{offsets.magOffsetY} µT</span>
            </div>
            <div className="bg-neutral-950 p-2.5 rounded-lg border border-neutral-800">
              <span className="text-neutral-500 text-[10px] block">OFFSET Z</span>
              <span className="font-bold text-sm text-amber-400">{offsets.magOffsetZ} µT</span>
            </div>
          </div>

          <div className="flex items-center gap-2 bg-neutral-950 p-2 rounded-lg border border-neutral-800 text-[11px] text-neutral-400">
            <Info className="w-3.5 h-3.5 text-neutral-400 shrink-0" />
            <span>
              Scale Factors: X={offsets.magScaleX}, Y={offsets.magScaleY}, Z={offsets.magScaleZ}
            </span>
          </div>
        </div>
      </div>

      {/* 3. Generated config.py Snippet */}
      <div className="bg-neutral-900 border border-neutral-800 rounded-xl p-4 flex flex-col gap-3 shadow-sm">
        <div className="flex items-center justify-between pb-2 border-b border-neutral-800">
          <div>
            <span className="font-bold text-sm text-neutral-100">
              Exported Python Configuration
            </span>
            <span className="text-xs text-neutral-400 ml-2">
              (Directly replace calibration constants in config.py)
            </span>
          </div>

          <button
            onClick={copyConfigSnippet}
            className="flex items-center gap-1.5 px-3 py-1.5 rounded-lg bg-neutral-800 hover:bg-neutral-700 text-neutral-200 text-xs border border-neutral-700 transition-colors"
          >
            {copied ? <Check className="w-3.5 h-3.5 text-emerald-400" /> : <Copy className="w-3.5 h-3.5" />}
            <span>{copied ? "Copied to Clipboard" : "Copy Snippet"}</span>
          </button>
        </div>

        <pre className="bg-neutral-950 p-3.5 rounded-lg border border-neutral-800 font-mono text-xs text-cyan-300 leading-relaxed overflow-x-auto">
          {generatedPythonSnippet}
        </pre>
      </div>
    </div>
  );
};
