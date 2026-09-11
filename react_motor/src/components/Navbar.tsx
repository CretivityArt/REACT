import React from "react";
import {
  Radio,
  FileCode,
  BookOpen,
  Sliders,
  Download,
  AlertOctagon,
  Cpu,
  RefreshCw,
  Terminal
} from "lucide-react";
import { NavTab, OperationMode } from "../types";
import { downloadProjectZip } from "../utils/zipGenerator";

interface NavbarProps {
  currentTab: NavTab;
  setTab: (tab: NavTab) => void;
  operationMode: OperationMode;
  packetCount: number;
  isStreaming: boolean;
  onEmergencyStop: () => void;
  onResetOrigin: () => void;
}

export const Navbar: React.FC<NavbarProps> = ({
  currentTab,
  setTab,
  operationMode,
  packetCount,
  isStreaming,
  onEmergencyStop,
  onResetOrigin
}) => {
  return (
    <header className="bg-neutral-900 border-b border-neutral-800 text-neutral-100 select-none sticky top-0 z-40">
      <div className="max-w-7xl mx-auto px-4 py-2.5 flex flex-wrap items-center justify-between gap-3">
        {/* Left: Branding & Status */}
        <div className="flex items-center gap-3">
          <div className="flex items-center justify-center w-9 h-9 rounded-lg bg-cyan-950/80 border border-cyan-500/40 text-cyan-400 font-black tracking-wider text-base shadow-sm">
            R
          </div>
          <div>
            <div className="flex items-center gap-2">
              <span className="font-bold text-base tracking-tight text-neutral-100">
                REACT ROVER
              </span>
              <span className="text-[10px] font-mono uppercase px-1.5 py-0.5 rounded bg-neutral-800 text-neutral-300 border border-neutral-700">
                GCS v2.4
              </span>
              <span
                className={`text-[10px] font-mono font-medium uppercase px-2 py-0.5 rounded-full flex items-center gap-1.5 ${
                  operationMode === "SIMULATION"
                    ? "bg-amber-950/70 text-amber-300 border border-amber-500/40"
                    : operationMode === "WEB_SERIAL"
                    ? "bg-emerald-950/70 text-emerald-300 border border-emerald-500/40"
                    : "bg-neutral-800 text-neutral-400 border border-neutral-700"
                }`}
              >
                <span
                  className={`w-1.5 h-1.5 rounded-full ${
                    isStreaming ? "bg-emerald-400 animate-pulse" : "bg-neutral-500"
                  }`}
                />
                {operationMode}
              </span>
            </div>
            <p className="text-xs text-neutral-400 tracking-tight hidden sm:block">
              Reactive Hazard Exploration & Underground Coal-Mine 2D Mapping
            </p>
          </div>
        </div>

        {/* Center: Navigation Tabs */}
        <nav className="flex items-center gap-1 bg-neutral-950 p-1 rounded-lg border border-neutral-800 text-xs font-medium">
          <button
            onClick={() => setTab("MONITOR")}
            className={`flex items-center gap-1.5 px-3 py-1.5 rounded-md transition-all ${
              currentTab === "MONITOR"
                ? "bg-neutral-800 text-cyan-400 font-semibold shadow-xs"
                : "text-neutral-400 hover:text-neutral-200 hover:bg-neutral-900"
            }`}
          >
            <Radio className="w-3.5 h-3.5" />
            <span>2D Map & Telemetry</span>
          </button>

          <button
            onClick={() => setTab("CODE_EXPLORER")}
            className={`flex items-center gap-1.5 px-3 py-1.5 rounded-md transition-all ${
              currentTab === "CODE_EXPLORER"
                ? "bg-neutral-800 text-cyan-400 font-semibold shadow-xs"
                : "text-neutral-400 hover:text-neutral-200 hover:bg-neutral-900"
            }`}
          >
            <FileCode className="w-3.5 h-3.5" />
            <span>Python & Arduino Code</span>
          </button>

          <button
            onClick={() => setTab("ENGINEERING_DOCS")}
            className={`flex items-center gap-1.5 px-3 py-1.5 rounded-md transition-all ${
              currentTab === "ENGINEERING_DOCS"
                ? "bg-neutral-800 text-cyan-400 font-semibold shadow-xs"
                : "text-neutral-400 hover:text-neutral-200 hover:bg-neutral-900"
            }`}
          >
            <BookOpen className="w-3.5 h-3.5" />
            <span>14 Deliverables Manual</span>
          </button>

          <button
            onClick={() => setTab("CALIBRATION")}
            className={`flex items-center gap-1.5 px-3 py-1.5 rounded-md transition-all ${
              currentTab === "CALIBRATION"
                ? "bg-neutral-800 text-cyan-400 font-semibold shadow-xs"
                : "text-neutral-400 hover:text-neutral-200 hover:bg-neutral-900"
            }`}
          >
            <Sliders className="w-3.5 h-3.5" />
            <span>Calibration</span>
          </button>
        </nav>

        {/* Right: Actions */}
        <div className="flex items-center gap-2">
          <button
            onClick={downloadProjectZip}
            title="Download full Python package and Arduino sketch as a ready-to-run ZIP"
            className="flex items-center gap-1.5 px-3 py-1.5 rounded-lg bg-cyan-600 hover:bg-cyan-500 text-neutral-950 font-semibold text-xs transition-colors shadow-xs"
          >
            <Download className="w-3.5 h-3.5" />
            <span>Download ZIP</span>
          </button>

          <button
            onClick={onResetOrigin}
            title="Re-zero Rover pose and clear occupancy map"
            className="flex items-center gap-1 px-2.5 py-1.5 rounded-lg bg-neutral-800 hover:bg-neutral-700 text-neutral-300 text-xs border border-neutral-700 transition-colors"
          >
            <RefreshCw className="w-3 h-3" />
            <span className="hidden md:inline">Reset Origin</span>
          </button>

          <button
            onClick={onEmergencyStop}
            title="Emergency Motor Halt"
            className="flex items-center gap-1 px-2.5 py-1.5 rounded-lg bg-red-950 hover:bg-red-900 text-red-300 border border-red-800 text-xs font-semibold transition-colors"
          >
            <AlertOctagon className="w-3.5 h-3.5 text-red-400" />
            <span className="hidden sm:inline">E-STOP</span>
          </button>
        </div>
      </div>
    </header>
  );
};
