import React, { useState } from "react";
import {
  FileText,
  FileCode,
  Download,
  Copy,
  Check,
  Search,
  ExternalLink,
  Code2,
  FolderTree,
  FileBox
} from "lucide-react";
import { CODE_FILES } from "../data/codeFiles";
import { CodeFile } from "../types";
import { downloadSingleFile, downloadProjectZip } from "../utils/zipGenerator";

export const CodeExplorer: React.FC = () => {
  const [selectedFile, setSelectedFile] = useState<CodeFile>(CODE_FILES[0]);
  const [searchQuery, setSearchQuery] = useState<string>("");
  const [copied, setCopied] = useState<boolean>(false);

  const filteredFiles = CODE_FILES.filter(
    (f) =>
      f.name.toLowerCase().includes(searchQuery.toLowerCase()) ||
      f.category.toLowerCase().includes(searchQuery.toLowerCase()) ||
      f.description.toLowerCase().includes(searchQuery.toLowerCase())
  );

  const handleCopy = () => {
    navigator.clipboard.writeText(selectedFile.content);
    setCopied(true);
    setTimeout(() => setCopied(false), 2000);
  };

  const handleDownloadCurrent = () => {
    downloadSingleFile(selectedFile.name, selectedFile.content);
  };

  // Group files by category
  const pythonFiles = filteredFiles.filter((f) => f.category === "python");
  const arduinoFiles = filteredFiles.filter((f) => f.category === "arduino");
  const configDocsFiles = filteredFiles.filter(
    (f) => f.category === "config" || f.category === "docs"
  );

  return (
    <div className="max-w-7xl mx-auto px-4 py-4 flex flex-col gap-4 text-neutral-100 font-sans">
      {/* Top Banner */}
      <div className="flex flex-wrap items-center justify-between gap-3 bg-neutral-900 border border-neutral-800 p-4 rounded-xl shadow-sm">
        <div>
          <h2 className="text-lg font-bold text-neutral-100 flex items-center gap-2">
            <Code2 className="w-5 h-5 text-cyan-400" />
            <span>Python & Arduino Firmware Codebase</span>
          </h2>
          <p className="text-xs text-neutral-400 mt-0.5">
            100% production-ready, modular source files for the REACT rover mapping system.
          </p>
        </div>

        <button
          onClick={downloadProjectZip}
          className="flex items-center gap-2 px-4 py-2 rounded-lg bg-cyan-600 hover:bg-cyan-500 text-neutral-950 font-bold text-xs transition-colors shadow-sm"
        >
          <Download className="w-4 h-4" />
          <span>Download Complete ZIP Package</span>
        </button>
      </div>

      {/* Main Split Layout: Sidebar File Tree + Code Display */}
      <div className="grid grid-cols-1 lg:grid-cols-12 gap-4">
        {/* Left Sidebar: File Directory */}
        <div className="lg:col-span-4 bg-neutral-900 border border-neutral-800 rounded-xl p-3.5 flex flex-col gap-3 h-[680px]">
          {/* Search Box */}
          <div className="relative">
            <Search className="w-3.5 h-3.5 absolute left-3 top-2.5 text-neutral-500" />
            <input
              type="text"
              placeholder="Search files..."
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
              className="w-full pl-8 pr-3 py-1.5 bg-neutral-950 border border-neutral-800 rounded-lg text-xs text-neutral-200 placeholder-neutral-500 focus:outline-hidden focus:border-cyan-500"
            />
          </div>

          <div className="flex-1 overflow-y-auto space-y-4 pr-1 text-xs">
            {/* Python Modules */}
            {pythonFiles.length > 0 && (
              <div>
                <div className="text-[10px] font-mono font-semibold uppercase text-cyan-400 tracking-wider px-2 py-1 flex items-center gap-1.5">
                  <FolderTree className="w-3 h-3" />
                  <span>Python Mapping Modules</span>
                </div>
                <div className="space-y-1 mt-1">
                  {pythonFiles.map((file) => (
                    <button
                      key={file.name}
                      onClick={() => setSelectedFile(file)}
                      className={`w-full text-left px-2.5 py-1.5 rounded-lg flex items-center justify-between transition-colors ${
                        selectedFile.name === file.name
                          ? "bg-neutral-800 text-cyan-400 font-semibold border border-neutral-700"
                          : "text-neutral-300 hover:bg-neutral-800/60"
                      }`}
                    >
                      <div className="flex items-center gap-2 truncate">
                        <FileCode className="w-3.5 h-3.5 text-neutral-400 shrink-0" />
                        <span className="truncate">{file.name}</span>
                      </div>
                      <span className="text-[9px] font-mono text-neutral-500 shrink-0">
                        {file.content.split("\n").length}L
                      </span>
                    </button>
                  ))}
                </div>
              </div>
            )}

            {/* Arduino Firmware */}
            {arduinoFiles.length > 0 && (
              <div>
                <div className="text-[10px] font-mono font-semibold uppercase text-emerald-400 tracking-wider px-2 py-1 flex items-center gap-1.5">
                  <FileBox className="w-3 h-3" />
                  <span>Arduino Firmware (C++)</span>
                </div>
                <div className="space-y-1 mt-1">
                  {arduinoFiles.map((file) => (
                    <button
                      key={file.name}
                      onClick={() => setSelectedFile(file)}
                      className={`w-full text-left px-2.5 py-1.5 rounded-lg flex items-center justify-between transition-colors ${
                        selectedFile.name === file.name
                          ? "bg-neutral-800 text-emerald-400 font-semibold border border-neutral-700"
                          : "text-neutral-300 hover:bg-neutral-800/60"
                      }`}
                    >
                      <div className="flex items-center gap-2 truncate">
                        <FileCode className="w-3.5 h-3.5 text-emerald-500 shrink-0" />
                        <span className="truncate">{file.name}</span>
                      </div>
                      <span className="text-[9px] font-mono text-neutral-500 shrink-0">
                        {file.content.split("\n").length}L
                      </span>
                    </button>
                  ))}
                </div>
              </div>
            )}

            {/* Configuration & Documentation */}
            {configDocsFiles.length > 0 && (
              <div>
                <div className="text-[10px] font-mono font-semibold uppercase text-amber-400 tracking-wider px-2 py-1 flex items-center gap-1.5">
                  <FileText className="w-3 h-3" />
                  <span>Config & Setup</span>
                </div>
                <div className="space-y-1 mt-1">
                  {configDocsFiles.map((file) => (
                    <button
                      key={file.name}
                      onClick={() => setSelectedFile(file)}
                      className={`w-full text-left px-2.5 py-1.5 rounded-lg flex items-center justify-between transition-colors ${
                        selectedFile.name === file.name
                          ? "bg-neutral-800 text-amber-400 font-semibold border border-neutral-700"
                          : "text-neutral-300 hover:bg-neutral-800/60"
                      }`}
                    >
                      <div className="flex items-center gap-2 truncate">
                        <FileText className="w-3.5 h-3.5 text-amber-500 shrink-0" />
                        <span className="truncate">{file.name}</span>
                      </div>
                      <span className="text-[9px] font-mono text-neutral-500 shrink-0">
                        {file.content.split("\n").length}L
                      </span>
                    </button>
                  ))}
                </div>
              </div>
            )}
          </div>
        </div>

        {/* Right Pane: Code Viewer & Header */}
        <div className="lg:col-span-8 bg-neutral-900 border border-neutral-800 rounded-xl p-4 flex flex-col h-[680px]">
          {/* Header */}
          <div className="flex flex-wrap items-center justify-between gap-2 pb-3 border-b border-neutral-800">
            <div>
              <div className="flex items-center gap-2">
                <span className="font-mono font-bold text-sm text-neutral-100">
                  {selectedFile.path}
                </span>
                <span className="text-[10px] font-mono uppercase px-2 py-0.5 rounded bg-neutral-800 text-cyan-400 border border-neutral-700">
                  {selectedFile.language}
                </span>
              </div>
              <p className="text-xs text-neutral-400 mt-1 max-w-xl">
                {selectedFile.description}
              </p>
            </div>

            <div className="flex items-center gap-2">
              <button
                onClick={handleCopy}
                className="flex items-center gap-1.5 px-3 py-1.5 rounded-lg bg-neutral-800 hover:bg-neutral-700 text-neutral-300 text-xs border border-neutral-700 transition-colors"
                title="Copy entire file to clipboard"
              >
                {copied ? <Check className="w-3.5 h-3.5 text-emerald-400" /> : <Copy className="w-3.5 h-3.5" />}
                <span>{copied ? "Copied!" : "Copy Code"}</span>
              </button>

              <button
                onClick={handleDownloadCurrent}
                className="flex items-center gap-1.5 px-3 py-1.5 rounded-lg bg-neutral-800 hover:bg-neutral-700 text-neutral-300 text-xs border border-neutral-700 transition-colors"
                title="Download this single file"
              >
                <Download className="w-3.5 h-3.5" />
                <span>Save File</span>
              </button>
            </div>
          </div>

          {/* Syntax Code Box with Line Numbers */}
          <div className="flex-1 overflow-auto mt-3 bg-neutral-950 p-3 rounded-lg border border-neutral-800 font-mono text-xs leading-relaxed">
            <div className="flex">
              {/* Line numbers */}
              <div className="select-none text-neutral-600 text-right pr-4 border-r border-neutral-800 font-mono">
                {selectedFile.content.split("\n").map((_, i) => (
                  <div key={i}>{i + 1}</div>
                ))}
              </div>

              {/* Code content */}
              <div className="pl-4 text-neutral-300 whitespace-pre overflow-x-auto">
                {selectedFile.content}
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};
