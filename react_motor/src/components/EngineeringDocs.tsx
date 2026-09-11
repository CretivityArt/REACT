import React, { useState } from "react";
import {
  BookOpen,
  Search,
  ChevronRight,
  Terminal,
  Cpu,
  Compass,
  AlertTriangle,
  Layers,
  Wrench,
  CheckCircle2,
  Copy,
  Check
} from "lucide-react";
import { ENGINEERING_DOCS, DocSection } from "../data/documentation";

export const EngineeringDocs: React.FC = () => {
  const [activeSectionId, setActiveSectionId] = useState<string>(ENGINEERING_DOCS[0].id);
  const [searchQuery, setSearchQuery] = useState<string>("");
  const [copiedCode, setCopiedCode] = useState<string | null>(null);

  const filteredDocs = ENGINEERING_DOCS.filter(
    (d) =>
      d.title.toLowerCase().includes(searchQuery.toLowerCase()) ||
      d.summary.toLowerCase().includes(searchQuery.toLowerCase()) ||
      d.content.toLowerCase().includes(searchQuery.toLowerCase())
  );

  const activeSection =
    ENGINEERING_DOCS.find((d) => d.id === activeSectionId) || ENGINEERING_DOCS[0];

  const handleCopy = (text: string, id: string) => {
    navigator.clipboard.writeText(text);
    setCopiedCode(id);
    setTimeout(() => setCopiedCode(null), 2000);
  };

  return (
    <div className="max-w-7xl mx-auto px-4 py-4 flex flex-col gap-4 text-neutral-100 font-sans">
      {/* Top Header */}
      <div className="bg-neutral-900 border border-neutral-800 p-4 rounded-xl shadow-sm flex flex-col sm:flex-row items-start sm:items-center justify-between gap-3">
        <div>
          <h2 className="text-lg font-bold text-neutral-100 flex items-center gap-2">
            <BookOpen className="w-5 h-5 text-cyan-400" />
            <span>14 Engineering Deliverables & Technical Manual</span>
          </h2>
          <p className="text-xs text-neutral-400 mt-0.5">
            Complete mathematical derivations, hardware wiring, calibration procedures, and deployment instructions.
          </p>
        </div>

        <div className="text-xs font-mono bg-neutral-950 px-3 py-1.5 rounded-lg border border-neutral-800 text-neutral-400">
          Target Platform: <strong className="text-cyan-400">Arduino Uno + Windows 10/11</strong>
        </div>
      </div>

      {/* Main Split Layout: Table of Contents + Active Document Reader */}
      <div className="grid grid-cols-1 lg:grid-cols-12 gap-4">
        {/* Left: Table of Contents (14 Deliverables) */}
        <div className="lg:col-span-4 bg-neutral-900 border border-neutral-800 rounded-xl p-3.5 flex flex-col gap-3 h-[700px]">
          <div className="relative">
            <Search className="w-3.5 h-3.5 absolute left-3 top-2.5 text-neutral-500" />
            <input
              type="text"
              placeholder="Search deliverables..."
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
              className="w-full pl-8 pr-3 py-1.5 bg-neutral-950 border border-neutral-800 rounded-lg text-xs text-neutral-200 placeholder-neutral-500 focus:outline-hidden focus:border-cyan-500"
            />
          </div>

          <div className="flex-1 overflow-y-auto space-y-1.5 pr-1 text-xs">
            {filteredDocs.map((doc) => {
              const isSelected = doc.id === activeSection.id;
              return (
                <button
                  key={doc.id}
                  onClick={() => setActiveSectionId(doc.id)}
                  className={`w-full text-left p-2.5 rounded-lg border transition-all flex flex-col gap-1 ${
                    isSelected
                      ? "bg-neutral-800 border-cyan-500/50 text-neutral-100 shadow-xs"
                      : "bg-neutral-950/40 border-neutral-800/80 text-neutral-400 hover:bg-neutral-800/50 hover:text-neutral-200"
                  }`}
                >
                  <div className="flex items-center justify-between">
                    <span
                      className={`text-[9px] font-mono uppercase px-1.5 py-0.5 rounded ${
                        isSelected
                          ? "bg-cyan-950 text-cyan-400 border border-cyan-500/40"
                          : "bg-neutral-800 text-neutral-400"
                      }`}
                    >
                      {doc.badge}
                    </span>
                    <ChevronRight
                      className={`w-3.5 h-3.5 transition-transform ${
                        isSelected ? "text-cyan-400 translate-x-0.5" : "text-neutral-600"
                      }`}
                    />
                  </div>
                  <span className="font-semibold text-xs leading-snug line-clamp-1">
                    {doc.title}
                  </span>
                  <p className="text-[11px] text-neutral-500 line-clamp-2 leading-relaxed">
                    {doc.summary}
                  </p>
                </button>
              );
            })}
          </div>
        </div>

        {/* Right: Rich Document Reader */}
        <div className="lg:col-span-8 bg-neutral-900 border border-neutral-800 rounded-xl p-5 flex flex-col h-[700px] overflow-y-auto">
          {/* Header */}
          <div className="pb-4 border-b border-neutral-800">
            <div className="flex items-center gap-2 mb-1.5">
              <span className="text-[10px] font-mono uppercase px-2 py-0.5 rounded bg-cyan-950 text-cyan-400 border border-cyan-500/40 font-semibold">
                {activeSection.badge}
              </span>
              <span className="text-xs text-neutral-500">Deliverable #{activeSection.id}</span>
            </div>
            <h3 className="text-xl font-bold text-neutral-100 tracking-tight">
              {activeSection.title}
            </h3>
            <p className="text-xs text-neutral-400 mt-1">
              {activeSection.summary}
            </p>
          </div>

          {/* Markdown-style Rendered Content */}
          <div className="pt-4 text-xs leading-relaxed text-neutral-300 space-y-4 font-sans">
            {activeSection.content.split("\n\n").map((block, idx) => {
              // Heading level 3
              if (block.startsWith("### ")) {
                return (
                  <h4 key={idx} className="text-sm font-bold text-cyan-400 mt-4 mb-1">
                    {block.replace("### ", "")}
                  </h4>
                );
              }
              // Heading level 4
              if (block.startsWith("#### ")) {
                return (
                  <h5 key={idx} className="text-xs font-semibold text-neutral-200 mt-2 mb-1">
                    {block.replace("#### ", "")}
                  </h5>
                );
              }
              // Code block
              if (block.startsWith("```")) {
                const lines = block.split("\n");
                const lang = lines[0].replace("```", "").trim();
                const codeBody = lines.slice(1, -1).join("\n");
                const blockId = `code-${activeSection.id}-${idx}`;

                return (
                  <div key={idx} className="relative my-2 rounded-lg bg-neutral-950 border border-neutral-800 font-mono text-[11px] overflow-hidden">
                    <div className="flex items-center justify-between px-3 py-1.5 bg-neutral-900/80 border-b border-neutral-800 text-[10px] text-neutral-400">
                      <span className="uppercase font-semibold">{lang || "code"}</span>
                      <button
                        onClick={() => handleCopy(codeBody, blockId)}
                        className="flex items-center gap-1 hover:text-neutral-200 transition-colors"
                      >
                        {copiedCode === blockId ? (
                          <>
                            <Check className="w-3 h-3 text-emerald-400" />
                            <span className="text-emerald-400">Copied</span>
                          </>
                        ) : (
                          <>
                            <Copy className="w-3 h-3" />
                            <span>Copy</span>
                          </>
                        )}
                      </button>
                    </div>
                    <pre className="p-3 text-neutral-300 overflow-x-auto whitespace-pre">
                      {codeBody}
                    </pre>
                  </div>
                );
              }

              // Markdown table format
              if (block.includes("| :---") || (block.startsWith("|") && block.includes("|"))) {
                const rows = block.split("\n").filter((r) => r.trim().length > 0);
                const headerCols = rows[0]
                  .split("|")
                  .map((c) => c.trim())
                  .filter(Boolean);
                const dataRows = rows.slice(2);

                return (
                  <div key={idx} className="overflow-x-auto my-3 rounded-lg border border-neutral-800">
                    <table className="w-full text-left border-collapse text-xs">
                      <thead>
                        <tr className="bg-neutral-950 border-b border-neutral-800 text-neutral-300">
                          {headerCols.map((col, cIdx) => (
                            <th key={cIdx} className="p-2 font-semibold font-mono text-[11px]">
                              {col.replace(/\*\*/g, "")}
                            </th>
                          ))}
                        </tr>
                      </thead>
                      <tbody className="divide-y divide-neutral-800/60 font-mono text-[11px]">
                        {dataRows.map((r, rIdx) => {
                          const cols = r
                            .split("|")
                            .map((c) => c.trim())
                            .filter(Boolean);
                          return (
                            <tr key={rIdx} className="hover:bg-neutral-800/30">
                              {cols.map((cell, cIdx) => (
                                <td key={cIdx} className="p-2 text-neutral-300">
                                  {cell.replace(/\*\*/g, "")}
                                </td>
                              ))}
                            </tr>
                          );
                        })}
                      </tbody>
                    </table>
                  </div>
                );
              }

              // Unordered list
              if (block.startsWith("* ") || block.startsWith("- ")) {
                const items = block.split("\n").filter((i) => i.trim());
                return (
                  <ul key={idx} className="space-y-1 my-1.5 pl-4 list-disc marker:text-cyan-500">
                    {items.map((item, iIdx) => (
                      <li key={iIdx} className="text-neutral-300">
                        {item.replace(/^[*|-]\s+/, "")}
                      </li>
                    ))}
                  </ul>
                );
              }

              // Standard text paragraph
              return (
                <p key={idx} className="text-neutral-300 leading-relaxed">
                  {block}
                </p>
              );
            })}
          </div>
        </div>
      </div>
    </div>
  );
};
