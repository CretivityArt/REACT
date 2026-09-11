import JSZip from "jszip";
import { CODE_FILES } from "../data/codeFiles";

export async function downloadProjectZip(): Promise<void> {
  const zip = new JSZip();
  const root = zip.folder("react_mapping");

  if (!root) return;

  // Add all Python files and configs to root
  for (const file of CODE_FILES) {
    if (file.path.startsWith("react_mapping/arduino/")) {
      const arduinoFolder = root.folder("arduino");
      if (arduinoFolder) {
        arduinoFolder.file(file.name, file.content);
      }
    } else {
      root.file(file.name, file.content);
    }
  }

  // Generate blob and trigger browser download
  const blob = await zip.generateAsync({ type: "blob" });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement("a");
  anchor.href = url;
  anchor.download = "react_rover_mapping_system.zip";
  document.body.appendChild(anchor);
  anchor.click();
  document.body.removeChild(anchor);
  URL.revokeObjectURL(url);
}

export function downloadSingleFile(filename: string, content: string): void {
  const blob = new Blob([content], { type: "text/plain;charset=utf-8" });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement("a");
  anchor.href = url;
  anchor.download = filename;
  document.body.appendChild(anchor);
  anchor.click();
  document.body.removeChild(anchor);
  URL.revokeObjectURL(url);
}
