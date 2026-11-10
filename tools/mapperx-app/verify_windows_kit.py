"""Verify Windows build-kit inputs before invoking pip or PyInstaller."""
import hashlib
import json
from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[2]
    manifest = root / "WINDOWS_BUILD_MANIFEST.json"
    if not manifest.is_file():
        print("Building from the project checkout (no distribution-kit manifest).")
        return
    records = json.loads(manifest.read_text(encoding="utf-8"))["files"]
    for relative, expected in records.items():
        path = (root / relative).resolve()
        if not path.is_relative_to(root) or not path.is_file() or path.stat().st_size != expected["size"]:
            raise ValueError("Missing or invalid build input: " + relative)
        with path.open("rb") as source:
            checksum = hashlib.file_digest(source, "sha256").hexdigest()
        if checksum != expected["sha256"]:
            raise ValueError("Damaged build input: " + relative)
    print("Build-kit SHA-256 verification passed:", len(records), "files")


if __name__ == "__main__":
    main()
