"""Inventory the robot project without recursively archiving app build caches."""
import base64
import hashlib
import io
import json
from pathlib import Path
import time
import zipfile

ROOT_FILES = ["field_planner.py", "MapperX.java", "MAPPERX_LAUNCHER.md", "MAPPERX-NO-AI.txt",
              "build.gradle", "settings.gradle", "gradlew", "gradlew.bat", ".gitignore"]
PROJECT_DIRS = ["src/main", "field_maps", "vendor", "gradle", "vendordeps", ".wpilib", ".vscode",
                "tools/vscode-mapperx"]
APP_SOURCES = ["mapperx_board.py", "bundle_bootstrap.py", "prepare_bundle.py", "project_snapshot.py",
               "make_windows_build_kit.py", "resolve_windows_build.py", "verify_windows_kit.py",
               "offline_verify.py", "offline_system_check.ps1", "package_release.py",
               "fetch_windows_prereqs.py", "make_icon.py", "build_mac.sh", "build_windows.bat",
               "windows-build.lock", "windows-build-dependencies.json", "splash.html", "icon.ico",
               "icon.png", "icon.icns", "ICON_SOURCE.md", "WPILIB_LICENSE.txt"]


def project_files(project):
    """Include current sources/settings/maps, excluding temporary files and recordings."""
    project = Path(project)
    files = {project / name for name in ROOT_FILES if (project / name).is_file()}
    files.update(project / "tools/mapperx-app" / name for name in APP_SOURCES
                 if (project / "tools/mapperx-app" / name).is_file())
    for name in PROJECT_DIRS:
        source = project / name
        if source.is_dir():
            files.update(p for p in source.rglob("*") if p.is_file())
    return sorted(p for p in files if not p.is_symlink()
                  and not any(part in {"__pycache__", "runs", "recordings"} for part in p.relative_to(project).parts)
                  and p.name != ".DS_Store" and p.suffix not in {".pyc", ".tmp", ".bak"})


def inventory(project):
    """Bind every release to exact current project bytes."""
    records = {}
    for path in project_files(project):
        with path.open("rb") as source:
            checksum = hashlib.file_digest(source, "sha256").hexdigest()
        records[path.relative_to(project).as_posix()] = {"size": path.stat().st_size, "sha256": checksum}
    return {"schema": 1, "files": records}


def vault_block(project, records):
    """Create a fresh restoration snapshot; the original planner file is untouched."""
    project = Path(project)
    buffer = io.BytesIO()
    total = 0
    names = [name for name in records["files"] if name != "field_planner.py"]
    with zipfile.ZipFile(buffer, "w", compression=zipfile.ZIP_LZMA) as archive:
        for name in names:
            path = project / name
            data = path.read_bytes()
            if hashlib.sha256(data).hexdigest() != records["files"][name]["sha256"]:
                raise ValueError("Project changed during packaging: " + name)
            total += len(data)
            archive.write(path, name)
    meta = {"created": time.strftime("%Y-%m-%d %H:%M:%S"), "files": len(names),
            "bytes": total, "project": project.name, "self": "field_planner.py"}
    encoded = base64.b64encode(buffer.getvalue()).decode("ascii")
    return ("# ==== MAPPERX VAULT BEGIN ====\n# meta " + json.dumps(meta, ensure_ascii=False) + "\n"
            + "".join("# " + encoded[i:i + 100] + "\n" for i in range(0, len(encoded), 100))
            + "# ==== MAPPERX VAULT END ====\n")
