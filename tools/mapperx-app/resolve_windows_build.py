"""Resolve the local wheel closure for CPython 3.12 Windows x64, without running Windows code."""
from collections import deque
from email.parser import BytesParser
import hashlib
import json
from pathlib import Path
import zipfile

from packaging.markers import default_environment
from packaging.requirements import Requirement
from packaging.tags import compatible_tags, cpython_tags
from packaging.utils import canonicalize_name, parse_wheel_filename


def main():
    app = Path(__file__).resolve().parent
    allowed = set(cpython_tags((3, 12), abis=["cp312"], platforms=["win_amd64"]))
    allowed.update(compatible_tags((3, 12), interpreter="cp312", platforms=["win_amd64"]))
    candidates = {}
    for folder in [app / "windows_build_wheels", app.parents[1] / "mapperx_wheels"]:
        for path in folder.glob("*.whl"):
            name, version, _build, tags = parse_wheel_filename(path.name)
            if not tags.intersection(allowed):
                continue
            with zipfile.ZipFile(path) as archive:
                metadata = next(n for n in archive.namelist() if n.endswith(".dist-info/METADATA"))
                data = BytesParser().parsebytes(archive.read(metadata))
            candidates.setdefault(canonicalize_name(name), []).append((version, path, data))
    environment = default_environment()
    environment.update(os_name="nt", sys_platform="win32", platform_system="Windows", platform_machine="AMD64",
                       python_version="3.12", python_full_version="3.12.12", implementation_name="cpython", extra="")
    queue = deque(Requirement(name) for name in ["pyinstaller", "pillow", "pefile", "pywin32-ctypes", "pywebview",
                                                 "pynetworktables", "numpy", "opencv-python-headless", "pythonnet", "clr_loader", "cffi"])
    constraints, selected = {}, {}
    while queue:
        requirement = queue.popleft()
        if requirement.marker and not requirement.marker.evaluate(environment):
            continue
        name = canonicalize_name(requirement.name)
        constraints.setdefault(name, []).append(requirement.specifier)
        valid = [item for item in candidates.get(name, []) if all(item[0] in s for s in constraints[name])]
        if not valid:
            raise ValueError("Missing compatible offline Windows wheel: " + str(requirement))
        item = max(valid, key=lambda value: value[0])
        if selected.get(name) == item:
            continue
        selected[name] = item
        queue.extend(Requirement(text) for text in item[2].get_all("Requires-Dist", []))
    lines, audit = [], []
    for name, (version, path, _metadata) in sorted(selected.items()):
        with path.open("rb") as source:
            checksum = hashlib.file_digest(source, "sha256").hexdigest()
        lines.append(f"{name}=={version} --hash=sha256:{checksum}")
        audit.append({"package": name, "version": str(version), "wheel": path.name, "sha256": checksum})
    (app / "windows-build.lock").write_text("\n".join(lines) + "\n")
    (app / "windows-build-dependencies.json").write_text(json.dumps(audit, indent=2) + "\n")
    print("Windows x64 / CPython 3.12 offline closure verified:", len(selected), "packages")


if __name__ == "__main__":
    main()
