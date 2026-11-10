"""Prepare a complete offline main file and a manifest without modifying project sources."""
import ast
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys
from project_snapshot import inventory, vault_block

APP = Path(__file__).resolve().parent
PROJECT = APP.parents[1]
PAYLOAD = APP / "bundle_payload"


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def main():
    PAYLOAD.mkdir(exist_ok=True)
    spec = importlib.util.spec_from_file_location("mapperx_pack_source", PROJECT / "field_planner.py")
    planner = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = planner
    spec.loader.exec_module(planner)
    snapshot = inventory(PROJECT)
    snapshot_path = PAYLOAD / "project-snapshot.json"
    snapshot_path.write_text(json.dumps(snapshot, indent=2) + "\n")
    code, _old_vault = planner._split_self()
    vault = vault_block(PROJECT, snapshot)
    # Native apps and exported offline files must restore the current project,
    # including the Java launcher, rather than a previously embedded backup.
    original = (PROJECT / "field_planner.py").read_text(encoding="utf-8")
    vault_at = original.rfind("\n" + planner.VAULT_BEGIN + "\n")
    native_source = (original[:vault_at + 1] if vault_at >= 0 else original).rstrip("\n") + "\n\n" + vault
    generated = APP / "generated"
    generated.mkdir(exist_ok=True)
    (generated / "field_planner.py").write_text(native_source, encoding="utf-8")
    wheels = sorted(planner.WHEELS_DIR.glob("*.whl"))
    pythons = sorted(planner.PYTHONS_DIR.glob("*.tar.gz"))
    required = {"aarch64-apple-darwin.tar.gz", "x86_64-apple-darwin.tar.gz", "x86_64-pc-windows-msvc.tar.gz"}
    if not required.issubset({p.name for p in pythons}) or not wheels:
        raise ValueError("Missing portable Python archives or offline wheels")
    # A content stamp avoids rewriting almost a gigabyte on each icon/UI build.
    prerequisites = [APP / "windows_prerequisites" / n for n in ["WebView2-x64.exe", "NET-Framework-4.8.exe"]]
    if any(not p.is_file() for p in prerequisites):
        raise ValueError("Run fetch_windows_prereqs.py once before preparing the full offline payload")
    prerequisite_records = {p.name: {"size": p.stat().st_size, "sha256": digest(p)} for p in prerequisites}
    inputs = [PROJECT / "field_planner.py", Path(__file__), APP / "project_snapshot.py", snapshot_path,
              *wheels, *pythons, *prerequisites]
    stamp = hashlib.sha256(json.dumps([(str(p.relative_to(PROJECT)), digest(p)) for p in inputs]).encode()).hexdigest()
    offline = PAYLOAD / "offline"
    offline.mkdir(exist_ok=True)
    main_file = offline / "MapperX_offline.py"
    stamp_file = PAYLOAD / ".source-stamp"
    payload_hash_file = PAYLOAD / ".payload-hashes.json"
    if not main_file.exists() or not stamp_file.exists() or stamp_file.read_text() != stamp or not payload_hash_file.exists():
        payload_records = []
        original_block_writer = planner._b64_block

        def record_block(out, begin, end, meta, source):
            offset = source.tell()
            checksum = hashlib.file_digest(source, "sha256").hexdigest()
            length = source.tell() - offset
            source.seek(offset)
            payload_records.append({"begin": begin, "end": end, "sha256": checksum, "size": length})
            original_block_writer(out, begin, end, meta, source)

        planner._b64_block = record_block
        # Only the generated distribution copy changes; original embedded blocks remain intact.
        code = code.replace("def setup_install(packages, out=print, online=True):",
                            "def setup_install(packages, out=print, online=False):")
        code = code.replace('webview.create_window("MapperX Board",', 'webview.create_window("Shuffleboard",')
        temporary = main_file.with_suffix(".tmp")
        planner._write_offline(temporary, wheels, pythons, "", code)
        with temporary.open("a", encoding="utf-8", newline="\n") as out:
            for prerequisite in prerequisites:
                name = prerequisite.name
                with prerequisite.open("rb") as source:
                    planner._b64_block(out, "# ==== SHUFFLEBOARD PREREQUISITE " + name + " BEGIN ====",
                                       "# ==== SHUFFLEBOARD PREREQUISITE " + name + " END ====",
                                       prerequisite_records[name], source)
            # Vault must remain last: existing update/backup code preserves everything before it.
            if vault:
                out.write("\n\n" + vault.lstrip("\n"))
        temporary.replace(main_file)
        payload_hash_file.write_text(json.dumps(payload_records))
        stamp_file.write_text(stamp)
    payload_records = json.loads(payload_hash_file.read_text())
    main_hash = digest(main_file)
    verifier = (APP / "offline_verify.py").read_text().replace("@@PAYLOAD_RECORDS@@", json.dumps(payload_records))
    verifier = verifier.replace("@@MAIN_SHA256@@", main_hash)
    verifier_file = offline / "Verify-Payloads.py"
    verifier_file.write_text(verifier)
    verifier_hash = digest(verifier_file)
    system_script = (APP / "offline_system_check.ps1").read_text()
    system_script = system_script.replace("@@NET_HASH@@", prerequisite_records["NET-Framework-4.8.exe"]["sha256"])
    system_script = system_script.replace("@@WEBVIEW_HASH@@", prerequisite_records["WebView2-x64.exe"]["sha256"])
    (offline / "Check-System.ps1").write_text(system_script)
    system_hash = digest(offline / "Check-System.ps1")
    system_command = ('powershell -NoProfile -Command "$p=Join-Path (Split-Path $env:PYFILE) '
                      "'Check-System.ps1'; "
                      "if ((Get-FileHash -LiteralPath $p -Algorithm SHA256).Hash -ne '" + system_hash
                      + "') { exit 13 }; "
                      '& ([scriptblock]::Create([IO.File]::ReadAllText($p)))"\n')
    verifier_command = ('powershell -NoProfile -Command "$v=Join-Path (Split-Path $env:PYFILE) '
                        "'Verify-Payloads.py'; "
                        "if ((Get-FileHash -LiteralPath $v -Algorithm SHA256).Hash -ne '" + verifier_hash
                        + "') { exit 13 }\"\n")
    win = planner.LAUNCH_WIN.replace('set "PYFILE=%~dp0MapperX_offline.py"',
        'set "PYFILE=%~dp0MapperX_offline.py"\n'
        + verifier_command +
        'if errorlevel 1 goto integrity_failed\n'
        + system_command +
        'if errorlevel 1 goto system_attention')
    win += '\n:integrity_failed\necho Offline main file is damaged. Copy the complete package again.\npause\nexit /b 13\n'
    win += '\n:system_attention\necho System setup needs attention. See messages above.\npause\nexit /b 1\n'
    win = win.replace('echo Python: %PY%', '"%PY%" "%~dp0Verify-Payloads.py" "%PYFILE%"\nif errorlevel 1 goto integrity_failed\necho Python: %PY%')
    mac = planner.LAUNCH_MAC.replace('PYFILE="MapperX_offline.py"',
        'PYFILE="MapperX_offline.py"\n'
        'ACTUAL=$(shasum -a 256 "Verify-Payloads.py" | cut -d " " -f 1)\n'
        '[ "$ACTUAL" = "' + verifier_hash + '" ] || { echo "Offline verifier is damaged"; exit 13; }')
    mac = mac.replace('"$PY" "$PYFILE" --app', '"$PY" "Verify-Payloads.py" "$PYFILE" || exit 13\n"$PY" "$PYFILE" --app')
    (offline / "Запустить MapperX (Windows).cmd").write_bytes(win.replace("\r\n", "\n").replace("\n", "\r\n").encode())
    mac_path = offline / "Запустить MapperX (Mac).command"
    mac_path.write_text(mac)
    mac_path.chmod(0o755)
    (offline / "SHA256SUMS.txt").write_text(main_hash + "  MapperX_offline.py\n")
    # DetectX's helpers are dynamic imports and are needed even without a robot project.
    helpers = PAYLOAD / "vision_helpers"
    helpers.mkdir(exist_ok=True)
    helper_names = ["calibrate_hsv.py", "color_mask.py"]
    for name in helper_names:
        shutil.copy2(PROJECT / "src/main/deploy" / name, helpers / name)
    imports = set()
    for path in [PROJECT / "field_planner.py", *(helpers / n for n in helper_names)]:
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.Import):
                imports.update(n.name.split(".")[0] for n in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.add(node.module.split(".")[0])
    local = {"calibrate_hsv", "color_mask"}
    external = sorted(imports - set(sys.stdlib_module_names) - local)
    audit = {"external_imports": external, "wheels": len(wheels), "portable_python": sorted(required),
             "runtime_packages": ["pywebview", "pynetworktables", "numpy", "opencv-python-headless"],
             "windows_packages": ["pythonnet", "clr_loader", "cffi", "pycparser"],
             "windows_prerequisites": [".NET Framework >= 4.8", "WebView2 Runtime >= 86"],
             "offline_main_sha256": main_hash}
    (PAYLOAD / "dependency-audit.json").write_text(json.dumps(audit, indent=2) + "\n")
    # Keep the same launcher and documentation available in portable exports.
    for name in ["MapperX.java", "MAPPERX_LAUNCHER.md", "MAPPERX-NO-AI.txt"]:
        shutil.copy2(PROJECT / name, offline / name)
    entries = {"field_planner.py": generated / "field_planner.py"}
    entries.update({n: APP / n for n in ["splash.html", "WPILIB_LICENSE.txt", "icon.png", "icon.ico"]})
    for path in PAYLOAD.rglob("*"):
        if path.is_file() and path.name != ".source-stamp":
            entries["bundle_payload/" + path.relative_to(PAYLOAD).as_posix()] = path
    manifest = {"schema": 1, "prerequisites": prerequisite_records,
                "files": {name: {"size": path.stat().st_size, "sha256": digest(path)}
                                          for name, path in sorted(entries.items())}}
    manifest_file = APP / "bundle-manifest.json"
    manifest_file.write_text(json.dumps(manifest, indent=2) + "\n")
    # This digest is compiled into the frozen Python archive, separately from the JSON.
    (APP / "bundle_info.py").write_text('MANIFEST_SHA256 = "' + digest(manifest_file) + '"\n')
    print(json.dumps(audit, indent=2), flush=True)
    print("Offline main file:", main_file.stat().st_size, "bytes", flush=True)


if __name__ == "__main__":
    main()
