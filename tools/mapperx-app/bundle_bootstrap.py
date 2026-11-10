"""Offline bootstrap and real dependency checks for the frozen application."""
import hashlib
import base64
import importlib
import json
import os
from pathlib import Path
import platform
import shutil
import struct
import subprocess
import sys
import tempfile


def sha256(path):
    with Path(path).open("rb") as source:
        return hashlib.file_digest(source, "sha256").hexdigest()


def resource_root():
    if getattr(sys, "frozen", False) and sys.platform == "darwin":
        return (Path(sys.executable).resolve().parent.parent / "Resources").resolve()
    return Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent)).resolve()


def read_manifest(root=None):
    from bundle_info import MANIFEST_SHA256
    root = Path(root or resource_root()).resolve()
    path = root / "bundle-manifest.json"
    if sha256(path) != MANIFEST_SHA256:
        raise ValueError("Повреждён манифест автономного пакета. Скопируйте приложение заново.")
    data = json.loads(path.read_text())
    if data.get("schema") != 1 or not isinstance(data.get("files"), dict):
        raise ValueError("Неизвестный формат автономного пакета")
    return data


def verify_files(root=None, manifest=None, prefix=None, progress=None):
    root = Path(root or resource_root()).resolve()
    manifest = manifest or read_manifest(root)
    records = [(name, info) for name, info in manifest["files"].items()
               if prefix is None or name.startswith(prefix)]
    if prefix is not None and not records:
        raise ValueError("В автономном пакете нет " + prefix)
    for index, (name, info) in enumerate(records):
        path = (root / name).resolve()
        if not path.is_relative_to(root):
            raise ValueError("Недопустимый путь в манифесте: " + name)
        if not path.is_file() or path.stat().st_size != info["size"] or sha256(path) != info["sha256"]:
            raise ValueError("Повреждён файл автономного пакета: " + name)
        if progress:
            progress(index + 1, len(records), name)
    return {"ok": True, "files": len(records)}


def platform_check():
    result = {"os": sys.platform, "version": platform.release(), "architecture": platform.machine(),
              "python": platform.python_version(), "bits": struct.calcsize("P") * 8, "errors": []}
    if result["bits"] != 64:
        result["errors"].append("Нужна 64-разрядная система")
    if sys.platform == "darwin":
        version = tuple(int(p) for p in platform.mac_ver()[0].split(".")[:2])
        if version < (14, 0):
            result["errors"].append("Этой сборке нужен macOS 14 или новее")
        if platform.machine() != "arm64":
            result["errors"].append("Эта .app собрана для Apple Silicon")
    elif sys.platform == "win32":
        version = sys.getwindowsversion()
        if version.major < 10 or version.build < 14393:
            result["errors"].append("Нужна Windows 10 версии 1607 или новее / Windows 11 x64")
    else:
        result["errors"].append("Сборка предназначена для Windows и macOS")
    result["ok"] = not result["errors"]
    return result


def windows_prerequisites():
    if sys.platform != "win32":
        return []
    import winreg
    missing = []
    try:
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"SOFTWARE\Microsoft\NET Framework Setup\NDP\v4\Full") as key:
            release = winreg.QueryValueEx(key, "Release")[0]
        if release < 528040:
            missing.append("NET-Framework-4.8.exe")
    except OSError:
        missing.append("NET-Framework-4.8.exe")
    guid = "{F3017226-FE2A-4295-8BDF-00C3A9A7E4C5}"
    present = False
    for hive, name in [(winreg.HKEY_CURRENT_USER, "SOFTWARE\\Microsoft\\EdgeUpdate\\Clients\\" + guid),
                       (winreg.HKEY_LOCAL_MACHINE, "SOFTWARE\\WOW6432Node\\Microsoft\\EdgeUpdate\\Clients\\" + guid)]:
        try:
            with winreg.OpenKey(hive, name) as key:
                version = str(winreg.QueryValueEx(key, "pv")[0])
            if int(version.split(".")[0]) >= 86:
                present = True
        except (OSError, ValueError):
            pass
    if not present:
        missing.append("WebView2-x64.exe")
    return missing


def ensure_system_prerequisites():
    system = platform_check()
    if not system["ok"]:
        raise ValueError("\n".join(system["errors"]))
    missing = windows_prerequisites()
    if not missing:
        return
    root = resource_root()
    manifest = read_manifest(root)
    verify_files(root, manifest, prefix="bundle_payload/offline/")
    for name in missing:
        installer = extract_prerequisite(name, root, manifest)
        # Full vendor installers contain their own UI and, where required, UAC/EULA.
        # No bootstrapper, pip index, or runtime download is used here.
        if name == "NET-Framework-4.8.exe":
            # This vendor installer declares requireAdministrator in its PE manifest.
            # ShellExecute via Start-Process presents the normal Windows UAC prompt.
            command = ("$ErrorActionPreference='Stop'; "
                       "$p=Start-Process -FilePath $env:SHUFFLEBOARD_OFFLINE_INSTALLER "
                       "-Verb RunAs -Wait -PassThru; exit $p.ExitCode")
            result = subprocess.run(["powershell.exe", "-NoProfile", "-Command", command], check=False,
                                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
                                    env=dict(os.environ, SHUFFLEBOARD_OFFLINE_INSTALLER=str(installer)))
        else:
            result = subprocess.run([str(installer)], check=False)
        if result.returncode in (3010, 1641):
            raise ValueError("Компонент установлен. Перезагрузите Windows и запустите Shuffleboard снова.")
        if result.returncode != 0:
            raise ValueError("Не удалось установить " + name + ": код " + str(result.returncode))
    remaining = windows_prerequisites()
    if remaining:
        raise ValueError("После установки ещё недоступно: " + ", ".join(remaining) + ". Перезагрузите Windows.")


def extract_prerequisite(name, root=None, manifest=None, cache=None):
    root = Path(root or resource_root()).resolve()
    manifest = manifest or read_manifest(root)
    record = manifest.get("prerequisites", {}).get(name)
    if name not in ("WebView2-x64.exe", "NET-Framework-4.8.exe") or not record:
        raise ValueError("Неизвестный системный компонент")
    base = Path(cache or os.environ.get("LOCALAPPDATA") or tempfile.gettempdir())
    target = base / "Shuffleboard/prerequisites" / record["sha256"][:16] / name
    if target.is_file() and target.stat().st_size == record["size"] and sha256(target) == record["sha256"]:
        return target
    target.parent.mkdir(parents=True, exist_ok=True)
    if shutil.disk_usage(target.parent).free < record["size"] * 3:
        raise ValueError("Недостаточно свободного места для установки " + name)
    begin = "# ==== SHUFFLEBOARD PREREQUISITE " + name + " BEGIN ===="
    end = "# ==== SHUFFLEBOARD PREREQUISITE " + name + " END ===="
    inside = complete = False
    chunks = []
    temporary = target.with_suffix(".tmp")
    with (root / "bundle_payload/offline/MapperX_offline.py").open(encoding="utf-8") as source, temporary.open("wb") as out:
        for raw in source:
            line = raw.rstrip("\r\n")
            if line == begin:
                inside = True
                continue
            if inside and line == end:
                complete = True
                break
            if inside and line.startswith("# ") and not line.startswith("# meta "):
                chunks.append(line[2:])
                if len(chunks) >= 4096:
                    out.write(base64.b64decode("".join(chunks), validate=True))
                    chunks.clear()
        if chunks:
            out.write(base64.b64decode("".join(chunks), validate=True))
    if not complete or temporary.stat().st_size != record["size"] or sha256(temporary) != record["sha256"]:
        raise ValueError("Повреждён встроенный установщик: " + name)
    temporary.replace(target)
    return target


def dependency_check():
    names = ["webview", "networktables", "numpy", "cv2"]
    names += ["AppKit", "WebKit"] if sys.platform == "darwin" else ["clr", "clr_loader", "cffi"]
    result = []
    for name in names:
        try:
            module = importlib.import_module(name)
            result.append({"module": name, "ok": True, "version": str(getattr(module, "__version__", "bundled"))})
        except Exception as exc:
            result.append({"module": name, "ok": False, "error": str(exc)})
    if all(r["ok"] for r in result):
        try:
            import cv2
            import numpy as np
            assert cv2.cvtColor(np.zeros((2, 2, 3), dtype=np.uint8), cv2.COLOR_BGR2HSV).shape == (2, 2, 3)
        except Exception as exc:
            result.append({"module": "cv2/numpy native operation", "ok": False, "error": str(exc)})
    return result


def self_check():
    report = {"system": platform_check(), "prerequisites_missing": windows_prerequisites(),
              "frozen": bool(getattr(sys, "frozen", False)), "executable": sys.executable}
    try:
        report["integrity"] = verify_files()
    except Exception as exc:
        report["integrity"] = {"ok": False, "error": str(exc)}
    report["dependencies"] = dependency_check()
    report["ok"] = (report["system"]["ok"] and report["integrity"]["ok"]
                    and not report["prerequisites_missing"] and all(r["ok"] for r in report["dependencies"]))
    return report


def show_error(message):
    text = "Shuffleboard не удалось запустить.\n\n" + str(message)
    log = Path(tempfile.gettempdir()) / "Shuffleboard-startup-error.txt"
    log.write_text(text, encoding="utf-8")
    if sys.platform == "win32":
        import ctypes
        ctypes.windll.user32.MessageBoxW(None, text, "Shuffleboard", 0x10)
    elif sys.platform == "darwin":
        import AppKit
        alert = AppKit.NSAlert.alloc().init()
        alert.setMessageText_("Shuffleboard")
        alert.setInformativeText_(text)
        alert.addButtonWithTitle_("Закрыть")
        alert.runModal()


def configure_planner(planner, project):
    """Keep the original planner intact; adapt packaging-specific routes here."""
    if project is None:
        # A writable workspace keeps calibration/autosave out of the signed .app.
        base = Path(os.environ.get("LOCALAPPDATA") or (Path.home() / "Library/Application Support"))
        workspace = base / "Shuffleboard/standalone-project"
        deploy = workspace / "src/main/deploy"
        deploy.mkdir(parents=True, exist_ok=True)
        for src in (resource_root() / "bundle_payload/vision_helpers").glob("*.py"):
            target = deploy / src.name
            if not target.exists() or sha256(target) != sha256(src):
                shutil.copy2(src, target)
        # The source file itself remains bundled; only writable project paths move.
        old_root = planner.PROJECT_DIR
        for name, value in list(vars(planner).items()):
            if isinstance(value, Path) and name != "SELF_PATH" and value.is_relative_to(old_root):
                setattr(planner, name, workspace / value.relative_to(old_root))
        planner.PROJECT_DIR = workspace
        planner.PadX.BINDINGS = planner.MAPS_DIR / "pad_bindings.json"

        def require_project(_request):
            raise ValueError("Выберите папку проекта, чтобы сохранить изменения MapperX.")

        for route in ["/vx/update", "/update/key", "/update/apply"]:
            planner.STUDIO_ROUTES[route] = require_project

    original_status = planner.setup_status

    def status(request):
        data = original_status(request)
        checks = dependency_check()
        pip_names = {m: p for m, p, _why in planner.SETUP_LIBS}
        data["missing"] = [{"module": c["module"], "pip": pip_names.get(c["module"], c["module"]),
                            "why": c.get("error", "")} for c in checks if not c["ok"]]
        data["checks"] = checks
        data["offline"] = True
        data["integrity"] = {"ok": True, "verified_at_startup": True}
        return data

    def export_offline(_request):
        verify_files(prefix="bundle_payload/offline/")
        source = resource_root() / "bundle_payload/offline"
        downloads = Path.home() / "Downloads"
        downloads.mkdir(exist_ok=True)
        size = sum(p.stat().st_size for p in source.iterdir() if p.is_file())
        if shutil.disk_usage(downloads).free < size + 64 * 1024 * 1024:
            raise ValueError("Недостаточно места: нужно %.1f ГБ" % (size / 1e9))
        target = downloads / "Shuffleboard_offline"
        index = 1
        while target.exists():
            target = downloads / ("Shuffleboard_offline-" + str(index))
            index += 1
        shutil.copytree(source, target)
        # Do not claim a successful export until the destination main file matches.
        if sha256(target / "MapperX_offline.py") != sha256(source / "MapperX_offline.py"):
            raise ValueError("Копирование автономного пакета не прошло проверку SHA-256")
        audit = json.loads((resource_root() / "bundle_payload/dependency-audit.json").read_text())
        return {"path": str(target), "size": size, "wheels": audit["wheels"], "pythons": len(audit["portable_python"])}

    planner.STUDIO_ROUTES["/setup/status"] = status
    planner.STUDIO_ROUTES["/setup/pack"] = export_offline
