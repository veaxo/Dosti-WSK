"""Checks what vision_aruco.py needs on this VMX-pi and installs it from the
deployed wheels when it is missing (another VMX may have a different image).

Only the standard library is used here: this runs before numpy / OpenCV are
imported. MapperX -> PitX "Система робота" shows env_report() and puts the
wheels for this robot into deploy/robot_wheels.
"""

import importlib
import importlib.util
import json
import os
import platform
import shutil
import subprocess
import sys
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
WHEELS_DIR = HERE / "robot_wheels"
PIP_TIMEOUT_SEC = 900
# Own folder for the installed libraries, not the system Python: works on
# images where pip refuses to touch the system ("externally managed") and
# never changes what the image itself installed.
_home = Path("/home/lvuser")
TARGET_DIR = (_home if _home.exists() else Path.home()) / "mapperx_pylibs"

state = {"bootstrap": "ok"}


def _missing():
    """Names of what is not importable: numpy, cv2, aruco."""
    missing = []
    try:
        importlib.import_module("numpy")
    except Exception:   # a broken install raises more than ImportError
        missing.append("numpy")
    try:
        cv2 = importlib.import_module("cv2")
        if not hasattr(cv2, "aruco"):
            missing.append("aruco")
    except Exception:
        missing.append("cv2")
    return missing


def _wheels():
    try:
        return sorted(f.name for f in WHEELS_DIR.iterdir() if f.name.endswith(".whl"))
    except OSError:
        return []


def _forget(*names):
    for name in list(sys.modules):
        if name.split(".")[0] in names:
            del sys.modules[name]
    importlib.invalidate_caches()


def _stamp_path():
    return TARGET_DIR.parent / ".robot_wheels_tried"


def ensure():
    """Makes numpy and cv2 (with aruco) importable. Returns "ok", "installed"
    or "error: ...". Java restarts this script when it dies, so one set of
    wheels is tried only once (a stamp file), not on every restart."""
    if TARGET_DIR.is_dir() and str(TARGET_DIR) not in sys.path:
        sys.path.insert(0, str(TARGET_DIR))   # installed by an earlier run
    missing = _missing()
    if missing and str(TARGET_DIR) in sys.path:
        # the image's own libraries win over a broken earlier install
        sys.path.remove(str(TARGET_DIR))
        _forget("numpy", "cv2")
        if not _missing():
            state["bootstrap"] = "ok"
            return state["bootstrap"]
        sys.path.insert(0, str(TARGET_DIR))
        _forget("numpy", "cv2")
    if not missing:
        state["bootstrap"] = "ok"
        return state["bootstrap"]
    wheels = _wheels()
    if not wheels:
        state["bootstrap"] = "error: нет %s, и в deploy/robot_wheels нет библиотек" % ", ".join(missing)
        return state["bootstrap"]
    signature = "|".join(wheels) + "|" + ",".join(missing)
    stamp = _stamp_path()
    try:
        tried = stamp.read_text(encoding="utf-8") == signature
    except OSError:
        tried = False
    if tried:
        state["bootstrap"] = "error: нет %s; библиотеки из robot_wheels уже пробовали — не подошли" % ", ".join(missing)
        return state["bootstrap"]
    packages = []
    if "numpy" in missing:
        packages.append("numpy")
    if "cv2" in missing or "aruco" in missing:
        packages.append("opencv-contrib-python-headless")
    command = [sys.executable, "-m", "pip", "install", "--target", str(TARGET_DIR),
               "--upgrade", "--no-index", "--find-links", str(WHEELS_DIR),
               "--disable-pip-version-check"] + packages
    print("robot_bootstrap: installing %s from %s" % (" ".join(packages), WHEELS_DIR), flush=True)
    try:
        done = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              timeout=PIP_TIMEOUT_SEC)
        output = done.stdout.decode("utf-8", "replace")[-600:]
        code = done.returncode
    except (OSError, subprocess.SubprocessError) as error:
        output, code = str(error), -1
    try:
        stamp.write_text(signature, encoding="utf-8")
    except OSError:
        pass
    print(output, flush=True)
    if TARGET_DIR.is_dir() and str(TARGET_DIR) not in sys.path:
        sys.path.insert(0, str(TARGET_DIR))
    _forget("numpy", "cv2")
    left = _missing()
    if not left:
        state["bootstrap"] = "installed"
    else:
        last = output.strip().splitlines()[-1] if output.strip() else "pip code %d" % code
        state["bootstrap"] = "error: не установилось %s (%s)" % (", ".join(left), last[:200])
    return state["bootstrap"]


def _os_name():
    try:
        with open("/etc/os-release", "r", encoding="utf-8") as source:
            for line in source:
                if line.startswith("PRETTY_NAME="):
                    return line.split("=", 1)[1].strip().strip('"')
    except OSError:
        pass
    return platform.platform()


def _module(name):
    try:
        module = importlib.import_module(name)
        return module, str(getattr(module, "__version__", "?")), str(getattr(module, "__file__", "") or "")
    except Exception:
        return None, None, None


def env_report():
    """What MapperX needs to know about this robot (see PitX "Система робота")."""
    cv2, cv2_version, cv2_path = _module("cv2")
    _numpy, numpy_version, numpy_path = _module("numpy")
    try:
        pip_ok = importlib.util.find_spec("pip") is not None
    except Exception:
        pip_ok = False
    try:
        free_mb = int(shutil.disk_usage(str(HERE)).free // (1024 * 1024))
    except OSError:
        free_mb = None
    return {
        "python": platform.python_version(),
        "pyTag": "cp%d%d" % sys.version_info[:2],
        "machine": platform.machine(),
        "os": _os_name(),
        "numpy": numpy_version,
        "cv2": cv2_version,
        "aruco": bool(cv2 is not None and hasattr(cv2, "aruco")),
        "cv2Path": cv2_path,
        "numpyPath": numpy_path,
        "pip": pip_ok,
        "pkill": shutil.which("pkill") is not None,
        "camera": os.path.exists("/dev/video0"),
        "freeMb": free_mb,
        "wheels": _wheels(),
        "bootstrap": state["bootstrap"],
    }


def serve_info_only(port, json_path, mode="MARKER"):
    """Used when OpenCV could not be made to work: the camera cannot run, but
    MapperX still gets /info (what is missing) and Java gets a fresh "nothing
    found" result instead of restarting this script in a loop."""
    import threading

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _format, *_args):
            pass

        def do_GET(self):
            if self.path.split("?")[0] != "/info":
                self.send_error(503, "vision is not running")
                return
            body = json.dumps({"files": {}, "time": time.time(), "env": env_report()}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

    try:
        server = HTTPServer(("0.0.0.0", port), Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
    except OSError as error:
        print("robot_bootstrap: cannot open port %d: %s" % (port, error), flush=True)
    print("robot_bootstrap: vision is NOT running - " + state["bootstrap"], flush=True)
    frame = 0
    while True:
        frame += 1
        result = {"mode": mode, "found": False, "targetCount": 0, "frame": frame,
                  "timestamp": time.time(), "frameWidth": 320, "frameHeight": 240,
                  "targets": [], "error": state["bootstrap"]}
        try:
            temp = str(json_path) + ".boot.tmp"
            with open(temp, "w", encoding="utf-8") as out:
                json.dump(result, out)
            os.replace(temp, str(json_path))
        except OSError:
            pass
        time.sleep(0.5)
