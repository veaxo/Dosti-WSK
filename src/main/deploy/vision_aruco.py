#!/usr/bin/env python3
"""ArUco vision process for the VMX-pi camera.

This process owns /dev/video0 only while MARKER mode is active.  Java starts
and stops it when the Start button changes the vision mode.
"""

import json
import os
import signal
import tempfile
import time
from pathlib import Path

import cv2


CAMERA_INDEX = int(os.getenv("VISION_CAMERA", "0"))
WIDTH = int(os.getenv("VISION_WIDTH", "320"))
HEIGHT = int(os.getenv("VISION_HEIGHT", "240"))
FPS = int(os.getenv("VISION_FPS", "20"))
DICTIONARY_ID = cv2.aruco.DICT_4X4_50

default_dir = Path("/home/lvuser")
if not default_dir.exists():
    default_dir = Path.home()

JSON_PATH = Path(
    os.getenv("VISION_JSON_PATH", str(default_dir / "vision_data.json"))
)

running = True


def stop(_signum, _frame):
    global running
    running = False


def empty_result(frame_number=0, timestamp=0.0):
    return {
        "found": False,
        "targetCount": 0,
        "frame": frame_number,
        "timestamp": timestamp,
        "targets": [],
    }


def write_atomic(data):
    JSON_PATH.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(
        prefix=".vision_data.",
        suffix=".tmp",
        dir=str(JSON_PATH.parent),
    )
    try:
        with os.fdopen(fd, "w") as output:
            json.dump(data, output, separators=(",", ":"))
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, JSON_PATH)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def detect(frame, frame_number, dictionary, parameters):
    result = empty_result(frame_number, time.time())
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _rejected = cv2.aruco.detectMarkers(
        gray,
        dictionary,
        parameters=parameters,
    )

    if ids is None:
        return result

    height, width = frame.shape[:2]
    for marker_corners, marker_id in zip(corners, ids.flatten()):
        points = marker_corners.reshape((4, 2))
        center_x = float(points[:, 0].mean())
        center_y = float(points[:, 1].mean())
        result["targets"].append(
            {
                "id": int(marker_id),
                "x": center_x,
                "y": center_y,
                "xNorm": center_x / width,
                "yNorm": center_y / height,
            }
        )

    result["found"] = bool(result["targets"])
    result["targetCount"] = len(result["targets"])
    return result


signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)

dictionary = cv2.aruco.Dictionary_get(DICTIONARY_ID)
parameters = cv2.aruco.DetectorParameters_create()

camera = None
camera_deadline = time.time() + 10.0
while running and time.time() < camera_deadline:
    candidate = cv2.VideoCapture(CAMERA_INDEX)
    candidate.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    candidate.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    candidate.set(cv2.CAP_PROP_FPS, FPS)

    if candidate.isOpened():
        camera = candidate
        break

    candidate.release()
    print("Camera is busy, waiting for /dev/video%d" % CAMERA_INDEX)
    time.sleep(0.25)

if camera is None:
    raise SystemExit("Cannot open camera index %d" % CAMERA_INDEX)

print("ArUco vision started: %dx%d @ %d FPS" % (WIDTH, HEIGHT, FPS))
print("JSON output: %s" % JSON_PATH)

frame_number = 0
try:
    while running:
        ok, frame = camera.read()
        if not ok:
            write_atomic(empty_result(frame_number, time.time()))
            time.sleep(0.1)
            continue

        write_atomic(detect(frame, frame_number, dictionary, parameters))
        frame_number += 1
finally:
    camera.release()
    write_atomic(empty_result(frame_number, time.time()))
    print("ArUco vision stopped")
