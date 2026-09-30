#!/usr/bin/env python3
"""Persistent MARKER/COLOR vision process for OpenCV 4.4 on VMX-pi.

This is the only process that owns /dev/video0. Java changes recognition mode
through vision_control.json, so the camera is never killed or reopened when
the robot switches between ArUco markers and colored fruit.
"""

import json
import os
import signal
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np


CAMERA_INDEX = int(os.getenv("VISION_CAMERA", "0"))
WIDTH = int(os.getenv("VISION_WIDTH", "320"))
HEIGHT = int(os.getenv("VISION_HEIGHT", "240"))
FPS = int(os.getenv("VISION_FPS", "20"))
DICTIONARY_ID = cv2.aruco.DICT_4X4_50
MIN_COLOR_AREA = 400.0
STREAM_PORT = int(os.getenv("VISION_STREAM_PORT", "1186"))
STREAM_FPS = max(1, int(os.getenv("VISION_STREAM_FPS", "10")))
STREAM_QUALITY = max(10, min(95, int(os.getenv("VISION_STREAM_QUALITY", "60"))))

default_dir = Path("/home/lvuser")
if not default_dir.exists():
    default_dir = Path.home()

JSON_PATH = Path(
    os.getenv("VISION_JSON_PATH", str(default_dir / "vision_data.json"))
)
CONTROL_PATH = Path(
    os.getenv("VISION_CONTROL_PATH", str(default_dir / "vision_control.json"))
)

running = True
stream_condition = threading.Condition()
stream_frame = {"jpeg": None, "number": 0}


class VisionStreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if not self.path.startswith("/"):
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache, no-store")
        self.end_headers()

        last_number = -1
        try:
            while running:
                with stream_condition:
                    stream_condition.wait_for(
                        lambda: not running
                        or stream_frame["number"] != last_number,
                        timeout=1.0,
                    )
                    jpeg = stream_frame["jpeg"]
                    last_number = stream_frame["number"]

                if jpeg is None:
                    continue
                self.wfile.write(
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    + ("Content-Length: %d\r\n\r\n" % len(jpeg)).encode("ascii")
                    + jpeg
                    + b"\r\n"
                )
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def log_message(self, _format, *_args):
        pass


def annotate_frame(frame, result):
    annotated = frame.copy()
    mode = result["mode"]
    cv2.rectangle(annotated, (0, 0), (frame.shape[1], 24), (0, 0, 0), -1)
    cv2.putText(
        annotated, mode, (6, 17), cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (255, 255, 255), 1, cv2.LINE_AA,
    )

    if mode == "MARKER":
        for target in result["targets"]:
            point = (int(target["x"]), int(target["y"]))
            cv2.circle(annotated, point, 7, (255, 255, 0), 2)
            cv2.putText(
                annotated, "ID %d" % target["id"],
                (point[0] + 8, max(35, point[1] - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2,
                cv2.LINE_AA,
            )
    elif result["found"]:
        x = int(result["boxX"])
        y = int(result["boxY"])
        width = int(result["width"])
        height = int(result["height"])
        cv2.rectangle(
            annotated, (x, y), (x + width, y + height), (0, 255, 255), 2,
        )
        cv2.circle(
            annotated, (int(result["x"]), int(result["y"])),
            4, (0, 255, 255), -1,
        )
        cv2.putText(
            annotated, result["name"], (6, frame.shape[0] - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2,
            cv2.LINE_AA,
        )

    if not result["found"]:
        cv2.putText(
            annotated, "NO TARGET", (6, frame.shape[0] - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
            cv2.LINE_AA,
        )
    return annotated


def publish_stream_frame(frame, result):
    annotated = annotate_frame(frame, result)
    ok, encoded = cv2.imencode(
        ".jpg", annotated,
        [int(cv2.IMWRITE_JPEG_QUALITY), STREAM_QUALITY],
    )
    if ok:
        with stream_condition:
            stream_frame["jpeg"] = encoded.tobytes()
            stream_frame["number"] += 1
            stream_condition.notify_all()


def stop(_signum, _frame):
    global running
    running = False


def write_atomic(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(
        prefix=".%s." % path.name,
        suffix=".tmp",
        dir=str(path.parent),
    )
    try:
        with os.fdopen(fd, "w") as output:
            json.dump(data, output, separators=(",", ":"))
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def read_control(previous):
    try:
        with CONTROL_PATH.open("r") as source:
            data = json.load(source)
        mode = str(data.get("mode", previous["mode"])).upper()
        if mode not in ("MARKER", "COLOR"):
            mode = previous["mode"]
        roi = data.get("roi", previous["roi"])
        if not isinstance(roi, list) or len(roi) != 4:
            roi = previous["roi"]
        return {
            "mode": mode,
            "processed": bool(data.get("processed", previous["processed"])),
            "roi": [max(0.0, min(1.0, float(value))) for value in roi],
        }
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return previous


def empty_result(mode, frame_number=0):
    return {
        "mode": mode,
        "found": False,
        "targetCount": 0,
        "frame": frame_number,
        "timestamp": time.time(),
        "targets": [],
    }


def crop_roi(frame, roi_values):
    height, width = frame.shape[:2]
    x_pct, y_pct, width_pct, height_pct = roi_values
    x = max(0, min(width - 1, int(width * x_pct)))
    y = max(0, min(height - 1, int(height * y_pct)))
    roi_width = max(1, min(width - x, int(width * width_pct)))
    roi_height = max(1, min(height - y, int(height * height_pct)))
    return frame[y:y + roi_height, x:x + roi_width], x, y


def detect_markers(frame, frame_number, dictionary, parameters):
    """Original ArUco detection path, kept independent from color mode."""
    result = empty_result("MARKER", frame_number)
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


COLOR_RANGES = (
    (
        "green ball",
        ((35, 60, 40), (85, 255, 255)),
        None,
    ),
    (
        "Red ball",
        ((0, 80, 40), (10, 255, 255)),
        ((160, 80, 40), (180, 255, 255)),
    ),
    (
        "Yellow ball",
        ((18, 80, 70), (35, 255, 255)),
        None,
    ),
)


def detect_colors(frame, frame_number, roi_values, processed):
    result = empty_result("COLOR", frame_number)
    if not processed:
        return result

    roi, offset_x, offset_y = crop_roi(frame, roi_values)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    candidates = []
    for name, primary, secondary in COLOR_RANGES:
        mask = cv2.inRange(
            hsv,
            np.array(primary[0], dtype=np.uint8),
            np.array(primary[1], dtype=np.uint8),
        )
        if secondary is not None:
            second_mask = cv2.inRange(
                hsv,
                np.array(secondary[0], dtype=np.uint8),
                np.array(secondary[1], dtype=np.uint8),
            )
            mask = cv2.bitwise_or(mask, second_mask)

        mask = cv2.erode(mask, kernel)
        mask = cv2.dilate(mask, kernel)
        contours, _hierarchy = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < MIN_COLOR_AREA:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            moments = cv2.moments(contour)
            if abs(moments["m00"]) > 1e-6:
                center_x = offset_x + moments["m10"] / moments["m00"]
                center_y = offset_y + moments["m01"] / moments["m00"]
            else:
                center_x = offset_x + x + width / 2.0
                center_y = offset_y + y + height / 2.0

            # boundingRect grows when a square is rotated.  The longest side
            # of the minimum-area rectangle is much more stable and therefore
            # gives Java a better close-range measurement for angled cubes.
            _rotated_center, rotated_size, _angle = cv2.minAreaRect(contour)
            object_size = max(float(rotated_size[0]), float(rotated_size[1]))
            bottom_y = offset_y + float(contour[:, :, 1].max())
            candidates.append(
                {
                    "name": name,
                    "x": center_x,
                    "y": center_y,
                    "boxX": offset_x + x,
                    "boxY": offset_y + y,
                    "width": float(width),
                    "height": float(height),
                    "size": object_size,
                    "bottomY": bottom_y,
                    "area": float(area),
                }
            )

    if not candidates:
        return result

    frame_center = frame.shape[1] / 2.0
    best = min(candidates, key=lambda item: abs(item["x"] - frame_center))
    result.update(best)
    result["found"] = True
    result["targetCount"] = len(candidates)
    return result


signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)

dictionary = cv2.aruco.Dictionary_get(DICTIONARY_ID)
parameters = cv2.aruco.DetectorParameters_create()

camera = None
while running and camera is None:
    candidate = cv2.VideoCapture(CAMERA_INDEX)
    candidate.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    candidate.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    candidate.set(cv2.CAP_PROP_FPS, FPS)
    if candidate.isOpened():
        camera = candidate
    else:
        candidate.release()
        print("Camera is busy, waiting for /dev/video%d" % CAMERA_INDEX)
        time.sleep(0.5)

if camera is None:
    raise SystemExit("Vision stopped before camera became available")

print("Persistent vision started: %dx%d @ %d FPS" % (WIDTH, HEIGHT, FPS))
print("JSON output: %s" % JSON_PATH)
print("Control input: %s" % CONTROL_PATH)

stream_server = None
try:
    stream_server = ThreadingHTTPServer(
        ("0.0.0.0", STREAM_PORT), VisionStreamHandler,
    )
    stream_server.daemon_threads = True
    threading.Thread(target=stream_server.serve_forever, daemon=True).start()
    print("Annotated MJPEG stream listening on port %d" % STREAM_PORT)
except OSError as error:
    print("MJPEG stream unavailable: %s" % error)

control = {
    "mode": "MARKER",
    "processed": False,
    "roi": [0.0, 0.2, 1.0, 0.6],
}
frame_number = 0
next_stream_frame = 0.0

try:
    while running:
        ok, frame = camera.read()
        control = read_control(control)

        if not ok:
            write_atomic(JSON_PATH, empty_result(control["mode"], frame_number))
            time.sleep(0.1)
            continue

        if control["mode"] == "COLOR":
            result = detect_colors(
                frame,
                frame_number,
                control["roi"],
                control["processed"],
            )
        else:
            result = detect_markers(
                frame,
                frame_number,
                dictionary,
                parameters,
            )

        write_atomic(JSON_PATH, result)
        now = time.monotonic()
        if stream_server is not None and now >= next_stream_frame:
            publish_stream_frame(frame, result)
            next_stream_frame = now + 1.0 / STREAM_FPS
        frame_number += 1
finally:
    if stream_server is not None:
        stream_server.shutdown()
        stream_server.server_close()
    camera.release()
    write_atomic(JSON_PATH, empty_result(control["mode"], frame_number))
    print("Persistent vision stopped")
