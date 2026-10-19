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

# Mask code shared with the HSV calibrators (same folder): the calibration
# preview shows exactly the mask computed here.
from color_mask import (
    build_color_mask, clean_mask, prepare_hsv, profile_base_name,
)


CAMERA_INDEX = int(os.getenv("VISION_CAMERA", "0"))
WIDTH = int(os.getenv("VISION_WIDTH", "320"))
HEIGHT = int(os.getenv("VISION_HEIGHT", "240"))
FPS = int(os.getenv("VISION_FPS", "20"))
DICTIONARY_ID = cv2.aruco.DICT_4X4_50
MIN_COLOR_AREA = 400.0
COLOR_CONTEXT_PADDING_PX = 8
# Lowered from 0.55: a white podium tinted yellow by the room light filled
# the padding around a cube and made real cubes fail this check.
MIN_COLOR_PURITY = 0.40
# Long thin blobs (tape lines, field edges) are not balls or cubes. Objects cut
# by the frame border are exempt because only a slice of them is visible.
MAX_COLOR_ASPECT = 3.0
# A contour this close to the frame border is reported as touching it, so Java
# knows the visible size and bottom edge are truncated.
EDGE_MARGIN_PX = 3
# Frame-to-frame tracking: keep following the same object instead of jumping
# to whichever candidate is closest to the image center in each frame.
TRACK_MAX_AGE_SEC = 0.5
TRACK_MIN_GATE_PX = 40.0
TRACK_GATE_SIZE_FACTOR = 0.75
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
COLOR_CONFIG_PATH = Path(
    os.getenv(
        "VISION_COLOR_CONFIG_PATH",
        str(Path(__file__).with_name("color_ranges.json")),
    )
)

# Stream: a viewer that disappears without closing the connection used to
# keep its thread (and memory) forever. Writes now time out, and only a few
# viewers are served at once.
STREAM_SOCKET_TIMEOUT_SEC = 5.0
MAX_STREAM_CLIENTS = 3
# Camera gives no frames for this long: exit so Java restarts the process
# and the camera is reopened (USB hiccup), instead of looping forever.
CAMERA_DEAD_EXIT_SEC = 3.0
# A USB camera may need a while after opening before the first frame; do
# not exit during this time (exiting at 3 s made a restart loop).
CAMERA_FIRST_FRAME_SEC = 20.0
# The same message is printed at most once per this many seconds, so a stuck
# state cannot fill the log (and the VMX storage).
LOG_REPEAT_SEC = 10.0

running = True
stream_clients = 0
stream_clients_lock = threading.Lock()
last_logged = {}


def log_limited(key, message):
    now = time.monotonic()
    if now - last_logged.get(key, -1e9) >= LOG_REPEAT_SEC:
        last_logged[key] = now
        print(message, flush=True)


stream_condition = threading.Condition()
stream_frame = {"jpeg": None, "number": 0}
color_track = {"name": None, "x": 0.0, "y": 0.0, "size": 0.0, "time": 0.0}


class VisionStreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global stream_clients
        if not self.path.startswith("/"):
            self.send_error(404)
            return
        with stream_clients_lock:
            if stream_clients >= MAX_STREAM_CLIENTS:
                self.send_error(503, "too many stream viewers")
                return
            stream_clients += 1
        try:
            self.connection.settimeout(STREAM_SOCKET_TIMEOUT_SEC)
            self.stream_frames()
        finally:
            with stream_clients_lock:
                stream_clients -= 1

    def stream_frames(self):
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
        label = "%s HSV %.0f%%" % (
            result["name"], 100.0 * result["colorPurity"],
        )
        cv2.putText(
            annotated, label, (6, frame.shape[0] - 8),
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
    """Returns False instead of crashing when the write fails (e.g. the
    storage is full): a crash here made Java restart the process in a loop,
    and every restart wrote another traceback into the log."""
    temporary = None
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(
            prefix=".%s." % path.name,
            suffix=".tmp",
            dir=str(path.parent),
        )
        with os.fdopen(fd, "w") as output:
            json.dump(data, output, separators=(",", ":"))
        # No fsync: the rename is already atomic for the Java reader, and
        # syncing the SD card on every frame added latency to each result.
        os.replace(temporary, path)
        return True
    except OSError as error:
        log_limited("write", "Cannot write %s: %s" % (path, error))
        return False
    finally:
        if temporary is not None and os.path.exists(temporary):
            try:
                os.unlink(temporary)
            except OSError:
                pass


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
        lock_name = data.get("lockName")
        if not isinstance(lock_name, str) or not lock_name.strip():
            lock_name = None
        return {
            "mode": mode,
            "processed": bool(data.get("processed", previous["processed"])),
            "roi": [max(0.0, min(1.0, float(value))) for value in roi],
            "lockName": lock_name.strip() if lock_name else None,
        }
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return previous


def empty_result(mode, frame_number=0, timestamp=None):
    return {
        "mode": mode,
        "found": False,
        "targetCount": 0,
        "frame": frame_number,
        "timestamp": time.time() if timestamp is None else timestamp,
        "frameWidth": WIDTH,
        "frameHeight": HEIGHT,
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


def detect_markers(frame, frame_number, dictionary, parameters, timestamp):
    """Original ArUco detection path, kept independent from color mode."""
    result = empty_result("MARKER", frame_number, timestamp)
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


def load_color_ranges(path):
    """Returns ((object name, [(lower, upper), ...]), ...).

    Profiles that differ only by a trailing number (several calibrations of
    the same object in different light) are merged into one object, so the
    robot does not lose a cube when its color moves from one calibration to
    another (the target is locked by name).
    """
    with path.open("r", encoding="utf-8") as source:
        config = json.load(source)
    if config.get("version") != 1 or not isinstance(config.get("objects"), list):
        raise ValueError("invalid color_ranges.json format")

    merged = {}
    order = []
    for item in config["objects"]:
        name = item.get("name")
        ranges = item.get("ranges")
        if not isinstance(name, str) or not name.strip():
            raise ValueError("color object has no name")
        if not isinstance(ranges, list) or not 1 <= len(ranges) <= 2:
            raise ValueError("expected one or two HSV ranges for %s" % name)

        parsed = []
        for entry in ranges:
            lower = entry.get("lower")
            upper = entry.get("upper")
            if (not isinstance(lower, list) or not isinstance(upper, list)
                    or len(lower) != 3 or len(upper) != 3):
                raise ValueError("invalid HSV range for %s" % name)
            if any(type(value) is not int for value in lower + upper):
                raise ValueError("HSV values must be integers for %s" % name)
            if not (0 <= lower[0] <= upper[0] <= 179
                    and 0 <= lower[1] <= upper[1] <= 255
                    and 0 <= lower[2] <= upper[2] <= 255):
                raise ValueError("HSV bounds are out of range for %s" % name)
            parsed.append((tuple(lower), tuple(upper)))

        base = profile_base_name(name)
        key = base.casefold()
        if key not in merged:
            merged[key] = (base, [])
            order.append(key)
        merged[key][1].extend(parsed)
    if not order:
        raise ValueError("color_ranges.json contains no objects")
    return tuple((merged[key][0], tuple(merged[key][1])) for key in order)


def select_color_target(candidates, lock_name, now):
    """Pick one candidate and keep following it across frames.

    With a lock name only that profile is considered, so another colored object
    closer to the image center can no longer steal the target mid-pickup.
    """
    if lock_name:
        key = lock_name.casefold()
        candidates = [item for item in candidates
                      if item["name"].casefold() == key]
    if not candidates:
        return None, False, 0

    def distance(item):
        return float(np.hypot(item["x"] - color_track["x"],
                              item["y"] - color_track["y"]))

    best = None
    if (color_track["name"] is not None
            and now - color_track["time"] <= TRACK_MAX_AGE_SEC):
        same = [item for item in candidates
                if item["name"] == color_track["name"]]
        if same:
            nearest = min(same, key=distance)
            gate = max(TRACK_MIN_GATE_PX,
                       TRACK_GATE_SIZE_FACTOR * color_track["size"])
            if distance(nearest) <= gate:
                best = nearest
    tracked = best is not None

    if best is None:
        frame_center = WIDTH / 2.0
        best = min(candidates, key=lambda item: abs(item["x"] - frame_center))

    color_track.update(
        name=best["name"], x=best["x"], y=best["y"],
        size=best["size"], time=now,
    )
    return best, tracked, len(candidates)


def detect_colors(frame, frame_number, roi_values, processed, color_ranges,
                  lock_name, timestamp):
    result = empty_result("COLOR", frame_number, timestamp)
    if not processed:
        return result

    roi, offset_x, offset_y = crop_roi(frame, roi_values)
    roi_height, roi_width = roi.shape[:2]
    hsv = prepare_hsv(roi)
    # Count all sufficiently colored pixels, including colors without a named
    # profile. A small red patch on a blue/green object is not a red object.
    colored_mask = cv2.inRange(hsv, (0, 60, 40), (179, 255, 255))

    candidates = []
    # While a color is locked (pickup) the other colors are not needed.
    lock_key = lock_name.casefold() if lock_name else None
    for name, ranges in color_ranges:
        if lock_key is not None and name.casefold() != lock_key:
            continue
        # The profile name comes only from its calibrated HSV mask. Contour
        # geometry is used for position/size and to discard tiny noise.
        mask = clean_mask(build_color_mask(hsv, ranges))
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
            touches_left = x <= EDGE_MARGIN_PX
            touches_right = x + width >= roi_width - EDGE_MARGIN_PX
            touches_top = y <= EDGE_MARGIN_PX
            touches_bottom = y + height >= roi_height - EDGE_MARGIN_PX
            touches_edge = (touches_left or touches_right
                            or touches_top or touches_bottom)

            # minAreaRect is stable for rotated cubes, unlike boundingRect.
            _rotated_center, rotated_size, _angle = cv2.minAreaRect(contour)
            long_side = max(float(rotated_size[0]), float(rotated_size[1]))
            short_side = max(1.0, min(float(rotated_size[0]),
                                      float(rotated_size[1])))
            if not touches_edge and long_side / short_side > MAX_COLOR_ASPECT:
                continue

            left = max(0, x - COLOR_CONTEXT_PADDING_PX)
            top = max(0, y - COLOR_CONTEXT_PADDING_PX)
            right = min(roi_width, x + width + COLOR_CONTEXT_PADDING_PX)
            bottom = min(roi_height, y + height + COLOR_CONTEXT_PADDING_PX)
            matched_pixels = cv2.countNonZero(mask[top:bottom, left:right])
            colored_pixels = cv2.countNonZero(
                colored_mask[top:bottom, left:right]
            )
            color_purity = (matched_pixels / float(colored_pixels)
                            if colored_pixels else 0.0)
            if color_purity < MIN_COLOR_PURITY:
                continue
            moments = cv2.moments(contour)
            if abs(moments["m00"]) > 1e-6:
                center_x = offset_x + moments["m10"] / moments["m00"]
                center_y = offset_y + moments["m01"] / moments["m00"]
            else:
                center_x = offset_x + x + width / 2.0
                center_y = offset_y + y + height / 2.0

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
                    "size": long_side,
                    "bottomY": bottom_y,
                    "area": float(area),
                    "colorPurity": color_purity,
                    "touchesLeft": touches_left,
                    "touchesRight": touches_right,
                    "touchesBottom": touches_bottom,
                    "touchesTop": touches_top,
                }
            )

    best, tracked, count = select_color_target(
        candidates, lock_name, timestamp)
    if best is None:
        return result
    result.update(best)
    result["found"] = True
    result["tracked"] = tracked
    result["targetCount"] = count
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
    # V4L2 queues several frames by default; with one buffer every result
    # describes the newest image instead of one that is 100-200 ms old.
    candidate.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if candidate.isOpened():
        camera = candidate
    else:
        candidate.release()
        log_limited("busy", "Camera is busy, waiting for /dev/video%d" % CAMERA_INDEX)
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
    "lockName": None,
}
frame_number = 0
last_good_frame = time.monotonic()
got_first_frame = False
next_stream_frame = 0.0
color_ranges = ()
color_config_signature = object()

try:
    while running:
        ok, frame = camera.read()
        frame_time = time.time()
        previous_control = control
        control = read_control(control)
        if (control["mode"] != previous_control["mode"]
                or not control["processed"]
                or control["lockName"] != previous_control["lockName"]):
            color_track["name"] = None

        if control["mode"] == "COLOR":
            try:
                config_stat = COLOR_CONFIG_PATH.stat()
                signature = (config_stat.st_mtime_ns, config_stat.st_size)
            except OSError:
                signature = None
            if signature != color_config_signature:
                if signature is None:
                    color_ranges = ()
                    print("HSV profiles unavailable: %s" % COLOR_CONFIG_PATH)
                else:
                    try:
                        color_ranges = load_color_ranges(COLOR_CONFIG_PATH)
                        print("Loaded %d color profiles from %s" % (
                            len(color_ranges), COLOR_CONFIG_PATH,
                        ))
                    except (OSError, ValueError, TypeError, AttributeError) as error:
                        color_ranges = ()
                        print("HSV profiles invalid; color detection disabled: %s" % error)
                color_config_signature = signature

        if not ok:
            write_atomic(JSON_PATH, empty_result(control["mode"], frame_number))
            silent = time.monotonic() - last_good_frame
            if silent > (CAMERA_DEAD_EXIT_SEC if got_first_frame
                         else CAMERA_FIRST_FRAME_SEC):
                print("Camera gives no frames; exiting so it is reopened",
                      flush=True)
                raise SystemExit(3)
            time.sleep(0.1)
            continue
        last_good_frame = time.monotonic()
        if not got_first_frame:
            got_first_frame = True
            print("First camera frame received", flush=True)

        # Java and the pickup thresholds use 320x240 pixel coordinates. Some
        # USB cameras ignore the requested size, so normalize it here.
        if frame.shape[1] != WIDTH or frame.shape[0] != HEIGHT:
            frame = cv2.resize(frame, (WIDTH, HEIGHT),
                               interpolation=cv2.INTER_AREA)

        if control["mode"] == "COLOR":
            result = detect_colors(
                frame,
                frame_number,
                control["roi"],
                control["processed"],
                color_ranges,
                control["lockName"],
                frame_time,
            )
        else:
            result = detect_markers(
                frame,
                frame_number,
                dictionary,
                parameters,
                frame_time,
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
