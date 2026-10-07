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
COLOR_CONTEXT_PADDING_PX = 8
MIN_COLOR_PURITY = 0.75
MIN_PROFILE_COVERAGE = 0.80
MAX_COLOR_FRAME_FRACTION = 0.60
# A 5 FPS camera produces intervals slightly above 0.20 seconds. Missing
# detections still break confirmation immediately; this bounds a frame pause.
COLOR_CONFIRM_MAX_FRAME_GAP_SEC = 0.35
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
        # COLOR mode displays no object outline or bounding box.
        cv2.circle(
            annotated, (int(result["x"]), int(result["y"])),
            4, (0, 255, 255), -1,
        )
        label = "%s mask %.0f%%" % (
            result["name"], 100.0 * result["colorPurity"],
        )
        cv2.putText(
            annotated, label, (6, frame.shape[0] - 28),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1,
            cv2.LINE_AA,
        )
        hue, saturation, value = result["hsv"]
        cv2.putText(
            annotated, "H:%d S:%d V:%d" % (hue, saturation, value),
            (6, frame.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX,
            0.45, (0, 255, 255), 1, cv2.LINE_AA,
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
    if hasattr(parameters, "detectMarkers"):
        corners, ids, _rejected = parameters.detectMarkers(gray)
    else:
        corners, ids, _rejected = cv2.aruco.detectMarkers(
            gray, dictionary, parameters=parameters,
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


def marker_detector():
    """Use the VMX OpenCV 4.4 API or the newer ArucoDetector API."""
    if hasattr(cv2.aruco, "Dictionary_get"):
        return (cv2.aruco.Dictionary_get(DICTIONARY_ID),
                cv2.aruco.DetectorParameters_create())
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY_ID)
    return dictionary, cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())


def load_color_ranges(path):
    with path.open("r", encoding="utf-8") as source:
        config = json.load(source)
    if config.get("version") != 1 or not isinstance(config.get("objects"), list):
        raise ValueError("invalid color_ranges.json format")

    objects = []
    seen_names = set()
    for item in config["objects"]:
        name = item.get("name")
        ranges = item.get("ranges")
        if not isinstance(name, str) or not name.strip():
            raise ValueError("color object has no name")
        if name.casefold() in seen_names:
            raise ValueError("duplicate color name: %s" % name)
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
        objects.append((name.strip(), parsed[0], parsed[1] if len(parsed) == 2 else None))
        seen_names.add(name.casefold())
    if not objects:
        raise ValueError("color_ranges.json contains no objects")
    return tuple(objects)


def color_groups(profiles):
    """Overlapping saved profiles share identity while each frame keeps a valid name."""
    parents = list(range(len(profiles)))

    def root(index):
        while parents[index] != index:
            index = parents[index]
        return index

    for i, (_, primary, secondary) in enumerate(profiles):
        for j in range(i):
            other_ranges = profiles[j][1:]
            overlap = any(all(max(a[0][axis], b[0][axis]) <= min(a[1][axis], b[1][axis])
                              for axis in range(3))
                          for a in (primary, secondary) if a is not None
                          for b in other_ranges if b is not None)
            if overlap:
                parents[root(i)] = root(j)
    return {name: min(profiles[j][0] for j in range(len(profiles)) if root(j) == root(i))
            for i, (name, _, _) in enumerate(profiles)}


def detect_colors(frame, frame_number, roi_values, processed, color_ranges):
    result = empty_result("COLOR", frame_number)
    result["frameWidth"] = frame.shape[1]
    result["frameHeight"] = frame.shape[0]
    if not processed:
        return result

    roi, offset_x, offset_y = crop_roi(frame, roi_values)
    # Classify original pixels by HSV only. No edge, rectangle or shape
    # detector participates in assigning the profile name.
    raw_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    context_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (2 * COLOR_CONTEXT_PADDING_PX + 1, 2 * COLOR_CONTEXT_PADDING_PX + 1),
    )
    # Count all sufficiently colored pixels, including colors without a named
    # profile. A small red patch on a blue/green object is not a red object.
    colored_mask = cv2.inRange(raw_hsv, (0, 40, 32), (179, 255, 255))
    # Check the whole connected colored region, including pixels outside the
    # selected profile. Computing confidence from matching pixels alone would
    # always report the color of a small reflection as the color of the object.
    _, colored_labels = cv2.connectedComponents(colored_mask, connectivity=8)
    colored_areas = np.bincount(colored_labels.ravel())

    candidates = []
    groups = color_groups(color_ranges)
    for name, primary, secondary in color_ranges:
        raw_mask = cv2.inRange(
            raw_hsv, np.array(primary[0], dtype=np.uint8),
            np.array(primary[1], dtype=np.uint8),
        )
        if secondary is not None:
            raw_mask = cv2.bitwise_or(raw_mask, cv2.inRange(
                raw_hsv, np.array(secondary[0], dtype=np.uint8),
                np.array(secondary[1], dtype=np.uint8),
            ))

        profile_areas = np.bincount(
            colored_labels[raw_mask > 0], minlength=len(colored_areas),
        )

        # Opening suppresses isolated pixel noise. Components group nearby
        # pixels of the same color, without fitting object boundaries.
        mask = cv2.erode(raw_mask, kernel)
        mask = cv2.dilate(mask, kernel)
        component_count, labels = cv2.connectedComponents(mask, connectivity=8)

        for component_id in range(1, component_count):
            component = (labels == component_id) & (raw_mask > 0)
            pixel_y, pixel_x = np.nonzero(component)
            area = len(pixel_x)
            if area < MIN_COLOR_AREA:
                continue
            if area > frame.shape[0] * frame.shape[1] * MAX_COLOR_FRAME_FRACTION:
                continue
            parent_ids = colored_labels[component]
            parent_id = int(np.bincount(parent_ids).argmax())
            if parent_id == 0:
                continue
            profile_coverage = profile_areas[parent_id] / float(colored_areas[parent_id])
            if profile_coverage < MIN_PROFILE_COVERAGE:
                continue
            # Judge neighboring colors through masks, not a rectangular box.
            # This rejects a small red patch surrounded by a blue object.
            context = cv2.dilate(component.astype(np.uint8), context_kernel) > 0
            matched_pixels = np.count_nonzero((raw_mask > 0) & context)
            colored_pixels = np.count_nonzero((colored_mask > 0) & context)
            color_purity = (matched_pixels / float(colored_pixels)
                            if colored_pixels else 0.0)
            if color_purity < MIN_COLOR_PURITY:
                continue
            samples = raw_hsv[component]
            hue_angles = samples[:, 0].astype(np.float64) * (2.0 * np.pi / 180.0)
            mean_hue = (np.arctan2(np.sin(hue_angles).mean(),
                                 np.cos(hue_angles).mean()) * 180.0 / (2.0 * np.pi)) % 180.0
            observed_hsv = [int(round(mean_hue)) % 180,
                            int(np.median(samples[:, 1])),
                            int(np.median(samples[:, 2]))]
            if not any(all(low <= value <= high for value, low, high
                           in zip(observed_hsv, bounds[0], bounds[1]))
                       for bounds in (primary, secondary) if bounds is not None):
                continue
            center_x = float(pixel_x.mean())
            center_y = float(pixel_y.mean())
            # Pickup still needs a proximity signal. Estimate color spread
            # from pixel covariance: no rectangle fit, rotation-invariant.
            # sqrt(12 * variance) keeps the existing pixel scale approximately
            # unchanged for filled patches; it is NOT used to identify color.
            dx = pixel_x.astype(np.float64) - center_x
            dy = pixel_y.astype(np.float64) - center_y
            variance_x = float(np.mean(dx * dx))
            variance_y = float(np.mean(dy * dy))
            covariance = float(np.mean(dx * dy))
            largest_variance = 0.5 * (
                variance_x + variance_y
                + np.sqrt((variance_x - variance_y) ** 2 + 4 * covariance ** 2)
            )
            object_size = float(np.sqrt(12 * largest_variance))
            candidates.append(
                {
                    "name": name,
                    "colorGroup": groups[name],
                    "x": offset_x + center_x,
                    "y": offset_y + center_y,
                    # Preserve Java's telemetry/pickup protocol as pixel spans.
                    "width": float(pixel_x.max() - pixel_x.min() + 1),
                    "height": float(pixel_y.max() - pixel_y.min() + 1),
                    "size": object_size,
                    "bottomY": offset_y + float(pixel_y.max()),
                    "area": float(area),
                    "colorPurity": color_purity,
                    "profileCoverage": profile_coverage,
                    "hsv": observed_hsv,
                    "region": parent_id,
                }
            )

    if not candidates:
        return result

    # Overlapping profiles may describe the same pixels. Publish one physical
    # region, keeping every valid name available for temporal continuity.
    regions = {}
    for candidate in candidates:
        region = candidate.pop("region")
        if region not in regions:
            candidate["matchingNames"] = [candidate["name"]]
            regions[region] = candidate
        else:
            regions[region]["matchingNames"].append(candidate["name"])
    candidates = list(regions.values())
    result["targets"] = candidates
    frame_center = frame.shape[1] / 2.0
    best = min(candidates, key=lambda item: abs(item["x"] - frame_center))
    result.update(best)
    result["found"] = True
    result["targetCount"] = len(candidates)
    return result


class ColorTargetTracker:
    """Confirm a real sequence of frames and keep one target through centering."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.target = None
        self.frames = 0
        self.started = 0.0
        self.last_seen = 0.0
        self.last_frame = None

    def update(self, result, now):
        if result["frame"] == self.last_frame:
            return self.not_found(result, "DUPLICATE_FRAME")
        self.last_frame = result["frame"]
        candidates = result.get("targets", [])
        selected = None
        if self.target is not None:
            compatible = [item for item in candidates
                          if self.target["colorGroup"] == item["colorGroup"]
                          and abs(item["x"] - self.target["x"]) <= 45
                          and abs(item["y"] - self.target["y"]) <= 60]
            if compatible:
                selected = min(compatible, key=lambda item:
                               (item["x"] - self.target["x"]) ** 2
                               + (item["y"] - self.target["y"]) ** 2)
            elif now - self.last_seen <= 0.75:
                self.frames = 0
                self.started = now
                return self.not_found(result, "WAITING_FOR_TRACKED_TARGET")
            else:
                self.reset()
                self.last_frame = result["frame"]
        if selected is None and candidates:
            selected = min(candidates, key=lambda item:
                           abs(item["x"] - result["frameWidth"] / 2.0))
        if selected is None:
            return self.not_found(result)
        selected = dict(selected)
        if self.target is None:
            self.started = now
            self.frames = 0
        else:
            if self.target["name"] in selected["matchingNames"]:
                selected["name"] = self.target["name"]
            # A gap breaks confirmation; never accumulate intermittent noise.
            if now - self.last_seen > COLOR_CONFIRM_MAX_FRAME_GAP_SEC:
                self.started = now
                self.frames = 0
        self.target = selected
        self.last_seen = now
        self.frames += 1
        if self.frames < 4 or now - self.started < 0.15:
            return self.not_found(result, "CONFIRMING_TARGET")
        result.update(selected)
        result["found"] = True
        result["trackingState"] = "TRACKED"
        result["candidateCount"] = len(candidates)
        return result

    @staticmethod
    def not_found(result, state="NO_COLOR_CANDIDATE"):
        clean = empty_result("COLOR", result["frame"])
        clean.update(timestamp=result["timestamp"],
                     frameWidth=result["frameWidth"], frameHeight=result["frameHeight"],
                     trackingState=state, candidateCount=len(result.get("targets", [])))
        return clean


def main():
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    dictionary, parameters = marker_detector()

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
    color_ranges = ()
    color_config_signature = object()
    color_tracker = ColorTargetTracker()
    previous_settings = None
    last_capture_time = None

    try:
        while running:
            ok, frame = camera.read()
            capture_time = time.monotonic()
            control = read_control(control)
            settings = (control["mode"], control["processed"], tuple(control["roi"]))
            if settings != previous_settings:
                color_tracker.reset()
                previous_settings = settings

            if control["mode"] == "COLOR":
                try:
                    config_stat = COLOR_CONFIG_PATH.stat()
                    signature = (config_stat.st_mtime_ns, config_stat.st_size)
                except OSError:
                    signature = None
                if signature != color_config_signature:
                    color_tracker.reset()
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
                last_capture_time = None
                color_tracker.reset()
                failed = empty_result(control["mode"], frame_number)
                failed["cameraOk"] = False
                write_atomic(JSON_PATH, failed)
                time.sleep(0.1)
                continue

            if control["mode"] == "COLOR":
                result = detect_colors(
                    frame,
                    frame_number,
                    control["roi"],
                    control["processed"],
                    color_ranges,
                )
                result = color_tracker.update(result, time.monotonic())
            else:
                result = detect_markers(
                    frame,
                    frame_number,
                    dictionary,
                    parameters,
                )

            result["cameraOk"] = True
            frame_interval = (capture_time - last_capture_time
                              if last_capture_time is not None else 0.0)
            result["processingFps"] = 1.0 / frame_interval if frame_interval > 0 else 0.0
            last_capture_time = capture_time
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
        stopped = empty_result(control["mode"], frame_number)
        stopped["cameraOk"] = False
        write_atomic(JSON_PATH, stopped)
        print("Persistent vision stopped")


if __name__ == "__main__":
    main()
