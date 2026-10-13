#!/usr/bin/env python3
"""Calibrate a named fruit color without opening the robot's USB camera.

Example:
    python3 calibrate_hsv.py --name "Red ball" --image fruit.jpg --roi 80 60 50 50

Use --stream-url http://127.0.0.1:1186/?action=stream instead of --image
to sample the existing vision process's MJPEG stream on VMX-pi.
"""

import argparse
import json
import math
import os
import tempfile
from pathlib import Path
from urllib.request import urlopen

import cv2
import numpy as np


DEFAULT_CONFIG = Path(__file__).with_name("color_ranges.json")


def read_stream_frame(url):
    data = bytearray()
    with urlopen(url, timeout=5) as response:
        for _ in range(128):
            chunk = response.read(4096)
            if not chunk:
                break
            data.extend(chunk)
            start = data.find(b"\xff\xd8")
            if start >= 0:
                end = data.find(b"\xff\xd9", start + 2)
                if end >= 0:
                    image = cv2.imdecode(
                        np.frombuffer(data[start:end + 2], dtype=np.uint8),
                        cv2.IMREAD_COLOR,
                    )
                    if image is not None:
                        return image
            elif len(data) > 4096:
                del data[:-4096]
    raise ValueError("could not read a JPEG frame from the vision stream")


def select_samples(image, roi, min_saturation, min_value):
    x, y, width, height = roi
    frame_height, frame_width = image.shape[:2]
    if (width < 4 or height < 4 or x < 0 or y < 0
            or x + width > frame_width or y + height > frame_height):
        raise ValueError("ROI must be a rectangle inside the image")

    # Sample the center of a tightly selected object, avoiding its outline
    # and the background around it.
    margin_x = int(width * 0.12)
    margin_y = int(height * 0.12)
    patch = image[
        y + margin_y:y + height - margin_y,
        x + margin_x:x + width - margin_x,
    ]
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV).reshape(-1, 3)
    colored = hsv[
        (hsv[:, 1] >= min_saturation) & (hsv[:, 2] >= min_value)
    ]
    if len(colored) < 30:
        raise ValueError(
            "too few colored pixels; select a larger object-only ROI or "
            "lower --min-saturation/--min-value"
        )
    return colored


def estimate_ranges(samples, hue_margin, sv_margin, min_saturation, min_value):
    hues = samples[:, 0].astype(np.float64)
    angles = hues * (2.0 * math.pi / 180.0)
    mean_angle = math.atan2(np.sin(angles).mean(), np.cos(angles).mean())
    mean_hue = (mean_angle * 180.0 / (2.0 * math.pi)) % 180.0
    hue_offsets = (hues - mean_hue + 90.0) % 180.0 - 90.0
    hue_low = float(np.percentile(hue_offsets, 5)) - hue_margin
    hue_high = float(np.percentile(hue_offsets, 95)) + hue_margin
    if hue_high - hue_low > 75.0:
        raise ValueError("ROI contains too many different colors; tighten it")

    saturation_low = max(
        min_saturation,
        int(math.floor(np.percentile(samples[:, 1], 5) - sv_margin)),
    )
    saturation_high = min(
        255,
        int(math.ceil(np.percentile(samples[:, 1], 95) + sv_margin)),
    )
    value_low = max(
        min_value,
        int(math.floor(np.percentile(samples[:, 2], 5) - sv_margin)),
    )
    value_high = min(
        255,
        int(math.ceil(np.percentile(samples[:, 2], 95) + sv_margin)),
    )

    start = mean_hue + hue_low
    end = mean_hue + hue_high
    while start < 0.0:
        start += 180.0
        end += 180.0
    while start >= 180.0:
        start -= 180.0
        end -= 180.0

    def hsv_range(low_h, high_h):
        return {
            "lower": [
                max(0, min(179, int(math.floor(low_h)))),
                saturation_low,
                value_low,
            ],
            "upper": [
                max(0, min(179, int(math.ceil(high_h)))),
                saturation_high,
                value_high,
            ],
        }

    if end < 180.0:
        return [hsv_range(start, end)]
    return [hsv_range(0.0, end - 180.0), hsv_range(start, 179.0)]


def save_profile(config_path, name, ranges, dry_run):
    with config_path.open("r", encoding="utf-8") as source:
        config = json.load(source)
    if (not isinstance(config, dict) or config.get("version") != 1
            or not isinstance(config.get("objects"), list)):
        raise ValueError("color_ranges.json has an unsupported format")

    objects = config["objects"]
    for item in objects:
        if not isinstance(item, dict) or not isinstance(item.get("name"), str):
            raise ValueError("color_ranges.json contains an invalid object")
        if item["name"].casefold() == name.casefold():
            # Preserve names already used by the Java sorting code.
            name = item["name"]
            item["ranges"] = ranges
            break
    else:
        objects.append({"name": name, "ranges": ranges})

    if dry_run:
        return name

    fd, temporary = tempfile.mkstemp(
        prefix=".color_ranges.", suffix=".tmp", dir=str(config_path.parent),
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as output:
            json.dump(config, output, indent=2, ensure_ascii=False)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, config_path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return name


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", required=True, help="Name shown in vision results")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image", type=Path, help="Photo of the object")
    source.add_argument("--stream-url", help="Existing vision MJPEG URL")
    region = parser.add_mutually_exclusive_group(required=True)
    region.add_argument(
        "--roi", type=int, nargs=4, metavar=("X", "Y", "W", "H"),
        help="Small rectangle inside the object, in image pixels",
    )
    region.add_argument(
        "--select-roi", action="store_true",
        help="Select the object with the mouse (needs a desktop display)",
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--hue-margin", type=int, default=6)
    parser.add_argument("--sv-margin", type=int, default=20)
    parser.add_argument("--min-saturation", type=int, default=40)
    parser.add_argument("--min-value", type=int, default=40)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    name = args.name.strip()
    if not name or name.casefold() in ("none", "color", "color_none"):
        parser.error("choose a non-empty, non-reserved object name")
    if not (0 <= args.hue_margin <= 30
            and 0 <= args.sv_margin <= 100
            and 0 <= args.min_saturation <= 255
            and 0 <= args.min_value <= 255):
        parser.error("HSV parameters are outside their supported ranges")

    try:
        image = (cv2.imread(str(args.image)) if args.image is not None
                 else read_stream_frame(args.stream_url))
        if image is None:
            raise ValueError("could not open the image")
        if args.select_roi:
            roi = cv2.selectROI("Select object", image, showCrosshair=True)
            cv2.destroyAllWindows()
        else:
            roi = args.roi
        samples = select_samples(
            image, roi, args.min_saturation, args.min_value,
        )
        ranges = estimate_ranges(
            samples, args.hue_margin, args.sv_margin,
            args.min_saturation, args.min_value,
        )
        actual_name = save_profile(args.config, name, ranges, args.dry_run)
    except (OSError, ValueError, json.JSONDecodeError, cv2.error) as error:
        parser.exit(1, "Calibration failed: %s\n" % error)

    print("Object: %s" % actual_name)
    print("HSV ranges: %s" % json.dumps(ranges, ensure_ascii=False))
    print(("Would update" if args.dry_run else "Updated")
          + ": " + str(args.config))


if __name__ == "__main__":
    main()
