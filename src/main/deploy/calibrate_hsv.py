#!/usr/bin/env python3
"""Calibrate a named object color for the robot vision (command line).

The object is marked with a rectangle; everything around it is used as
"not this object", and the HSV range is chosen to cover as much of the
object as possible while matching almost none of the background.

Example:
    python3 calibrate_hsv.py --name red_cube --image cube.jpg --roi 80 60 50 50
    python3 calibrate_hsv.py --name red_cube \\
        --stream-url http://127.0.0.1:1186/?action=stream --select-roi

For several photos in different light, prefer the browser tool
calibrate_hsv_pc.py: it collects samples from many frames and previews the
mask before saving.
"""

import argparse
import json
import os
import tempfile
from pathlib import Path
from urllib.request import urlopen

import cv2
import numpy as np

from color_mask import (
    estimate_profile, object_mask, prepare_hsv, profile_base_name,
    ranges_from_json, DEFAULT_MAX_FALSE_POSITIVE,
)


DEFAULT_CONFIG = Path(__file__).with_name("color_ranges.json")
RESERVED_NAMES = ("none", "color", "color_none")
# Pixels kept per frame (random subsample keeps the estimation fast).
MAX_OBJECT_SAMPLES = 25000
MAX_BACKGROUND_SAMPLES = 60000
# Background samples start this far outside the selection, so the object's
# own edge is not counted as background.
BACKGROUND_GAP_FRACTION = 0.2
BACKGROUND_GAP_MIN_PX = 8
# Explicitly marked background ("this must never match") counts extra.
EXPLICIT_BACKGROUND_WEIGHT = 3


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


def _check_roi(image, roi):
    x, y, width, height = [int(v) for v in roi]
    frame_height, frame_width = image.shape[:2]
    if (width < 4 or height < 4 or x < 0 or y < 0
            or x + width > frame_width or y + height > frame_height):
        raise ValueError("область должна быть прямоугольником внутри кадра")
    return x, y, width, height


def _subsample(pixels, limit, rng):
    if len(pixels) <= limit:
        return pixels
    return pixels[rng.choice(len(pixels), limit, replace=False)]


def sample_object_and_background(image, roi, background_rois=(), seed=0):
    """HSV pixels of the object (inside roi) and of the background (outside
    roi with a gap, plus any explicitly marked background rectangles).

    The same blur and HSV conversion as the robot is used.
    """
    rng = np.random.default_rng(seed)
    hsv = prepare_hsv(image)
    x, y, width, height = _check_roi(image, roi)
    frame_height, frame_width = image.shape[:2]

    inner_x = int(width * 0.05)
    inner_y = int(height * 0.05)
    obj = hsv[y + inner_y:y + height - inner_y,
              x + inner_x:x + width - inner_x].reshape(-1, 3)

    gap = max(BACKGROUND_GAP_MIN_PX, int(BACKGROUND_GAP_FRACTION
                                         * max(width, height)))
    outside = np.ones((frame_height, frame_width), dtype=bool)
    outside[max(0, y - gap):min(frame_height, y + height + gap),
            max(0, x - gap):min(frame_width, x + width + gap)] = False
    background = [hsv[outside]]
    for extra in background_rois:
        bx, by, bw, bh = _check_roi(image, extra)
        marked = hsv[by:by + bh, bx:bx + bw].reshape(-1, 3)
        background.extend([marked] * EXPLICIT_BACKGROUND_WEIGHT)
    background = np.concatenate(background)

    return (_subsample(obj, MAX_OBJECT_SAMPLES, rng).astype(np.uint8),
            _subsample(background, MAX_BACKGROUND_SAMPLES, rng).astype(np.uint8))


def mask_preview(image, ranges):
    """Image with the robot's mask painted green, plus the mask itself."""
    mask = object_mask(image, ranges_from_json(ranges))
    overlay = image.copy()
    painted = mask > 0
    overlay[painted] = (0.35 * overlay[painted]
                        + 0.65 * np.array([60, 255, 60])).astype(np.uint8)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(overlay, contours, -1, (0, 255, 0), 1)
    return overlay, mask


def save_profile(config_path, name, ranges, dry_run=False,
                 replace_family=True):
    """Writes one profile. With replace_family, older calibrations of the
    same object (red_cube1, red_cube_2, ...) are removed, so only the new
    profile is used. Returns (saved name, removed names)."""
    with config_path.open("r", encoding="utf-8") as source:
        config = json.load(source)
    if (not isinstance(config, dict) or config.get("version") != 1
            or not isinstance(config.get("objects"), list)):
        raise ValueError("color_ranges.json has an unsupported format")

    family = profile_base_name(name).casefold()
    kept, removed = [], []
    saved_name = name
    for item in config["objects"]:
        if not isinstance(item, dict) or not isinstance(item.get("name"), str):
            raise ValueError("color_ranges.json contains an invalid object")
        same = item["name"].casefold() == name.casefold()
        related = profile_base_name(item["name"]).casefold() == family
        if same:
            # Keep the exact spelling already used elsewhere.
            saved_name = item["name"]
        if same or (replace_family and related):
            if not same:
                removed.append(item["name"])
            continue
        kept.append(item)
    kept.append({"name": saved_name, "ranges": ranges})
    config["objects"] = kept

    if dry_run:
        return saved_name, removed

    fd, temporary = tempfile.mkstemp(
        prefix=".color_ranges.", suffix=".tmp", dir=str(config_path.parent),
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as output:
            json.dump(config, output, indent=2, ensure_ascii=False)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        # mkstemp creates the file as owner-only; keep the original mode.
        try:
            os.chmod(temporary, os.stat(config_path).st_mode & 0o777)
        except OSError:
            os.chmod(temporary, 0o644)
        os.replace(temporary, config_path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return saved_name, removed


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--name", required=True, help="Object name, e.g. red_cube")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image", type=Path, help="Photo of the object")
    source.add_argument("--stream-url", help="Existing vision MJPEG URL")
    region = parser.add_mutually_exclusive_group(required=True)
    region.add_argument(
        "--roi", type=int, nargs=4, metavar=("X", "Y", "W", "H"),
        help="Rectangle around the whole object, in image pixels",
    )
    region.add_argument(
        "--select-roi", action="store_true",
        help="Select the object with the mouse (needs a desktop display)",
    )
    parser.add_argument(
        "--background-roi", type=int, nargs=4, action="append", default=[],
        metavar=("X", "Y", "W", "H"),
        help="Background that must never match (repeatable)",
    )
    parser.add_argument("--max-false-positive", type=float,
                        default=DEFAULT_MAX_FALSE_POSITIVE,
                        help="Allowed share of background inside the range")
    parser.add_argument("--keep-old", action="store_true",
                        help="Do not remove red_cube1/red_cube2-style profiles")
    parser.add_argument("--preview", type=Path,
                        help="Write the mask preview image to this file")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    name = args.name.strip()
    if not name or name.casefold() in RESERVED_NAMES:
        parser.error("choose a non-empty, non-reserved object name")
    if not 0.0 < args.max_false_positive < 0.2:
        parser.error("--max-false-positive must be between 0 and 0.2")

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
        positives, negatives = sample_object_and_background(
            image, roi, args.background_roi)
        ranges, stats = estimate_profile(
            positives, negatives, args.max_false_positive)
        saved, removed = save_profile(
            args.config, name, ranges, args.dry_run,
            replace_family=not args.keep_old)
        if args.preview:
            overlay, _ = mask_preview(image, ranges)
            cv2.imwrite(str(args.preview), overlay)
    except (OSError, ValueError, json.JSONDecodeError, cv2.error) as error:
        parser.exit(1, "Calibration failed: %s\n" % error)

    print("Object: %s" % saved)
    print("HSV ranges: %s" % json.dumps(ranges, ensure_ascii=False))
    print("Object covered: %.1f %%   background matched: %.2f %%" % (
        100.0 * stats["objectCoverage"],
        100.0 * stats["backgroundFalsePositive"]))
    if removed:
        print("Old profiles replaced: %s" % ", ".join(removed))
    print(("Would update" if args.dry_run else "Updated")
          + ": " + str(args.config))


if __name__ == "__main__":
    main()
