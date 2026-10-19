"""Color mask shared by the robot vision process and the HSV calibrators.

Both use exactly this code, so the mask shown while calibrating is the mask
the robot will see.
"""

import re

import cv2
import numpy as np


# Hysteresis mask: the calibrated ranges give confident "seed" pixels; a
# relaxed range of the same hue (lower S/V minimum) adds the shadowed side
# of the object, but only next to a seed. The hue is never widened, so a
# brown floor next to a red cube does not leak in.
WEAK_SAT_FACTOR = 0.75
WEAK_VAL_FACTOR = 0.5
WEAK_REACH_PX = 35

_OPEN_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
_CLOSE_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
# Rectangle, not ellipse: OpenCV dilates with a rectangle row/column-wise,
# ~35x faster. The 71x71 ellipse cost most of a frame and dropped the
# robot's stream to ~1 FPS.
_REACH_KERNEL = cv2.getStructuringElement(
    cv2.MORPH_RECT, (2 * WEAK_REACH_PX + 1, 2 * WEAK_REACH_PX + 1))


def profile_base_name(name):
    """'red_cube', 'red_cube1', 'red_cube_2' -> 'red_cube'."""
    base = re.sub(r"[\s_-]*\d+$", "", name.strip())
    return base or name.strip()


def prepare_hsv(bgr):
    # Blur in BGR before conversion: averaging hue values would corrupt colors
    # close to the red 0/179 wraparound in OpenCV's HSV representation.
    return cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)


def build_color_mask(hsv, ranges):
    """Raw hysteresis mask (0/255) for a list of ((h,s,v), (h,s,v)) ranges.

    Seeds use the calibrated hue and minimum S/V without an upper S/V limit:
    a brightly lit side of an object is still the same color.
    """
    strong = np.zeros(hsv.shape[:2], dtype=np.uint8)
    weak = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in ranges:
        strong |= cv2.inRange(hsv, np.array(lower, dtype=np.uint8),
                              np.array((upper[0], 255, 255), dtype=np.uint8))
        weak_lower = (lower[0], int(lower[1] * WEAK_SAT_FACTOR),
                      int(lower[2] * WEAK_VAL_FACTOR))
        weak |= cv2.inRange(hsv, np.array(weak_lower, dtype=np.uint8),
                            np.array((upper[0], 255, 255), dtype=np.uint8))
    if not strong.any():
        return strong
    weak &= cv2.dilate(strong, _REACH_KERNEL)
    weak |= strong
    count, labels = cv2.connectedComponents(weak, connectivity=8)
    if count <= 1:
        return strong
    keep = np.zeros(count, dtype=np.uint8)
    keep[labels[strong > 0]] = 255
    keep[0] = 0
    return keep[labels]


def clean_mask(mask):
    """Opening removes speckles, closing fills highlights inside an object."""
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _OPEN_KERNEL)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _CLOSE_KERNEL)


def object_mask(bgr, ranges):
    """Final mask exactly as the robot computes it for one object."""
    return clean_mask(build_color_mask(prepare_hsv(bgr), ranges))


# ── Range estimation for the calibrators ──────────────────────────────────

# Saturation/value histogram resolution (bins of 4 levels).
_SV_BIN = 4
_SV_BINS = 256 // _SV_BIN
# Gray, white and black have no reliable hue: never go below these minimums.
MIN_SAT_FLOOR = 50
MIN_VAL_FLOOR = 30
# Largest hue distance from the object's mean hue that is searched.
MAX_HUE_HALF_WIDTH = 30
# Allowed share of background pixels inside the range.
DEFAULT_MAX_FALSE_POSITIVE = 0.003
# After the search the hue range is widened by up to this much on each
# side while the background limit still holds: the camera's hue drifts a
# few units with light, and a range fitted exactly to the samples is fragile.
HUE_SAFETY_MARGIN = 4
# ...and only while the background share grows by less than this.
HUE_MARGIN_FP_GROWTH = 0.0005
# Coverage the robust choice may give up (of the best possible) to move the
# range away from the background.
ROBUST_COVERAGE_LOSS = 0.04
# Finally the S/V minimums are lowered by up to this many levels (same
# background rule), so a slightly paler object in other light still seeds.
SAT_SAFETY_MARGIN = 24
VAL_SAFETY_MARGIN = 32
# Margins are checked as if the camera's hue had drifted this much more, so
# they stop short of a background color that is only a few hue units away.
MARGIN_HUE_DRIFT = 3


def _histogram(hsv_pixels):
    h = hsv_pixels[:, 0].astype(np.int64) % 180
    s = hsv_pixels[:, 1].astype(np.int64) // _SV_BIN
    v = hsv_pixels[:, 2].astype(np.int64) // _SV_BIN
    hist = np.zeros((180, _SV_BINS, _SV_BINS), dtype=np.float64)
    np.add.at(hist, (h, s, v), 1.0)
    return hist


def _counts_at_least(hist):
    """prefix[k, s, v] = pixels of hue < k (over a doubled hue axis, so a
    range may wrap past 179) with S-bin >= s and V-bin >= v."""
    suffix = hist[:, ::-1, ::-1].cumsum(axis=1).cumsum(axis=2)[:, ::-1, ::-1]
    doubled = np.concatenate([suffix, suffix], axis=0)
    prefix = np.zeros((361, _SV_BINS, _SV_BINS), dtype=np.float64)
    prefix[1:] = doubled.cumsum(axis=0)
    return prefix


def dominant_hue(hues):
    """Peak of the circularly smoothed hue histogram. Unlike the mean it is
    not pulled away by background corners inside a rectangular selection."""
    hist = np.bincount(hues.astype(np.int64) % 180, minlength=180)
    hist = hist.astype(np.float64)
    window = np.ones(7)
    smooth = np.convolve(np.concatenate([hist[-3:], hist, hist[:3]]),
                         window, mode="valid")
    return int(np.argmax(smooth)) % 180


def estimate_profile(positives, negatives,
                     max_false_positive=DEFAULT_MAX_FALSE_POSITIVE):
    """Chooses the hue interval and S/V minimums that cover as much of the
    object as possible while matching almost none of the background.

    positives / negatives: N x 3 uint8 HSV arrays (object / not object).
    Returns (ranges for color_ranges.json, statistics dict).
    """
    positives = np.asarray(positives, dtype=np.uint8).reshape(-1, 3)
    negatives = np.asarray(negatives, dtype=np.uint8).reshape(-1, 3)
    colored = positives[(positives[:, 1] >= MIN_SAT_FLOOR)
                        & (positives[:, 2] >= MIN_VAL_FLOOR)]
    if len(colored) < 50:
        raise ValueError("слишком мало цветных пикселей объекта: выделите "
                         "сам объект крупнее или добавьте образцы")
    center = dominant_hue(colored[:, 0])

    pos_prefix = _counts_at_least(_histogram(positives))
    neg_prefix = (_counts_at_least(_histogram(negatives))
                  if len(negatives) else None)
    pos_total = float(len(positives))
    neg_total = float(max(1, len(negatives)))

    s_floor = -(-MIN_SAT_FLOOR // _SV_BIN)
    v_floor = -(-MIN_VAL_FLOOR // _SV_BIN)
    widths = range(0, MAX_HUE_HALF_WIDTH + 1)

    def grids(below, above):
        start = (center - below) % 180
        end = start + below + above + 1            # exclusive, may pass 180
        recall = (pos_prefix[end] - pos_prefix[start])[s_floor:, v_floor:]
        recall = recall / pos_total
        if neg_prefix is not None:
            fpr = (neg_prefix[end] - neg_prefix[start])[s_floor:, v_floor:]
            fpr = fpr / neg_total
        else:
            fpr = np.zeros_like(recall)
        return recall, fpr

    # 1) Best possible object coverage within the background limit.
    best_recall = 0.0
    for below in widths:
        for above in widths:
            recall, fpr = grids(below, above)
            allowed = recall[fpr <= max_false_positive]
            if allowed.size:
                best_recall = max(best_recall, float(allowed.max()))
    if best_recall <= 0.0:
        raise ValueError("не удалось отделить объект от фона: цвет объекта "
                         "совпадает с фоном, добавьте образцы или выделите "
                         "объект точнее")

    # 2) Among ranges that keep nearly that coverage, take the one farthest
    #    from the background: higher S/V minimums and a narrower hue. The
    #    robot's hysteresis mask adds the shadowed edges back.
    needed = best_recall - ROBUST_COVERAGE_LOSS
    s_index = np.arange(_SV_BINS - s_floor)[:, None]
    v_index = np.arange(_SV_BINS - v_floor)[None, :]
    best = None
    for below in widths:
        for above in widths:
            recall, fpr = grids(below, above)
            valid = (fpr <= max_false_positive) & (recall >= needed)
            if not valid.any():
                continue
            robust = (s_index + 0.5 * v_index - 0.3 * (below + above)
                      + 0.0 * recall)
            robust = np.where(valid, robust, -1e9)
            index = np.unravel_index(int(np.argmax(robust)), robust.shape)
            candidate = (robust[index], below, above, index[0] + s_floor,
                         index[1] + v_floor)
            if best is None or candidate[0] > best[0]:
                best = candidate
    _, below, above, s_bin, v_bin = best

    def false_positive(lo, hi):
        if neg_prefix is None:
            return 0.0
        start = (center - lo) % 180
        end = start + lo + hi + 1
        return float((neg_prefix[end] - neg_prefix[start])[s_bin, v_bin]
                     / neg_total)

    # 3) Safety margins (hue, then S/V), only while the background does not
    #    grow, checked with an extra hue drift of MARGIN_HUE_DRIFT.
    def drifted(lo, hi):
        return false_positive(lo + MARGIN_HUE_DRIFT, hi + MARGIN_HUE_DRIFT)

    limit = min(max_false_positive,
                drifted(below, above) + HUE_MARGIN_FP_GROWTH)
    for _ in range(HUE_SAFETY_MARGIN):
        if below + above + 2 + 2 * MARGIN_HUE_DRIFT >= 90:
            break
        if drifted(below + 1, above) <= limit:
            below += 1
        if drifted(below, above + 1) <= limit:
            above += 1
    for _ in range(SAT_SAFETY_MARGIN // _SV_BIN):
        if s_bin <= s_floor:
            break
        s_bin -= 1
        if drifted(below, above) > limit:
            s_bin += 1
            break
    for _ in range(VAL_SAFETY_MARGIN // _SV_BIN):
        if v_bin <= v_floor:
            break
        v_bin -= 1
        if drifted(below, above) > limit:
            v_bin += 1
            break
    fpr = false_positive(below, above)
    start = (center - below) % 180
    end_excl = start + below + above + 1
    recall = float((pos_prefix[end_excl] - pos_prefix[start])[s_bin, v_bin]
                   / pos_total)

    s_min = int(s_bin * _SV_BIN)
    v_min = int(v_bin * _SV_BIN)
    start = (center - below) % 180
    end = start + below + above
    if end <= 179:
        ranges = [{"lower": [start, s_min, v_min], "upper": [end, 255, 255]}]
    else:
        ranges = [
            {"lower": [0, s_min, v_min], "upper": [end - 180, 255, 255]},
            {"lower": [start, s_min, v_min], "upper": [179, 255, 255]},
        ]
    stats = {
        "objectCoverage": float(recall),
        "backgroundFalsePositive": float(fpr),
        "hueCenter": center,
        "objectPixels": int(pos_total),
        "backgroundPixels": int(len(negatives)),
    }
    return ranges, stats


def ranges_from_json(ranges):
    return [(tuple(r["lower"]), tuple(r["upper"])) for r in ranges]
