"""Offline color regressions: importing the processor must not open a camera."""
import sys
import unittest
from pathlib import Path
from unittest import mock

import cv2
import numpy as np

DEPLOY = Path(__file__).resolve().parents[2] / "main" / "deploy"
sys.path.insert(0, str(DEPLOY))
import vision_aruco as vision


class VisionColorTest(unittest.TestCase):
    def setUp(self):
        # Algorithm fixtures must not depend on the user's live calibration.
        self.profiles = (
            ("Yellow ball", ((17, 65, 55), (37, 255, 255)), None),
            ("red_cube", ((0, 110, 70), (8, 255, 255)),
             ((172, 110, 70), (179, 255, 255))),
            ("blue_cube", ((94, 40, 32), (122, 255, 255)), None),
        )

    def solid_object(self, hsv):
        image = np.full((240, 320, 3), 180, dtype=np.uint8)
        color = cv2.cvtColor(np.uint8([[hsv]]), cv2.COLOR_HSV2BGR)[0, 0]
        image[110:190, 125:195] = color
        return image

    def detect(self, image):
        return vision.detect_colors(image, 1, [0, 0, 1, 1], True, self.profiles)

    def test_both_red_hue_bands_remain_accepted(self):
        for hue in (2, 176):
            result = self.detect(self.solid_object((hue, 210, 180)))
            self.assertTrue(result["found"])
            self.assertEqual("red_cube", result["name"])

    def test_dim_saturated_red_remains_accepted(self):
        result = self.detect(self.solid_object((2, 160, 90)))
        self.assertEqual("red_cube", result["name"])

    def test_gray_and_weak_red_cast_are_not_red(self):
        for hsv in ((0, 0, 180), (2, 85, 160), (176, 60, 160)):
            self.assertFalse(self.detect(self.solid_object(hsv))["found"])

    def test_orange_brown_and_very_dark_pixels_are_not_red(self):
        for hsv in ((12, 200, 120), (170, 200, 120), (2, 180, 50)):
            self.assertFalse(self.detect(self.solid_object(hsv))["found"])

    def test_same_geometry_different_hues_keep_different_names(self):
        for hsv, name in (((2, 210, 180), "red_cube"),
                          ((110, 210, 180), "blue_cube"),
                          ((25, 210, 180), "Yellow ball")):
            result = self.detect(self.solid_object(hsv))
            self.assertTrue(result["found"])
            self.assertEqual(name, result["name"])

    def test_red_patch_does_not_relabel_a_blue_object(self):
        image = self.solid_object((110, 210, 180))
        red = cv2.cvtColor(np.uint8([[[2, 210, 180]]]), cv2.COLOR_HSV2BGR)[0, 0]
        image[125:165, 145:165] = red
        result = self.detect(image)
        self.assertEqual("blue_cube", result["name"])

    def test_red_ring_is_red_without_a_shape_fill_requirement(self):
        image = self.solid_object((2, 210, 180))
        image[116:184, 131:189] = 180
        self.assertEqual("red_cube", self.detect(image)["name"])

    def test_tiny_noise_is_rejected(self):
        image = np.full((240, 320, 3), 180, dtype=np.uint8)
        red = cv2.cvtColor(np.uint8([[[2, 210, 180]]]), cv2.COLOR_HSV2BGR)[0, 0]
        image[145:155, 155:165] = red
        self.assertFalse(self.detect(image)["found"])

    def test_observed_hsv_is_published_and_hue_wrap_is_handled(self):
        image = self.solid_object((0, 210, 180))
        wrap_red = cv2.cvtColor(np.uint8([[[179, 210, 180]]]), cv2.COLOR_HSV2BGR)[0, 0]
        image[110:190, 160:195] = wrap_red
        result = self.detect(image)
        hue, saturation, value = result["hsv"]
        self.assertTrue(hue < 10 or hue > 170)
        self.assertGreater(saturation, 180)
        self.assertGreater(value, 150)
        annotated = vision.annotate_frame(image, result)
        self.assertEqual(image.shape, annotated.shape)

    def test_disabled_color_processing_finds_nothing(self):
        result = vision.detect_colors(self.solid_object((2, 210, 180)),
                                      1, [0, 0, 1, 1], False, self.profiles)
        self.assertFalse(result["found"])

    def test_user_calibrated_profiles_still_load(self):
        self.assertGreater(len(vision.load_color_ranges(DEPLOY / "color_ranges.json")), 0)

    def test_color_detection_never_calls_edge_or_box_detectors(self):
        with mock.patch.object(cv2, "findContours", side_effect=AssertionError("contour")), \
                mock.patch.object(cv2, "boundingRect", side_effect=AssertionError("box")), \
                mock.patch.object(cv2, "minAreaRect", side_effect=AssertionError("rotated box")):
            result = self.detect(self.solid_object((2, 210, 180)))
        self.assertEqual("red_cube", result["name"])
        self.assertNotIn("boxX", result)
        self.assertNotIn("boxY", result)

    def test_color_annotation_draws_no_object_rectangle(self):
        image = self.solid_object((2, 210, 180))
        result = self.detect(image)
        with mock.patch.object(cv2, "rectangle", wraps=cv2.rectangle) as rectangle:
            vision.annotate_frame(image, result)
        self.assertEqual(1, rectangle.call_count)  # Mode banner only.
        self.assertEqual((0, 0), rectangle.call_args.args[1])

    def test_same_color_with_different_shapes_keeps_same_name(self):
        red = cv2.cvtColor(np.uint8([[[2, 210, 180]]]), cv2.COLOR_HSV2BGR)[0, 0]
        color = tuple(int(value) for value in red)
        circle = np.full((240, 320, 3), 180, dtype=np.uint8)
        cv2.circle(circle, (160, 150), 35, color, -1)
        triangle = np.full((240, 320, 3), 180, dtype=np.uint8)
        cv2.fillPoly(triangle, [np.int32([[160, 100], [120, 180], [200, 180]])], color)
        for image in (circle, triangle, self.solid_object((2, 210, 180))):
            self.assertEqual("red_cube", self.detect(image)["name"])

    def test_spread_for_pickup_is_stable_when_patch_rotates(self):
        image = self.solid_object((2, 210, 180))
        rotated = cv2.warpAffine(image, cv2.getRotationMatrix2D((160, 150), 30, 1),
                                 (320, 240), flags=cv2.INTER_NEAREST,
                                 borderValue=(180, 180, 180))
        original_size = self.detect(image)["size"]
        rotated_size = self.detect(rotated)["size"]
        self.assertAlmostEqual(80, original_size, delta=2)
        self.assertAlmostEqual(original_size, rotated_size, delta=3)

    def test_separate_color_patches_do_not_merge_into_a_fake_center(self):
        image = np.full((240, 320, 3), 180, dtype=np.uint8)
        red = cv2.cvtColor(np.uint8([[[2, 210, 180]]]), cv2.COLOR_HSV2BGR)[0, 0]
        image[100:140, 40:80] = red
        image[100:140, 140:180] = red
        result = self.detect(image)
        self.assertEqual(2, result["targetCount"])
        self.assertAlmostEqual(159.5, result["x"], delta=1)

    def test_pixel_positions_preserve_roi_offsets_and_pickup_fields(self):
        result = vision.detect_colors(self.solid_object((2, 210, 180)),
                                      1, [0.3125, 0.375, 0.375, 0.5], True, self.profiles)
        self.assertAlmostEqual(159.5, result["x"], delta=1)
        self.assertAlmostEqual(149.5, result["y"], delta=1)
        self.assertEqual(189, result["bottomY"])
        for field in ("size", "width", "height", "area"):
            self.assertTrue(np.isfinite(result[field]))
            self.assertGreater(result[field], 0)

    def test_large_in_range_patch_does_not_name_an_out_of_range_object(self):
        orange = cv2.cvtColor(np.uint8([[[12, 210, 180]]]), cv2.COLOR_HSV2BGR)[0, 0]
        red = cv2.cvtColor(np.uint8([[[2, 210, 180]]]), cv2.COLOR_HSV2BGR)[0, 0]
        image = np.full((240, 320, 3), 180, dtype=np.uint8)
        image[30:220, 50:270] = orange
        image[65:185, 100:220] = red
        self.assertFalse(self.detect(image)["found"])

    def test_uniform_color_background_is_not_a_pickup_object(self):
        color = cv2.cvtColor(np.uint8([[[2, 210, 180]]]), cv2.COLOR_HSV2BGR)[0, 0]
        image = np.full((240, 320, 3), color, dtype=np.uint8)
        self.assertFalse(self.detect(image)["found"])

    def test_actual_config_never_accepts_pixels_outside_all_saved_ranges(self):
        profiles = vision.load_color_ranges(DEPLOY / "color_ranges.json")
        rejected = 0
        for hue in range(0, 180, 7):
            for saturation, value in ((25, 160), (100, 60), (220, 190), (120, 135)):
                image = self.solid_object((hue, saturation, value))
                observed = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)[150, 160]
                matches = any(all(low <= component <= high
                                 for component, low, high in zip(observed, bounds[0], bounds[1]))
                              for _, primary, secondary in profiles
                              for bounds in (primary, secondary) if bounds is not None)
                if not matches:
                    self.assertFalse(vision.detect_colors(
                        image, 1, [0, 0, 1, 1], True, profiles)["found"],
                        "Unexpected profile match for HSV %s" % observed)
                    rejected += 1
        self.assertGreater(rejected, 80)

    def test_overlapping_actual_profiles_count_one_physical_object(self):
        profiles = vision.load_color_ranges(DEPLOY / "color_ranges.json")
        result = vision.detect_colors(self.solid_object((110, 120, 130)),
                                      1, [0, 0, 1, 1], True, profiles)
        self.assertTrue(result["found"])
        self.assertEqual(1, result["targetCount"])
        self.assertEqual(3, len(result["matchingNames"]))

    def test_tall_narrow_patch_has_small_width_despite_large_size(self):
        image = np.full((240, 320, 3), 180, dtype=np.uint8)
        red = cv2.cvtColor(np.uint8([[[2, 210, 180]]]), cv2.COLOR_HSV2BGR)[0, 0]
        image[30:220, 145:175] = red
        result = self.detect(image)
        self.assertLess(result["width"], 40)
        self.assertGreater(result["size"], 180)


class ColorTrackerTest(unittest.TestCase):
    setUp = VisionColorTest.setUp
    solid_object = VisionColorTest.solid_object

    def sequence(self, tracker, frames, image=None, start=1.0):
        image = self.solid_object((2, 210, 180)) if image is None else image
        result = None
        for frame in frames:
            result = tracker.update(vision.detect_colors(image, frame, [0, 0, 1, 1],
                                                         True, self.profiles),
                                    start + frame * 0.06)
        return result

    def test_single_frame_and_duplicate_frame_never_confirm(self):
        tracker = vision.ColorTargetTracker()
        self.assertFalse(self.sequence(tracker, [1])["found"])
        self.assertFalse(self.sequence(tracker, [1] * 20)["found"])
        self.assertTrue(self.sequence(tracker, [2, 3, 4])["found"])

    def test_missing_frame_breaks_confirmation(self):
        tracker = vision.ColorTargetTracker()
        self.sequence(tracker, [1, 2, 3])
        empty = np.full((240, 320, 3), 180, dtype=np.uint8)
        self.assertFalse(self.sequence(tracker, [4], empty)["found"])
        self.assertFalse(self.sequence(tracker, [5, 6, 7])["found"])
        self.assertTrue(self.sequence(tracker, [8])["found"])

    def test_five_fps_with_small_timing_jitter_confirms_and_keeps_target(self):
        tracker = vision.ColorTargetTracker()
        image = self.solid_object((2, 210, 180))
        for number, now in enumerate((1.0, 1.201, 1.409, 1.630, 1.835, 2.051), 1):
            result = tracker.update(vision.detect_colors(image, number, [0, 0, 1, 1],
                                                         True, self.profiles), now)
            self.assertEqual(number >= 4, result["found"])

    def test_long_pause_requires_four_new_frames_again(self):
        tracker = vision.ColorTargetTracker()
        self.assertTrue(self.sequence(tracker, [1, 2, 3, 4])["found"])
        image = self.solid_object((2, 210, 180))
        for number in range(5, 9):
            result = tracker.update(vision.detect_colors(image, number, [0, 0, 1, 1],
                                                         True, self.profiles),
                                    2.0 + (number - 5) * 0.201)
            self.assertEqual(number == 8, result["found"])

    def test_new_central_object_does_not_replace_tracked_target(self):
        tracker = vision.ColorTargetTracker()
        self.assertTrue(self.sequence(tracker, [1, 2, 3, 4])["found"])
        image = self.solid_object((2, 210, 180))
        image[110:190, 125:195] = 180
        red = cv2.cvtColor(np.uint8([[[2, 210, 180]]]), cv2.COLOR_HSV2BGR)[0, 0]
        image[110:190, 115:165] = red
        blue = cv2.cvtColor(np.uint8([[[110, 210, 180]]]), cv2.COLOR_HSV2BGR)[0, 0]
        image[40:80, 140:180] = blue
        result = self.sequence(tracker, [5], image)
        self.assertTrue(result["found"])
        self.assertEqual("red_cube", result["name"])

    def test_overlap_keeps_previously_valid_profile_name(self):
        tracker = vision.ColorTargetTracker()
        profiles = vision.load_color_ranges(DEPLOY / "color_ranges.json")
        for number, hsv in enumerate([(110, 120, 90)] * 4 + [(110, 120, 130)], 1):
            result = tracker.update(vision.detect_colors(self.solid_object(hsv), number,
                                                         [0, 0, 1, 1], True, profiles),
                                    1 + number * 0.06)
        self.assertTrue(result["found"])
        self.assertEqual("blue_cube_1", result["name"])

    def test_settings_reset_requires_new_confirmation(self):
        tracker = vision.ColorTargetTracker()
        self.assertTrue(self.sequence(tracker, [1, 2, 3, 4])["found"])
        tracker.reset()
        self.assertFalse(self.sequence(tracker, [5])["found"])

    def test_lighting_change_keeps_identity_but_name_always_matches_current_hsv(self):
        tracker = vision.ColorTargetTracker()
        profiles = vision.load_color_ranges(DEPLOY / "color_ranges.json")
        for number, hsv in enumerate([(110, 120, 90)] * 4 + [(110, 120, 170)], 1):
            result = tracker.update(vision.detect_colors(self.solid_object(hsv), number,
                                                         [0, 0, 1, 1], True, profiles),
                                    1 + number * 0.06)
        self.assertTrue(result["found"])
        self.assertEqual("blue_cube", result["name"])
        self.assertEqual("blue_cube", result["colorGroup"])
        self.assertNotIn("blue_cube_1", result["matchingNames"])


if __name__ == "__main__":
    unittest.main()
