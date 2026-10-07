"""Exercise the real process loop with an in-memory camera, without opening hardware."""
import contextlib
import io
import sys
import unittest
from pathlib import Path
from unittest import mock

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "main" / "deploy"))
import vision_aruco as vision


class VisionRuntimeTest(unittest.TestCase):
    def test_current_opencv_detects_marker_id_and_blank_frame(self):
        dictionary, parameters = vision.marker_detector()
        if hasattr(cv2.aruco, "generateImageMarker"):
            marker = cv2.aruco.generateImageMarker(dictionary, 7, 120)
        else:
            marker = cv2.aruco.drawMarker(dictionary, 7, 120)
        image = np.full((240, 320, 3), 255, dtype=np.uint8)
        image[60:180, 100:220] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        result = vision.detect_markers(image, 1, dictionary, parameters)
        self.assertTrue(result["found"])
        self.assertEqual(7, result["targets"][0]["id"])
        self.assertAlmostEqual(160, result["targets"][0]["x"], delta=1)
        blank = np.full((240, 320, 3), 255, dtype=np.uint8)
        self.assertFalse(vision.detect_markers(blank, 2, dictionary, parameters)["found"])

    def test_legacy_marker_api_remains_supported(self):
        legacy = mock.Mock()
        legacy.Dictionary_get.return_value = "dictionary"
        legacy.DetectorParameters_create.return_value = "parameters"
        legacy.detectMarkers.return_value = ([], None, [])
        with mock.patch.object(vision.cv2, "aruco", legacy):
            dictionary, parameters = vision.marker_detector()
            result = vision.detect_markers(np.zeros((240, 320, 3), np.uint8),
                                           1, dictionary, parameters)
        self.assertFalse(result["found"])
        legacy.detectMarkers.assert_called_once()

    def run_one_frame(self, camera_ok):
        camera = mock.Mock()
        camera.isOpened.return_value = True

        def read():
            vision.running = False
            return camera_ok, np.zeros((240, 320, 3), np.uint8) if camera_ok else None

        camera.read.side_effect = read
        previous_running = vision.running
        vision.running = True
        try:
            with mock.patch.object(vision.signal, "signal"), \
                    mock.patch.object(vision.cv2, "VideoCapture", return_value=camera), \
                    mock.patch.object(vision, "ThreadingHTTPServer", side_effect=OSError("offline")), \
                    mock.patch.object(vision, "read_control", side_effect=lambda old: old), \
                    mock.patch.object(vision, "write_atomic") as write, \
                    mock.patch.object(vision.time, "sleep"), \
                    contextlib.redirect_stdout(io.StringIO()):
                vision.main()
            camera.release.assert_called_once()
            return write.call_args_list[0].args[1]
        finally:
            vision.running = previous_running

    def test_main_starts_and_publishes_real_camera_status(self):
        self.assertTrue(self.run_one_frame(True)["cameraOk"])

    def test_failed_camera_read_is_not_reported_as_ready(self):
        result = self.run_one_frame(False)
        self.assertFalse(result["cameraOk"])
        self.assertFalse(result["found"])


if __name__ == "__main__":
    unittest.main()
