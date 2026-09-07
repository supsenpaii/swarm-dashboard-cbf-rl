import unittest
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np

import main


class CameraPipelineResolutionTests(unittest.TestCase):
    def test_raw_tracking_frame_preserves_1280x720_source(self):
        rgb = np.zeros((720, 1280, 3), dtype=np.uint8)
        rgb[10, 20] = (11, 22, 33)
        message = SimpleNamespace(
            width=1280,
            height=720,
            step=1280 * 3,
            pixel_format_type=3,
            data=rgb.tobytes(),
        )

        frame = main.GazeboDashboardBridge._image_to_bgr(message)

        self.assertEqual(frame.shape, (720, 1280, 3))
        self.assertEqual(frame[10, 20].tolist(), [33, 22, 11])
        self.assertTrue(frame.flags["C_CONTIGUOUS"])

    def test_browser_jpeg_only_is_resized_to_preview_width(self):
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        with patch.object(main, "CAMERA_MAX_WIDTH", 640):
            jpeg = main.GazeboDashboardBridge._bgr_to_jpeg(frame)
        decoded = cv2.imdecode(
            np.frombuffer(jpeg, dtype=np.uint8),
            cv2.IMREAD_COLOR,
        )

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.shape, (360, 640, 3))


if __name__ == "__main__":
    unittest.main()
