import json
import unittest
from pathlib import Path


LAYOUT_PATH = Path(__file__).parents[1] / "config" / "jetson_layout.json"
EXPECTED_ANCHORS = [
    ("A1", 0x00A1, 0.0, 0.0, 2.2),
    ("A2", 0x00A2, 6.3, 0.0, 2.2),
    ("A3", 0x00A3, 0.0, 4.6, 2.2),
    ("A4", 0x00A4, 6.3, 4.6, 2.2),
]


class LayoutConfigTest(unittest.TestCase):
    def setUp(self):
        self.layout = json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))

    def test_confirmed_anchor_layout(self):
        self.assertEqual(self.layout["schema"], 1)
        self.assertEqual(self.layout["coordinate_unit"], "m")
        self.assertEqual(self.layout["origin_anchor"], "A1")
        anchors = self.layout["anchors"]
        self.assertEqual(len(anchors), 4)
        observed = [
            (
                anchor["id"],
                anchor["short_address"],
                anchor["x_m"],
                anchor["y_m"],
                anchor["z_m"],
            )
            for anchor in anchors
        ]
        self.assertEqual(observed, EXPECTED_ANCHORS)

    def test_bias_is_explicitly_uncalibrated(self):
        self.assertEqual(
            self.layout["range_bias_status"],
            "uncalibrated_zero_passthrough",
        )
        for anchor in self.layout["anchors"]:
            self.assertEqual(anchor["range_bias_m"], 0.0)
            self.assertIs(anchor["range_bias_calibrated"], False)


if __name__ == "__main__":
    unittest.main()
