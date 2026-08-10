import math
import unittest

from pose_simulation.coordinate_adapter import (
    Wgs84ReferenceOrigin,
    ecef_from_wgs84,
    wgs84_to_gazebo_world_enu,
    wgs84_to_reference_enu,
)


class CoordinateAdapterTests(unittest.TestCase):
    def setUp(self):
        self.origin = Wgs84ReferenceOrigin(37.0, 127.0, 100.0, "ellipsoid")

    def test_reference_origin_maps_to_zero(self):
        result = wgs84_to_reference_enu(37.0, 127.0, 100.0, self.origin)
        self.assertEqual(result, (0.0, 0.0, 0.0))

    def test_longitude_delta_maps_to_east_metres(self):
        result = wgs84_to_reference_enu(37.0, 127.00001, 100.0, self.origin)
        self.assertGreater(result[0], 0.8)
        self.assertLess(result[0], 1.0)
        self.assertAlmostEqual(result[1], 0.0, delta=1.0e-4)

    def test_gazebo_origin_offset_is_explicit(self):
        result = wgs84_to_gazebo_world_enu(
            37.0,
            127.0,
            100.0,
            self.origin,
            (10.0, -2.0, 3.0),
        )
        self.assertEqual(result, (10.0, -2.0, 3.0))

    def test_nonfinite_and_invalid_ranges_fail(self):
        with self.assertRaises(ValueError):
            ecef_from_wgs84(math.nan, 127.0, 100.0)
        with self.assertRaises(ValueError):
            ecef_from_wgs84(91.0, 127.0, 100.0)
        with self.assertRaises(ValueError):
            Wgs84ReferenceOrigin(37.0, 127.0, 100.0, "unknown").validate()


if __name__ == "__main__":
    unittest.main()
