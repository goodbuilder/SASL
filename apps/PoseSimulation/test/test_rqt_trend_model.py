import unittest

from pose_simulation_rqt.trend_model import append_sample, series_for_key, split_valid_segments


class RqtTrendModelTests(unittest.TestCase):
    def test_invalid_sample_creates_a_gap_between_valid_samples(self):
        samples = []
        append_sample(samples, timestamp_s=1.0, values={"yaw": 0.1}, valid=True, token=1, maxlen=10)
        append_sample(samples, timestamp_s=2.0, values={"yaw": float("nan")}, valid=False, token=2, maxlen=10)
        append_sample(samples, timestamp_s=3.0, values={"yaw": 0.3}, valid=True, token=3, maxlen=10)
        series = series_for_key(samples, "yaw")
        self.assertEqual(series, [(1.0, 0.1), (2.0, None), (3.0, 0.3)])
        self.assertEqual(split_valid_segments(series), [[(1.0, 0.1)], [(3.0, 0.3)]])

    def test_invalid_values_are_not_coerced_to_zero(self):
        samples = []
        append_sample(samples, timestamp_s=1.0, values={"yaw": None}, valid=True, token=1, maxlen=10)
        self.assertEqual(series_for_key(samples, "yaw"), [(1.0, None)])

    def test_duplicate_tokens_do_not_grow_buffer(self):
        samples = []
        self.assertTrue(append_sample(samples, timestamp_s=1.0, values={"yaw": 0.1}, valid=True, token="same", maxlen=10))
        self.assertFalse(append_sample(samples, timestamp_s=1.1, values={"yaw": 0.2}, valid=True, token="same", maxlen=10))
        self.assertEqual(len(samples), 1)


if __name__ == "__main__":
    unittest.main()
