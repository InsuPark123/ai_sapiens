import importlib.util
from pathlib import Path
import unittest

import numpy as np

p = Path(__file__).resolve().parents[1] / 'tools' / 'prepare_supermimic.py'
spec = importlib.util.spec_from_file_location('prepare_supermimic', p)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


class ExportTest(unittest.TestCase):
    def test_reference_fields_and_frame_layout(self):
        d = {
            'joint_pos': np.ones((2, 23)), 'joint_vel': np.full((2, 23), 2.),
            'body_names': np.array(['pelvis']),
            'body_pos_w': np.array([[[0., 0., .8]], [[0., 0., .9]]]),
            'body_quat_w': np.array([[[1., 0., 0., 0.]], [[1., 0., 0., 0.]]]),
            'body_ang_vel_w': np.array([[[1., 2., 3.]], [[1., 2., 3.]]]),
            'body_lin_vel_w': np.array([[[4., 5., 6.]], [[4., 5., 6.]]])}
        a = mod.reference_features(d, np.full(23, .25))
        self.assertEqual(a.shape, (2, 56))
        expected = np.concatenate([np.full(23, .75), np.full(23, 2.),
                                   [0, 0, -1, 1, 2, 3, 4, 5, 6, .8]])
        np.testing.assert_allclose(a[0], expected)
        self.assertEqual(a[1, -1], .9)


if __name__ == '__main__':
    unittest.main()
