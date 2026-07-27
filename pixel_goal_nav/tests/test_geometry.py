import unittest

import numpy as np

from pixel_goal_nav.geometry import GoalProjectionError, ProjectionConfig, project_pixel_goal, world_goal_from_local


class PixelGoalGeometryTest(unittest.TestCase):
    def setUp(self):
        self.config = ProjectionConfig(fx=400.0, fy=400.0, cx=320.0, cy=240.0, min_goal_m=0.4, max_goal_m=1.2)
        self.depth = np.full((480, 640), 1.0, dtype=np.float32)

    def test_left_image_goal_maps_to_positive_left_base_goal(self):
        goal = project_pixel_goal((220, 240), self.depth, self.config)
        self.assertGreater(goal["left_m"], 0.0)
        self.assertGreater(goal["forward_m"], 0.0)

    def test_right_image_goal_maps_to_negative_left_base_goal(self):
        goal = project_pixel_goal((420, 240), self.depth, self.config)
        self.assertLess(goal["left_m"], 0.0)

    def test_invalid_depth_is_rejected(self):
        with self.assertRaises(GoalProjectionError):
            project_pixel_goal((320, 240), np.zeros((480, 640), dtype=np.float32), self.config)

    def test_calibrated_transform_is_used_when_supplied(self):
        transform = np.array(
            [[0.0, 0.0, 1.0, 0.15], [-1.0, 0.0, 0.0, 0.0], [0.0, -1.0, 0.0, 0.70], [0.0, 0.0, 0.0, 1.0]]
        )
        config = ProjectionConfig(fx=400.0, fy=400.0, cx=320.0, cy=240.0, base_from_optical=transform)
        goal = project_pixel_goal((320, 240), self.depth, config)
        self.assertEqual(goal["projection_mode"], "calibrated_extrinsic")
        self.assertAlmostEqual(goal["forward_m"], 1.15, places=3)

    def test_world_transform_respects_heading(self):
        world_goal = world_goal_from_local((1.0, 2.0, np.pi / 2.0), 1.0, 0.0)
        np.testing.assert_allclose(world_goal, [1.0, 3.0], atol=1e-6)


if __name__ == "__main__":
    unittest.main()
