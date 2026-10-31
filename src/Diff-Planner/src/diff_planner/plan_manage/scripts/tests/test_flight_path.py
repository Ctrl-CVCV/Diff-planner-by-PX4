#!/usr/bin/env python3
"""Unit tests for the pure pose buffer behind flight_path.py."""

import os
import sys
import unittest

from geometry_msgs.msg import Pose
from rospy import Time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import flight_path  # noqa: E402


def pose(x, y=0.0, z=0.0):
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    return p


class PathAccumulatorTest(unittest.TestCase):

    def test_first_pose_is_accepted(self):
        acc = flight_path.PathAccumulator(min_distance=0.02, max_points=100)
        self.assertTrue(acc.add(Time(1.0), 'world', pose(0.0)))
        self.assertEqual(len(acc), 1)
        self.assertEqual(acc.poses()[0].header.frame_id, 'world')

    def test_close_pose_is_skipped(self):
        acc = flight_path.PathAccumulator(min_distance=0.05, max_points=100)
        acc.add(Time(1.0), 'world', pose(0.0))
        self.assertFalse(acc.add(Time(2.0), 'world', pose(0.04)))
        self.assertTrue(acc.add(Time(3.0), 'world', pose(0.06)))
        self.assertEqual(len(acc), 2)

    def test_max_points_drops_oldest(self):
        acc = flight_path.PathAccumulator(min_distance=0.0, max_points=5)
        for i in range(8):
            acc.add(Time(float(i)), 'world', pose(float(i)))
        self.assertEqual(len(acc), 5)
        self.assertAlmostEqual(acc.poses()[0].pose.position.x, 3.0)

    def test_rejects_bad_limits(self):
        with self.assertRaises(ValueError):
            flight_path.PathAccumulator(min_distance=-1.0, max_points=10)
        with self.assertRaises(ValueError):
            flight_path.PathAccumulator(min_distance=0.0, max_points=1)


if __name__ == '__main__':
    unittest.main()
