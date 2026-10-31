#!/usr/bin/env python3
"""Unit tests for the pure pose buffer behind flight_path.py."""

import os
import sys
import unittest

from geometry_msgs.msg import Point, Pose
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


class StripRosArgsTest(unittest.TestCase):

    def test_drops_remap_arguments(self):
        argv = ['--odom', '/ekf/ekf_odom', '__name:=flight_path',
                '--topic', '/ekf/flight_path', '__log:=/tmp/flight_path-1.log']
        self.assertEqual(flight_path.strip_ros_args(argv),
                         ['--odom', '/ekf/ekf_odom', '--topic', '/ekf/flight_path'])

    def test_keeps_plain_arguments(self):
        argv = ['--min-distance', '0.05']
        self.assertEqual(flight_path.strip_ros_args(argv), argv)

    def test_parses_launch_style_argv(self):
        argv = ['--odom', '/odom', '--ref', '/cmd', '__name:=flight_path',
                '__log:=/tmp/x.log']
        args = flight_path.parse_args(flight_path.strip_ros_args(argv))
        self.assertEqual(args.odom, '/odom')
        self.assertEqual(args.ref, '/cmd')


class PoseFromPositionTest(unittest.TestCase):

    def test_copies_position_and_uses_identity_orientation(self):
        point = Point()
        point.x, point.y, point.z = 1.0, -2.0, 0.8
        pose = flight_path.pose_from_position(point)
        self.assertAlmostEqual(pose.position.x, 1.0)
        self.assertAlmostEqual(pose.position.y, -2.0)
        self.assertAlmostEqual(pose.position.z, 0.8)
        self.assertAlmostEqual(pose.orientation.w, 1.0)


if __name__ == '__main__':
    unittest.main()
