#!/usr/bin/env python3
"""Unit tests for the pure helpers behind ekf_vision_bridge.py."""

import math
import os
import sys
import unittest

from nav_msgs.msg import Odometry
from rospy import Time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import ekf_vision_bridge  # noqa: E402


def odometry(x=1.0, y=-2.0, z=0.8, frame='world', stamp=100.0, qw=1.0):
    message = Odometry()
    message.header.frame_id = frame
    message.header.stamp = Time.from_sec(stamp)
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.position.z = z
    message.pose.pose.orientation.w = qw
    return message


class PoseFromOdometryTest(unittest.TestCase):

    def test_copies_position_orientation_and_stamp(self):
        pose = ekf_vision_bridge.pose_from_odometry(odometry(), frame='')
        self.assertAlmostEqual(pose.pose.position.x, 1.0)
        self.assertAlmostEqual(pose.pose.position.y, -2.0)
        self.assertAlmostEqual(pose.pose.position.z, 0.8)
        self.assertAlmostEqual(pose.pose.orientation.w, 1.0)
        self.assertEqual(pose.header.frame_id, 'world')
        self.assertAlmostEqual(pose.header.stamp.to_sec(), 100.0)

    def test_frame_override(self):
        pose = ekf_vision_bridge.pose_from_odometry(odometry(), frame='vision')
        self.assertEqual(pose.header.frame_id, 'vision')


class PoseValidationTest(unittest.TestCase):

    def test_nan_is_rejected(self):
        message = odometry()
        message.pose.pose.position.x = float('nan')
        self.assertFalse(ekf_vision_bridge.pose_is_finite(message))

    def test_zero_quaternion_is_rejected(self):
        message = odometry(qw=0.0)
        self.assertFalse(ekf_vision_bridge.pose_is_finite(message))

    def test_normal_pose_is_accepted(self):
        self.assertTrue(ekf_vision_bridge.pose_is_finite(odometry()))


class FreshnessTest(unittest.TestCase):

    def test_recent_stamp_is_fresh(self):
        self.assertTrue(ekf_vision_bridge.is_fresh(Time.from_sec(100.0), Time.from_sec(100.2), 0.5))

    def test_old_stamp_is_stale(self):
        self.assertFalse(ekf_vision_bridge.is_fresh(Time.from_sec(99.0), Time.from_sec(100.0), 0.5))

    def test_zero_stamp_is_stale(self):
        self.assertFalse(ekf_vision_bridge.is_fresh(Time.from_sec(0.0), Time.from_sec(100.0), 0.5))

    def test_limits_are_respected(self):
        self.assertFalse(ekf_vision_bridge.is_fresh(Time.from_sec(50.0), Time.from_sec(100.0), 0.5))
        self.assertTrue(math.isfinite(0.0))


if __name__ == '__main__':
    unittest.main()
