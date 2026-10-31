#!/usr/bin/env python3
"""Publish the flown trajectory as a nav_msgs/Path line for RViz.

Subscribes to odometry (default ``/ekf/ekf_odom``), appends each pose farther
than ``--min-distance`` from the previous one, and republishes the accumulated
path at ``--publish-hz`` (latched, so a RViz opened later still shows the whole
line). Older poses are dropped once ``--max-points`` is reached.

Read-only: it never publishes flight commands and never changes any parameter.
"""

import argparse
import math
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='把里程计累积成连续轨迹线发布给 RViz（只读，不发控制指令）')
    parser.add_argument('--odom', default='/ekf/ekf_odom', help='里程计话题')
    parser.add_argument('--topic', default='/ekf/flight_path', help='轨迹 nav_msgs/Path 话题')
    parser.add_argument('--frame', default='',
                        help='Path 坐标系；留空则跟随里程计 header.frame_id')
    parser.add_argument('--min-distance', type=float, default=0.02,
                        help='相邻轨迹点的最小间距 m（防止原地抖动堆点）')
    parser.add_argument('--max-points', type=int, default=20000,
                        help='轨迹点上限，超出后丢弃最旧的点')
    parser.add_argument('--publish-hz', type=float, default=2.0,
                        help='轨迹发布频率 Hz；只在有新点时发布')
    return parser.parse_args(argv)


class PathAccumulator(object):
    """Pure pose buffer, kept free of ROS types so it can be unit tested."""

    def __init__(self, min_distance, max_points):
        if min_distance < 0.0:
            raise ValueError('min_distance must be >= 0')
        if max_points < 2:
            raise ValueError('max_points must be >= 2')
        self.min_distance = min_distance
        self.max_points = max_points
        self._last_xyz = None
        self._poses = []

    def __len__(self):
        return len(self._poses)

    def add(self, stamp, frame_id, pose):
        """Append ``pose``; return True when it was accepted."""
        xyz = (pose.position.x, pose.position.y, pose.position.z)
        if self._last_xyz is not None and math.dist(xyz, self._last_xyz) < self.min_distance:
            return False
        stamped = PoseStamped()
        stamped.header.stamp = stamp
        stamped.header.frame_id = frame_id
        stamped.pose = pose
        self._poses.append(stamped)
        # Drop the oldest tenth at once instead of paying a pop(0) per point.
        if len(self._poses) > self.max_points:
            del self._poses[:len(self._poses) - self.max_points]
        self._last_xyz = xyz
        return True

    def poses(self):
        return self._poses


class FlightPath(object):

    def __init__(self, args):
        self.acc = PathAccumulator(args.min_distance, args.max_points)
        self.frame_override = args.frame
        self.dirty = False
        self.publisher = rospy.Publisher(args.topic, Path, queue_size=1, latch=True)
        self.subscriber = rospy.Subscriber(args.odom, Odometry, self.odom_cb, queue_size=20)
        period = 1.0 / args.publish_hz if args.publish_hz > 0 else 0.5
        self.timer = rospy.Timer(rospy.Duration(period), self.timer_cb)
        self.odom_topic = args.odom
        self.topic = args.topic
        rospy.loginfo('轨迹线：订阅 %s，累积后发布到 %s（间距 %.3fm，最多 %d 点）',
                      args.odom, args.topic, args.min_distance, args.max_points)

    def odom_cb(self, msg):
        frame = self.frame_override or msg.header.frame_id
        if not frame:
            return
        if self.acc.add(msg.header.stamp, frame, msg.pose.pose):
            self.dirty = True

    def timer_cb(self, _event):
        if not self.dirty or not len(self.acc):
            return
        self.dirty = False
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.acc.poses()[0].header.frame_id
        path.poses = list(self.acc.poses())
        self.publisher.publish(path)


def main(argv):
    args = parse_args(argv)
    rospy.init_node('flight_path', anonymous=False)
    FlightPath(args)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
