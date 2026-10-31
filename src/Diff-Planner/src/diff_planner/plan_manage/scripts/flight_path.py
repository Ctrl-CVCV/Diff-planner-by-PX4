#!/usr/bin/env python3
"""Publish accumulated trajectories as nav_msgs/Path lines for RViz.

Two independent streams:

* flown path -- subscribes to odometry (default ``/ekf/ekf_odom``);
* commanded path -- subscribes to the position command sent to the controller
  (default ``/setpoints_cmd``, i.e. what traj_server/mission asked the drone to
  fly, planner and direct-control legs alike).

Each stream appends a pose whenever it is farther than ``--min-distance`` from
the previous one and republishes the accumulated path at ``--publish-hz``
(latched, so a RViz opened later still shows the whole line). The line starts
when this node starts and keeps growing for the whole flight; older poses are
dropped once ``--max-points`` is reached.

Read-only: it never publishes flight commands and never changes any parameter.
"""

import argparse
import math
import sys

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from quadrotor_msgs.msg import PositionCommand


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='把里程计和位置指令累积成连续轨迹线发布给 RViz（只读，不发控制指令）')
    parser.add_argument('--odom', default='/ekf/ekf_odom', help='里程计话题（实测轨迹）')
    parser.add_argument('--topic', default='/ekf/flight_path', help='实测轨迹 nav_msgs/Path 话题')
    parser.add_argument('--frame', default='',
                        help='实测轨迹坐标系；留空则跟随里程计 header.frame_id')
    parser.add_argument('--ref', default='/setpoints_cmd',
                        help='位置指令话题（quadrotor_msgs/PositionCommand，参考轨迹）；留空则关闭')
    parser.add_argument('--ref-topic', default='/commanded_path',
                        help='参考轨迹 nav_msgs/Path 话题')
    parser.add_argument('--ref-frame', default='',
                        help='参考轨迹坐标系；留空则跟随指令 header.frame_id，再退回 world')
    parser.add_argument('--min-distance', type=float, default=0.02,
                        help='相邻轨迹点的最小间距 m（防止原地抖动堆点）')
    parser.add_argument('--max-points', type=int, default=20000,
                        help='轨迹点上限，超出后丢弃最旧的点')
    parser.add_argument('--publish-hz', type=float, default=2.0,
                        help='轨迹发布频率 Hz；只在有新点时发布')
    return parser.parse_args(argv)


def strip_ros_args(argv):
    """Drop ROS remapping arguments (``__name:=x``, ``foo:=bar``) before argparse.

    roslaunch appends ``__name:=...`` and ``__log:=...`` to the command line of
    every node.  argparse rejects unknown arguments with ``SystemExit(2)``, which
    killed this node about one second after a ``roslaunch`` start.
    """
    return [arg for arg in argv if ':=' not in arg]


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


def pose_from_position(position):
    """Build a Pose whose orientation is the identity for a bare Point."""
    pose = Pose()
    pose.position = position
    pose.orientation.w = 1.0
    return pose


class PathStream(object):
    """One accumulator plus its latched Path publisher."""

    def __init__(self, accumulator, publisher):
        self.acc = accumulator
        self.publisher = publisher
        self.dirty = False

    def add(self, stamp, frame_id, pose):
        if self.acc.add(stamp, frame_id, pose):
            self.dirty = True

    def publish_if_dirty(self, now):
        if not self.dirty or not len(self.acc):
            return
        self.dirty = False
        path = Path()
        path.header.stamp = now
        path.header.frame_id = self.acc.poses()[0].header.frame_id
        path.poses = list(self.acc.poses())
        self.publisher.publish(path)


class FlightPath(object):

    def __init__(self, args):
        self.frame_override = args.frame
        self.ref_frame_override = args.ref_frame
        self.streams = []

        self.odom_stream = self.make_stream(args.topic, args.min_distance, args.max_points)
        self.odom_sub = rospy.Subscriber(args.odom, Odometry, self.odom_cb, queue_size=20)
        rospy.loginfo('实测轨迹线：订阅 %s，累积后发布到 %s（间距 %.3fm，最多 %d 点）',
                      args.odom, args.topic, args.min_distance, args.max_points)

        self.ref_stream = None
        if args.ref:
            self.ref_stream = self.make_stream(args.ref_topic, args.min_distance, args.max_points)
            self.ref_sub = rospy.Subscriber(args.ref, PositionCommand, self.ref_cb, queue_size=50)
            rospy.loginfo('参考轨迹线：订阅 %s，累积后发布到 %s（从本节点启动开始累积）',
                          args.ref, args.ref_topic)

        period = 1.0 / args.publish_hz if args.publish_hz > 0 else 0.5
        self.timer = rospy.Timer(rospy.Duration(period), self.timer_cb)

    def make_stream(self, topic, min_distance, max_points):
        stream = PathStream(PathAccumulator(min_distance, max_points),
                            rospy.Publisher(topic, Path, queue_size=1, latch=True))
        self.streams.append(stream)
        return stream

    def odom_cb(self, msg):
        frame = self.frame_override or msg.header.frame_id
        if not frame:
            return
        self.odom_stream.add(msg.header.stamp, frame, msg.pose.pose)

    def ref_cb(self, msg):
        frame = self.ref_frame_override or msg.header.frame_id or self.frame_override or 'world'
        stamp = msg.header.stamp
        if stamp == rospy.Time(0):
            stamp = rospy.Time.now()
        self.ref_stream.add(stamp, frame, pose_from_position(msg.position))

    def timer_cb(self, _event):
        now = rospy.Time.now()
        for stream in self.streams:
            stream.publish_if_dirty(now)


def main(argv):
    args = parse_args(strip_ros_args(argv))
    rospy.init_node('flight_path', anonymous=False)
    FlightPath(args)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
