#!/usr/bin/env python3
"""Exact live occupancy probe for a hand-held / stationary drone.

Queries GridMap::queryOccupancy, i.e. the very same getOccupancy() and
getInflateOccupancy() calls the planner uses for collision checking, with the
current odometry position. Prints a colour-coded state in the terminal and
publishes a sphere + text marker for RViz.

Read-only: it subscribes to odometry and calls a service. It never publishes
flight commands.
"""

import argparse
import math
import sys
import threading
import time

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

from plan_env.srv import QueryOccupancy, QueryOccupancyRequest


RESET = '\033[0m'
COLORS = {
    'free': '\033[32m',           # green
    'raw': '\033[38;5;208m',      # orange
    'inflated': '\033[31m',       # red
    'virtual_wall': '\033[33m',   # yellow
    'outside': '\033[90m',        # grey
}
# label shown in the terminal, text shown in RViz, marker colour
STATES = {
    'free': ('自由空间', 'FREE', (0.10, 0.85, 0.10)),
    'raw': ('原始障碍格内', 'RAW OCCUPIED', (1.00, 0.55, 0.00)),
    'inflated': ('膨胀障碍内(会触发避障)', 'INFLATED (OBSTACLE)', (1.00, 0.10, 0.10)),
    'virtual_wall': ('虚拟边界外(地面/天花板)', 'VIRTUAL WALL', (1.00, 0.90, 0.10)),
    'outside': ('超出膨胀地图范围', 'OUTSIDE MAP', (0.60, 0.60, 0.60)),
}


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='实时查询当前位置是否位于障碍物内(只读，不发送控制指令)')
    parser.add_argument('--odom', default='/ekf/ekf_odom', help='里程计话题')
    parser.add_argument('--service', default='/drone_0_diff_planner_node/grid_map/query_occupancy',
                        help='grid_map 占用查询服务')
    parser.add_argument('--marker-topic', default='/map_position_check/state',
                        help='RViz Marker 话题')
    parser.add_argument('--frame', default='world', help='Marker 坐标系')
    parser.add_argument('--rate', type=float, default=10.0, help='查询频率 Hz')
    parser.add_argument('--print-interval', type=float, default=1.0,
                        help='状态不变时的打印间隔 s')
    parser.add_argument('--odom-timeout', type=float, default=1.0,
                        help='里程计超时 s')
    parser.add_argument('--marker-size', type=float, default=0.10, help='球体直径 m')
    parser.add_argument('--no-marker', action='store_true', help='不发布 RViz Marker')
    parser.add_argument('--no-color', action='store_true', help='关闭终端颜色')
    args = parser.parse_args(argv)
    if not math.isfinite(args.rate) or args.rate <= 0:
        parser.error('--rate 必须为正数')
    if not math.isfinite(args.print_interval) or args.print_interval < 0:
        parser.error('--print-interval 必须为非负数')
    return args


class PositionCheck:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.position = None
        self.last_odom = rospy.Time(0)
        self.last_state = None
        self.last_print = 0.0
        self.last_warn = 0.0

        self.service = rospy.ServiceProxy(args.service, QueryOccupancy)
        self.marker_pub = None
        if not args.no_marker:
            self.marker_pub = rospy.Publisher(args.marker_topic, Marker, queue_size=2)
        rospy.Subscriber(args.odom, Odometry, self.odom_callback, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / args.rate), self.check)

    def odom_callback(self, message):
        position = message.pose.pose.position
        if not all(math.isfinite(value) for value in (position.x, position.y, position.z)):
            return
        with self.lock:
            self.position = (position.x, position.y, position.z)
            self.last_odom = rospy.Time.now()

    @staticmethod
    def classify(response):
        if response.occupancy < 0 or response.inflate_occupancy < 0:
            return 'virtual_wall'
        if response.inflate_occupancy > 0:
            return 'inflated'
        if response.occupancy > 0:
            return 'raw'
        if not response.in_inflate_map:
            return 'outside'
        return 'free'

    def call_service(self, position):
        request = QueryOccupancyRequest()
        request.position = Point(*position)
        return self.service.call(request)

    def check(self, _event):
        with self.lock:
            position = self.position
            last_odom = self.last_odom
        if position is None:
            self.warn('还没有收到里程计 %s' % self.args.odom)
            return
        if (rospy.Time.now() - last_odom).to_sec() > self.args.odom_timeout:
            self.warn('%s 超时，位置可能已过期' % self.args.odom)
            return
        try:
            response = self.call_service(position)
        except (rospy.ServiceException, rospy.ROSException) as error:
            self.warn('调用 %s 失败: %s' % (self.args.service, error))
            return
        state = self.classify(response)
        self.report(position, state, response)
        self.publish_marker(position, state)

    def report(self, position, state, response):
        now = time.monotonic()
        changed = state != self.last_state
        self.last_state = state
        if not changed and now - self.last_print < self.args.print_interval:
            return
        self.last_print = now
        color = '' if self.args.no_color else COLORS[state]
        reset = '' if self.args.no_color else RESET
        print('%s[%s]%s pos=(%.3f, %.3f, %.3f) raw=%d inflate=%d in_map=%d in_inflate_map=%d' % (
            color, STATES[state][0], reset,
            position[0], position[1], position[2],
            response.occupancy, response.inflate_occupancy,
            int(response.in_map), int(response.in_inflate_map)), flush=True)

    def publish_marker(self, position, state):
        if self.marker_pub is None:
            return
        stamp = rospy.Time.now()
        sphere = Marker()
        sphere.header.frame_id = self.args.frame
        sphere.header.stamp = stamp
        sphere.ns = 'map_position_check'
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = Point(*position)
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = self.args.marker_size
        sphere.color.r, sphere.color.g, sphere.color.b = STATES[state][2]
        sphere.color.a = 0.95
        sphere.lifetime = rospy.Duration(0.4)
        self.marker_pub.publish(sphere)

        text = Marker()
        text.header.frame_id = self.args.frame
        text.header.stamp = stamp
        text.ns = 'map_position_check'
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position = Point(position[0], position[1], position[2] + 0.25)
        text.pose.orientation.w = 1.0
        text.scale.z = 0.08
        text.color.r = text.color.g = text.color.b = 1.0
        text.color.a = 1.0
        text.text = STATES[state][1]
        text.lifetime = rospy.Duration(0.4)
        self.marker_pub.publish(text)

    def shutdown(self):
        if self.marker_pub is None:
            return
        for marker_id in (0, 1):
            marker = Marker()
            marker.header.frame_id = self.args.frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = 'map_position_check'
            marker.id = marker_id
            marker.action = Marker.DELETE
            self.marker_pub.publish(marker)

    def warn(self, message):
        now = time.monotonic()
        if now - self.last_warn >= 2.0:
            self.last_warn = now
            rospy.logwarn(message)


def main():
    args = parse_args(rospy.myargv(argv=sys.argv)[1:])
    rospy.init_node('map_position_check', anonymous=True)
    node = PositionCheck(args)
    rospy.loginfo('监听 %s，查询 %s（只读，不发送控制指令）', args.odom, args.service)
    try:
        rospy.spin()
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
