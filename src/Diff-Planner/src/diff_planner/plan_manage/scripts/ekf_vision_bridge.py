#!/usr/bin/env python3
"""把 EKF 里程计的位姿转发给 MAVROS，作为 PX4 的外部视觉输入。

链路：

    /Odometry ──> EKF ──> /ekf/ekf_odom ──> 本节点 ──> /mavros/vision_pose/pose
                                                        └─> VISION_POSITION_ESTIMATE ──> PX4 EKF2

MAVROS 的 vision_pose 插件会把 ENU/FLU 转成 NED/FRD 再发给 PX4，所以这里按 ROS
约定原样转发 EKF 的 world 系位姿即可，不要自己转轴。

不修改 LIO/EKF 状态、不调用解锁或切模式服务，也不发布控制指令。但发往 MAVROS
的视觉位姿可能参与飞控估计，是否融合取决于对应 PX4 固件的估计器配置。

参数：
  --odom         里程计话题（默认 /ekf/ekf_odom）
  --topic        输出的 PoseStamped 话题（默认 /mavros/vision_pose/pose）
  --frame        覆盖 header.frame_id；留空则跟随里程计（默认留空，不做 TF 变换）
  --rate         最大发布频率 Hz（默认 50；PX4 建议 30–50）
  --max-age      里程计超过该年龄（秒）就跳过，避免把旧位姿送给飞控（默认 0.5）
  --stamp-offset 额外加到时间戳上的秒数（默认 0；一般应改用 PX4 的 EKF2_EV_DELAY）
"""

import argparse
import math
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def pose_from_odometry(odom, frame=''):
    """把 nav_msgs/Odometry 的位姿复制成 PoseStamped；不改变坐标定义。"""
    pose = PoseStamped()
    pose.header.stamp = odom.header.stamp
    pose.header.frame_id = frame or odom.header.frame_id
    pose.pose = odom.pose.pose
    return pose


def pose_is_finite(odom):
    position = odom.pose.pose.position
    orientation = odom.pose.pose.orientation
    values = (position.x, position.y, position.z,
              orientation.x, orientation.y, orientation.z, orientation.w)
    if not all(math.isfinite(value) for value in values):
        return False
    # 四元数范数明显为 0 说明姿态无效，PX4 会把它当成坏数据。
    norm = math.sqrt(sum(value * value for value in values[3:]))
    return norm > 0.5


def is_fresh(stamp, now, max_age):
    """时间戳为正且与当前时间相差不超过 max_age 秒。"""
    stamp_sec = stamp.to_sec()
    if stamp_sec <= 0.0:
        return False
    return abs(now.to_sec() - stamp_sec) <= max_age


class VisionBridge(object):

    def __init__(self, args):
        self.frame = args.frame
        self.max_age = args.max_age
        self.stamp_offset = args.stamp_offset
        self.period = 1.0 / args.rate if args.rate > 0 else 0.0
        self.last_publish = None
        self.publisher = rospy.Publisher(args.topic, PoseStamped, queue_size=1)
        self.subscriber = rospy.Subscriber(args.odom, Odometry, self.callback,
                                           queue_size=1, tcp_nodelay=True)
        self.published = 0
        self.skipped_stale = 0
        self.skipped_bad = 0
        self.timer = rospy.Timer(rospy.Duration(5.0), self.report)
        rospy.loginfo('EKF 视觉位姿桥：%s -> %s（≤%.0f Hz，里程计超 %.2fs 丢弃，frame=%s）',
                      args.odom, args.topic, args.rate, args.max_age,
                      self.frame or '跟随里程计')

    def callback(self, message):
        now = rospy.Time.now()
        if not is_fresh(message.header.stamp, now, self.max_age):
            self.skipped_stale += 1
            return
        if not pose_is_finite(message):
            self.skipped_bad += 1
            return
        if self.period and self.last_publish is not None \
                and (now - self.last_publish).to_sec() < self.period:
            return
        pose = pose_from_odometry(message, self.frame)
        if self.stamp_offset:
            pose.header.stamp += rospy.Duration(self.stamp_offset)
        self.publisher.publish(pose)
        self.last_publish = now
        self.published += 1

    def report(self, _event):
        if self.published == 0 and self.skipped_stale:
            rospy.logwarn_throttle(10.0, '视觉位姿桥还没发出数据：里程计过期 %d 次，检查 /ekf/ekf_odom',
                                   self.skipped_stale)
            return
        rospy.loginfo('视觉位姿桥：已发 %d 帧，过期丢弃 %d，无效丢弃 %d',
                      self.published, self.skipped_stale, self.skipped_bad)


def parse_args(argv):
    parser = argparse.ArgumentParser(description='把 /ekf/ekf_odom 的位姿转发到 MAVROS 视觉位姿话题')
    parser.add_argument('--odom', default='/ekf/ekf_odom')
    parser.add_argument('--topic', default='/mavros/vision_pose/pose')
    parser.add_argument('--frame', default='')
    parser.add_argument('--rate', type=float, default=50.0)
    parser.add_argument('--max-age', type=float, default=0.5)
    parser.add_argument('--stamp-offset', type=float, default=0.0)
    return parser.parse_args(argv)


def strip_ros_args(argv):
    return [arg for arg in argv if ':=' not in arg]


def main(argv):
    args = parse_args(strip_ros_args(argv))
    rospy.init_node('ekf_vision_bridge', anonymous=False)
    VisionBridge(args)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
