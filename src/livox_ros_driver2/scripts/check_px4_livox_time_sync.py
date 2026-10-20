#!/usr/bin/env python3
"""Measure Livox frame latency and PX4/MAVROS IMU timestamp health.

This tool deliberately does not "correct" timestamps.  It verifies whether both
streams use the ROS/system epoch, measures delivery latency, and reports MAVROS
TIMESYNC stability so that transport delay is not confused with clock offset.
"""

import math
import statistics
import threading
from collections import deque

import rospy
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu

try:
    from mavros_msgs.msg import TimesyncStatus
except ImportError:
    TimesyncStatus = None


def percentile(values, percent):
    if not values:
        return float("nan")
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    pos = (len(ordered) - 1) * percent / 100.0
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - pos) + ordered[hi] * (pos - lo)


def fmt_ms(value):
    return "n/a" if not math.isfinite(value) else f"{value * 1e3:.3f} ms"


class Monitor:
    def __init__(self):
        self.lock = threading.Lock()
        self.lidar_start_age = []
        self.lidar_end_age = []
        self.lidar_span = []
        self.lidar_period = []
        self.imu_age = []
        self.imu_period = []
        self.imu_minus_lidar_end = []
        self.timesync_offset = []
        self.timesync_rtt_ms = []
        self.last_lidar_stamp = None
        self.last_imu_stamp = None
        self.latest_imu_stamp = None
        self.lidar_out_of_order = 0
        self.imu_out_of_order = 0
        self.bad_epoch = 0
        self.start_wall = rospy.Time.now().to_sec()

    def lidar_cb(self, msg):
        if not msg.points:
            return
        now = rospy.Time.now().to_sec()
        start = msg.header.stamp.to_sec()
        # offset_time is uint32 nanoseconds relative to header.stamp/timebase.
        max_offset = max(point.offset_time for point in msg.points) * 1e-9
        end = start + max_offset
        with self.lock:
            self.lidar_start_age.append(now - start)
            self.lidar_end_age.append(now - end)
            self.lidar_span.append(max_offset)
            if abs(now - start) > 3600.0:
                self.bad_epoch += 1
            if self.last_lidar_stamp is not None:
                period = start - self.last_lidar_stamp
                self.lidar_period.append(period)
                if period <= 0.0:
                    self.lidar_out_of_order += 1
            self.last_lidar_stamp = start
            if self.latest_imu_stamp is not None:
                self.imu_minus_lidar_end.append(self.latest_imu_stamp - end)

    def imu_cb(self, msg):
        now = rospy.Time.now().to_sec()
        stamp = msg.header.stamp.to_sec()
        with self.lock:
            self.imu_age.append(now - stamp)
            if abs(now - stamp) > 3600.0:
                self.bad_epoch += 1
            if self.last_imu_stamp is not None:
                period = stamp - self.last_imu_stamp
                self.imu_period.append(period)
                if period <= 0.0:
                    self.imu_out_of_order += 1
            self.last_imu_stamp = stamp
            self.latest_imu_stamp = stamp

    def timesync_cb(self, msg):
        with self.lock:
            self.timesync_offset.append(float(msg.estimated_offset_ns) * 1e-9)
            self.timesync_rtt_ms.append(float(msg.round_trip_time_ms))

    def snapshot(self):
        with self.lock:
            return {
                "lidar_n": len(self.lidar_end_age),
                "imu_n": len(self.imu_age),
                "lidar_end_p50": percentile(self.lidar_end_age, 50),
                "lidar_end_p95": percentile(self.lidar_end_age, 95),
                "imu_age_p50": percentile(self.imu_age, 50),
                "imu_lead_p50": percentile(self.imu_minus_lidar_end, 50),
                "rtt_p50_ms": percentile(self.timesync_rtt_ms, 50),
            }

    def report(self):
        with self.lock:
            lidar_rate = (1.0 / statistics.median(self.lidar_period)
                          if self.lidar_period and statistics.median(self.lidar_period) > 0 else float("nan"))
            imu_rate = (1.0 / statistics.median(self.imu_period)
                        if self.imu_period and statistics.median(self.imu_period) > 0 else float("nan"))
            print("\n========== PX4 / Livox 时间戳检查 ==========")
            print(f"LiDAR 帧数: {len(self.lidar_end_age)}, 估计频率: {lidar_rate:.2f} Hz")
            if self.lidar_span:
                print("LiDAR 扫描跨度: "
                      f"p50={fmt_ms(percentile(self.lidar_span, 50))}, "
                      f"p95={fmt_ms(percentile(self.lidar_span, 95))}")
                print("LiDAR 最后点传输/组帧延迟: "
                      f"p50={fmt_ms(percentile(self.lidar_end_age, 50))}, "
                      f"p95={fmt_ms(percentile(self.lidar_end_age, 95))}, "
                      f"max={fmt_ms(max(self.lidar_end_age))}")
            print(f"IMU 样本数: {len(self.imu_age)}, 估计频率: {imu_rate:.2f} Hz")
            if self.imu_age:
                print("IMU 消息延迟: "
                      f"p50={fmt_ms(percentile(self.imu_age, 50))}, "
                      f"p95={fmt_ms(percentile(self.imu_age, 95))}, "
                      f"max={fmt_ms(max(self.imu_age))}")
            if self.imu_minus_lidar_end:
                print("收到点云时，最新 IMU 时间领先点云末点: "
                      f"p50={fmt_ms(percentile(self.imu_minus_lidar_end, 50))}, "
                      f"p95={fmt_ms(percentile(self.imu_minus_lidar_end, 95))}")
            if self.timesync_offset:
                offset_std = statistics.pstdev(self.timesync_offset) if len(self.timesync_offset) > 1 else 0.0
                print("MAVROS TIMESYNC: "
                      f"offset稳定性(std)={fmt_ms(offset_std)}, "
                      f"RTT p50={percentile(self.timesync_rtt_ms, 50):.3f} ms, "
                      f"RTT p95={percentile(self.timesync_rtt_ms, 95):.3f} ms")
                print("注：estimated_offset 的绝对值通常很大（PX4 启动时钟到 Unix 时钟），应看稳定性而非是否接近 0。")
            else:
                print("MAVROS TIMESYNC: 未收到 /mavros/timesync_status")
            print(f"乱序计数: LiDAR={self.lidar_out_of_order}, IMU={self.imu_out_of_order}")
            if self.bad_epoch:
                print(f"警告：发现 {self.bad_epoch} 个时间戳与 ROS 当前时间相差超过 1 小时，时钟域不一致。")
            if not self.lidar_end_age:
                print("错误：未收到 LiDAR CustomMsg。")
            if not self.imu_age:
                print("错误：未收到 PX4 IMU。")
            print("=============================================\n")


def main():
    rospy.init_node("check_px4_livox_time_sync", anonymous=True)
    lidar_topic = rospy.get_param("~lidar_topic", "/livox/lidar")
    imu_topic = rospy.get_param("~imu_topic", "/mavros/imu/data_raw")
    timesync_topic = rospy.get_param("~timesync_topic", "/mavros/timesync_status")
    duration = float(rospy.get_param("~duration", 30.0))

    monitor = Monitor()
    rospy.Subscriber(lidar_topic, CustomMsg, monitor.lidar_cb, queue_size=20, tcp_nodelay=True)
    rospy.Subscriber(imu_topic, Imu, monitor.imu_cb, queue_size=1000, tcp_nodelay=True)
    if TimesyncStatus is not None:
        rospy.Subscriber(timesync_topic, TimesyncStatus, monitor.timesync_cb, queue_size=100)
    else:
        rospy.logwarn("mavros_msgs is unavailable; TIMESYNC status will not be measured")

    rospy.loginfo("Monitoring %s and %s for %.1f s", lidar_topic, imu_topic, duration)
    deadline = rospy.Time.now().to_sec() + duration if duration > 0 else float("inf")
    rate = rospy.Rate(1.0)
    try:
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            snap = monitor.snapshot()
            rospy.loginfo("LiDAR=%d end_age(p50/p95)=%s/%s; IMU=%d age(p50)=%s; IMU lead=%s; RTT(p50)=%.3f ms",
                          snap["lidar_n"], fmt_ms(snap["lidar_end_p50"]), fmt_ms(snap["lidar_end_p95"]),
                          snap["imu_n"], fmt_ms(snap["imu_age_p50"]), fmt_ms(snap["imu_lead_p50"]),
                          snap["rtt_p50_ms"])
            rate.sleep()
    finally:
        monitor.report()


if __name__ == "__main__":
    main()
