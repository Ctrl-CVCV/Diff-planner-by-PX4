#!/usr/bin/env python3
"""Request PX4 MAVLink IMU-related message intervals once per connection.

The rate is set once, `apply_delay` seconds after MAVROS reports a connection;
PX4 keeps the requested SET_MESSAGE_INTERVAL, so there is nothing to refresh
while the link stays up (2026-10-31: the old 30 s periodic resend was pointless
traffic).  A request is sent again only when MAVROS reconnects (new link, e.g.
PX4 reboot) and, if `refresh_period` > 0, on that explicit period.

The requested rate is not a promise that the serial/USB link and PX4 can
actually deliver that rate.
"""

import threading

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong


MAV_CMD_SET_MESSAGE_INTERVAL = 511
DEFAULT_MESSAGE_IDS = [31, 105]  # ATTITUDE_QUATERNION, HIGHRES_IMU


class Px4ImuRateKeeper:
    def __init__(self):
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.command_service = rospy.get_param("~command_service", "/mavros/cmd/command")
        self.rate_hz = float(rospy.get_param("~rate_hz", 500.0))
        self.message_ids = [int(value) for value in rospy.get_param(
            "~message_ids", DEFAULT_MESSAGE_IDS)]
        self.apply_delay = max(0.0, float(rospy.get_param("~apply_delay", 2.0)))
        self.refresh_period = float(rospy.get_param("~refresh_period", 0.0))
        self.service_timeout = max(0.1, float(rospy.get_param("~service_timeout", 5.0)))

        if self.rate_hz <= 0.0:
            raise ValueError("~rate_hz must be greater than zero")
        if not self.message_ids:
            raise ValueError("~message_ids must not be empty")

        self.interval_us = max(1, int(round(1.0e6 / self.rate_hz)))
        self.connected = False
        self.was_connected = False
        self.applying = False
        self.lock = threading.Lock()
        self.command_client = rospy.ServiceProxy(self.command_service, CommandLong)

        self.state_sub = rospy.Subscriber(
            self.state_topic, State, self.state_cb, queue_size=10)
        self.refresh_timer = None
        if self.refresh_period > 0.0:
            self.refresh_timer = rospy.Timer(
                rospy.Duration(self.refresh_period), self.refresh_cb)

        rospy.loginfo(
            "PX4 IMU rate setter ready: request=%.1f Hz, interval=%d us, "
            "message_ids=%s, apply once per connection (periodic refresh=%.1f s)",
            self.rate_hz, self.interval_us, self.message_ids,
            self.refresh_period)
        rospy.logwarn(
            "This requests a MAVLink rate; the observed ROS topic rate may be lower "
            "because of PX4 scheduling, MAVLink bandwidth, baud rate, USB/serial, "
            "MAVROS, or CPU limits.")

    def state_cb(self, msg):
        self.connected = bool(msg.connected)
        if self.connected and not self.was_connected:
            rospy.loginfo(
                "MAVROS connected; applying message-rate request after %.1f s",
                self.apply_delay)
            rospy.Timer(
                rospy.Duration(self.apply_delay), self.delayed_apply_cb,
                oneshot=True)
        elif not self.connected and self.was_connected:
            rospy.logwarn("MAVROS disconnected; rate request will be reapplied on reconnect")
        self.was_connected = self.connected

    def delayed_apply_cb(self, _event):
        self.apply("MAVROS reconnect")

    def refresh_cb(self, _event):
        self.apply("periodic refresh")

    def apply(self, reason):
        if not self.connected or rospy.is_shutdown():
            return

        with self.lock:
            if self.applying:
                return
            self.applying = True

        try:
            try:
                rospy.wait_for_service(self.command_service, timeout=self.service_timeout)
            except rospy.ROSException as exc:
                rospy.logwarn("Cannot reach %s: %s", self.command_service, exc)
                return

            all_ok = True
            for message_id in self.message_ids:
                if not self.connected or rospy.is_shutdown():
                    all_ok = False
                    break
                try:
                    response = self.command_client(
                        broadcast=False,
                        command=MAV_CMD_SET_MESSAGE_INTERVAL,
                        confirmation=0,
                        param1=float(message_id),
                        param2=float(self.interval_us),
                        param3=0.0,
                        param4=0.0,
                        param5=0.0,
                        param6=0.0,
                        param7=0.0,
                    )
                except rospy.ServiceException as exc:
                    rospy.logwarn(
                        "SET_MESSAGE_INTERVAL failed for MAVLink message %d: %s",
                        message_id, exc)
                    all_ok = False
                    continue

                if response.success:
                    rospy.loginfo(
                        "Requested MAVLink message %d at %.1f Hz (%d us), result=%d",
                        message_id, self.rate_hz, self.interval_us, response.result)
                else:
                    rospy.logwarn(
                        "PX4 rejected MAVLink message %d rate request: result=%d",
                        message_id, response.result)
                    all_ok = False

            if all_ok:
                rospy.loginfo("PX4 IMU-related rates set (%s)", reason)
        finally:
            with self.lock:
                self.applying = False


def main():
    rospy.init_node("px4_imu_rate_keeper")
    try:
        Px4ImuRateKeeper()
    except (TypeError, ValueError) as exc:
        rospy.logfatal("Invalid parameter: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
