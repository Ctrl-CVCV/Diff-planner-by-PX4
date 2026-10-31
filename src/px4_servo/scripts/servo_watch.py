#!/usr/bin/env python3
"""把 /servo/state 的 JSON 压成一行打印，测试时一眼看清当前动作和 ACK。

用法：
    rosrun px4_servo servo_watch.py              # 或 python3 scripts/servo_watch.py
    rosparam set /servo_watch/state_topic /other/state   # 换话题

输出示例：
    us= 1400 norm=  -0.1 holding=True  armed=True  ok=35 fail=0 result=0
"""

import json

import rospy
from std_msgs.msg import String


def _format(payload):
    acks = payload.get('acks') or {}
    return 'us=%5s norm=%6s holding=%-5s armed=%-5s ok=%d fail=%d result=%s' % (
        payload.get('commanded_us'),
        payload.get('normalized'),
        payload.get('holding'),
        payload.get('armed'),
        acks.get('ok', 0),
        acks.get('fail', 0),
        payload.get('last_result'),
    )


def main():
    rospy.init_node('servo_watch')
    topic = str(rospy.get_param('~state_topic', '/servo/state'))

    def on_state(message):
        try:
            payload = json.loads(message.data)
        except ValueError:
            rospy.logwarn('解析 /servo/state 失败，原始内容：%s', message.data)
            return
        print(_format(payload))

    rospy.Subscriber(topic, String, on_state, queue_size=1)
    rospy.loginfo('监听 %s（Ctrl-C 退出）', topic)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
