#!/usr/bin/env python3
"""四路物资舵机测试工具（拆桨、可解锁状态下使用）。

对应 payload_servos.launch（PWM_MAIN5..8，服务 /servo1../servo4）：

    1 号物资（消防）  -> /servo3/push          （MAIN7，500→2500）
    2 号物资（救援）  -> /servo4/push          （MAIN8，500→2500）
    下放 3/4 号       -> /servo1/push + /servo2/push（MAIN5/6，500→1800）
    3 号物资（救援）  -> /servo3/push          （MAIN7，500→2500）
    4 号物资（救援）  -> /servo4/push          （MAIN8，500→2500）

安全约束：本工具只调用舵机服务，不会解锁、不会发飞控指令、不会切模式。
PX4 上锁时 MAIN 输出被强制成 DIS=500，所以真正要看动作需要解锁（务必拆桨、
确认机架固定）。每次动作前都会要求确认。

用法：
    python3 payload_servo_test.py                 # 交互菜单
    python3 payload_servo_test.py --status        # 只显示状态
    python3 payload_servo_test.py --action 1      # 1/2/lower/3/4/all/retract
    python3 payload_servo_test.py --pulse 3 1143  # 直接给 /servo3 发任意脉宽（调试）
"""

import argparse
import json
import sys
import time

import rospy
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger


PAYLOADS = {
    '1': dict(name='1 号物资（消防）', services=['/servo3/push']),
    '2': dict(name='2 号物资（救援）', services=['/servo4/push']),
    'lower': dict(name='下放 3/4 号物资', services=['/servo1/push', '/servo2/push']),
    '3': dict(name='3 号物资（救援）', services=['/servo3/push']),
    '4': dict(name='4 号物资（救援）', services=['/servo4/push']),
}

SERVO_CHANNELS = {
    '/servo1': 'MAIN5（1800：下放）',
    '/servo2': 'MAIN6（1800：下放）',
    '/servo3': 'MAIN7（2500：1/3 号物资）',
    '/servo4': 'MAIN8（2500：2/4 号物资）',
}


class ServoTest(object):

    def __init__(self):
        self.states = {}
        self.armed = None
        self.mode = None
        self.publishers = {}
        for prefix in SERVO_CHANNELS:
            rospy.Subscriber(prefix + '/state', String,
                             lambda msg, prefix=prefix: self._state_cb(prefix, msg),
                             queue_size=1)
            self.publishers[prefix] = rospy.Publisher(prefix + '/set_pulse_us', Float32, queue_size=1)
        from mavros_msgs.msg import State
        rospy.Subscriber('/mavros/state', State, self._mavros_cb, queue_size=1)

    def _state_cb(self, prefix, message):
        try:
            self.states[prefix] = json.loads(message.data)
        except (TypeError, ValueError):
            self.states[prefix] = {'raw': message.data}

    def _mavros_cb(self, message):
        self.armed = bool(message.armed)
        self.mode = str(message.mode)

    def show_status(self):
        print('\n=== 飞控状态 ===')
        if self.armed is None:
            print('  尚未收到 /mavros/state：MAVROS 未启动或未连接飞控')
        else:
            print('  armed=%s  mode=%s' % (self.armed, self.mode))
            if not self.armed:
                print('  提示：未解锁时 PX4 输出 DIS=500，舵机动作可能看不到。'
                      '测试请拆桨后解锁，并确认机架固定。')
        print('\n=== 舵机通道 ===')
        for prefix, mapping in SERVO_CHANNELS.items():
            state = self.states.get(prefix)
            if state is None:
                print('  %-8s %-28s 无状态（节点未启动？）' % (prefix, mapping))
                continue
            limits = state.get('limits_us')
            text = ('  %-8s %-28s ready=%-5s ch=%-2s func=%-4s set=%-2s '
                    '指令=%s us 区间=%s holding=%s push_ok=%s'
                    % (prefix, mapping, state.get('ready'), state.get('channel'),
                       state.get('function'), state.get('actuator_set'),
                       state.get('commanded_us'), limits, state.get('holding'),
                       state.get('push_ok')))
            print(text)
            reason = state.get('reason')
            if reason:
                print('           错误：%s' % reason)
            if state.get('dry_run'):
                print('           注意：该实例 dry_run=true，不会真正驱动飞控输出')
        print()

    def call(self, service, timeout=3.0):
        try:
            from std_srvs.srv import Trigger  # noqa: F401
            rospy.wait_for_service(service, timeout=timeout)
            proxy = rospy.ServiceProxy(service, Trigger)
            response = proxy()
        except Exception as error:
            print('  失败 %-18s %s' % (service, error))
            return False
        print('  %-18s success=%s  %s' % (service, bool(response.success), response.message))
        return bool(response.success)

    def run_payload(self, key, pause_after=3.0):
        item = PAYLOADS[key]
        print('\n执行：%s -> %s' % (item['name'], ' + '.join(item['services'])))
        results = [self.call(service) for service in item['services']]
        print('  结果：%s' % ('全部成功' if all(results) else '有失败，查看上面的消息'))
        if pause_after > 0:
            print('  等待 %.1fs（让舵机保持/自动回位）…' % pause_after)
            time.sleep(pause_after)
            self.show_status()
        return all(results)

    def run_sequence(self):
        for key in ('1', '2', 'lower', '3', '4'):
            if not self.run_payload(key):
                print('序列在 %s 处失败，停止。' % key)
                return False
        return True

    def retract_all(self):
        print('\n收舵：/servo1../servo4/retract')
        results = [self.call(prefix + '/retract') for prefix in SERVO_CHANNELS]
        return all(results)

    def set_pulse(self, channel, pulse_us):
        prefix = '/servo%d' % channel
        if prefix not in self.publishers:
            print('未知通道：%d（应为 1..4）' % channel)
            return False
        print('\n直接设置 %s 脉宽 %.0f us（绕过 push/retract 判定，节点仍会做区间/状态检查）'
              % (prefix, pulse_us))
        self.publishers[prefix].publish(Float32(data=float(pulse_us)))
        time.sleep(0.5)
        self.show_status()
        return True


def confirm(prompt):
    try:
        return input(prompt).strip().lower() in ('y', 'yes', '是')
    except EOFError:
        return False


def interactive(test):
    while True:
        print('''\n========== 物资舵机测试 ==========
  1) 1 号物资（/servo3，2500）
  2) 2 号物资（/servo4，2500）
  3) 下放 3/4 号（/servo1 + /servo2，1800）
  4) 3 号物资（/servo3，2500）
  5) 4 号物资（/servo4，2500）
  6) 完整顺序 1→2→下放→3→4（每步确认）
  r) 全部收舵（retract）
  p) 自定义脉宽
  s) 显示状态
  q) 退出''')
        try:
            choice = input('选择：').strip().lower()
        except EOFError:
            return 0
        if choice in ('q', 'quit', 'exit'):
            return 0
        if choice == 's':
            test.show_status()
            continue
        if choice == 'r':
            if confirm('确认全部收舵？(y/N) '):
                test.retract_all()
            continue
        if choice == 'p':
            try:
                channel = int(input('通道 1..4：').strip())
                pulse = float(input('脉宽 us：').strip())
            except (ValueError, EOFError):
                print('输入无效')
                continue
            if confirm('确认向 /servo%d 发 %.0f us？(y/N) ' % (channel, pulse)):
                test.set_pulse(channel, pulse)
            continue
        if choice in ('1', '2', '3', '4', '5'):
            key = {'1': '1', '2': '2', '3': 'lower', '4': '3', '5': '4'}[choice]
            item = PAYLOADS[key]
            if confirm('确认执行「%s」？(y/N) ' % item['name']):
                test.run_payload(key)
            continue
        if choice == '6':
            print('完整序列会依次执行 1→2→下放→3→4，每步之间会询问。')
            if not confirm('开始？(y/N) '):
                continue
            for key in ('1', '2', 'lower', '3', '4'):
                item = PAYLOADS[key]
                if not confirm('  执行「%s」？(y/N) ' % item['name']):
                    print('  跳过')
                    continue
                test.run_payload(key)
            continue
        print('未知选择')


def main(argv):
    parser = argparse.ArgumentParser(description='四路物资舵机测试（不会解锁/发飞控指令）')
    parser.add_argument('--action', choices=['1', '2', '3', '4', 'lower', 'all', 'retract'],
                        help='非交互执行一个动作；不带参数进入交互菜单')
    parser.add_argument('--pulse', nargs=2, metavar=('CHANNEL', 'US'),
                        help='直接给 /servoCHANNEL/set_pulse_us 发脉宽')
    parser.add_argument('--status', action='store_true', help='只显示状态')
    parser.add_argument('--yes', action='store_true', help='脚本化执行时跳过确认（危险）')
    args = parser.parse_args(argv)

    rospy.init_node('payload_servo_test', anonymous=True, disable_signals=True)
    test = ServoTest()
    time.sleep(1.0)
    test.show_status()

    if args.status:
        return 0
    if args.pulse:
        channel, pulse = int(args.pulse[0]), float(args.pulse[1])
        if not args.yes and not confirm('确认向 /servo%d 发 %.0f us？(y/N) ' % (channel, pulse)):
            return 0
        return 0 if test.set_pulse(channel, pulse) else 1
    if args.action:
        if not args.yes and not confirm('确认执行「%s」？(y/N) ' % args.action):
            return 0
        if args.action == 'retract':
            ok = test.retract_all()
        elif args.action == 'all':
            ok = test.run_sequence()
        else:
            ok = test.run_payload(args.action)
        return 0 if ok else 1
    return interactive(test)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
