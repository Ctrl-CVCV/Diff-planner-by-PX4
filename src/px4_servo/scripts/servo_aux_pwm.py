#!/usr/bin/env python3
"""通过 MAV_CMD_DO_SET_ACTUATOR 驱动 PX4 AUX 输出上的舵机（默认 AUX6）。

工作方式
  1. 飞控侧把该 AUX 通道的输出功能设成 "Offboard Actuator Set N"（301..306）；
     本节点读 ``<prefix>_FUNC<ch>`` 自己推断 N，不猜通道号。
  2. 节点把目标脉宽（µs）按该通道的 ``_MIN`` / ``_MAX`` 换算成归一化量，
     以 ``publish_hz`` 持续发 MAV_CMD_DO_SET_ACTUATOR，只填 paramN，其余为 NaN，
     不会碰云台/其它外设输出。
  3. 想输出 2500 µs 就必须保证 ``PWM_AUX_MAX6`` ≥ 2500：PX4 按
     ``pulse = v*(max-min)/2 + (min+max)/2`` 把归一化量还原成脉宽（没有 trim 项），
     节点只是把这件事显式算出来并检查。

安全约定
  * 只在 ``/mavros/state`` 显示 connected 时下发；可选用 ``arming_gate`` 限定
    只在 armed / disarmed 时动作。
  * 输出是锁存的：节点退出不会自动回位；回位要显式调用 ``/servo/retract``。
  * 上锁状态下 PX4 输出的是 ``PWM_AUX_DIS<ch>``，不是本节点算的位置。本机配置
    DIS6 = 500 µs = 收起位，所以上锁时舵机就停在收起位，和 rest_us 一致。
"""

import json
import math
import os
import sys
import threading
import time

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong, ParamGet
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger, TriggerResponse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import servo_map  # noqa: E402


def _clamp(value, low, high):
    return max(low, min(high, value))


class AuxServo(object):

    def __init__(self):
        self.ns = str(rospy.get_param('~mavros_ns', '/mavros')).rstrip('/')
        self.channel = int(rospy.get_param('~channel', 6))
        self.prefix = str(rospy.get_param('~param_prefix', 'PWM_AUX'))
        # AUX6 实测配置：解锁区间 500..2500 µs（中点 1500，PX4 无独立 trim）；500 收起。
        # 2026-10-31 实测推出行程 1400 µs（归一化 -0.1），不是区间上限 2500。
        self.rest_us = float(rospy.get_param('~rest_us', 500.0))
        self.push_us = float(rospy.get_param('~push_us', 1400.0))
        # 0 = 推出后一直保持，直到 /servo/retract 或下一条动作指令（本机投放要求不自动收回）。
        self.hold_s = float(rospy.get_param('~hold_s', 0.0))
        self.publish_hz = float(rospy.get_param('~publish_hz', 5.0))
        self.arming_gate = str(rospy.get_param('~arming_gate', 'any'))
        self.param_timeout = float(rospy.get_param('~param_timeout', 10.0))
        self.param7 = float(rospy.get_param('~param7', 0.0))
        self.shutdown_retract = bool(rospy.get_param('~shutdown_retract', False))
        self.warn_when_disarmed = bool(rospy.get_param('~warn_when_disarmed', True))
        self.dry_run = bool(rospy.get_param('~dry_run', False))
        self.topic_prefix = str(rospy.get_param('~topic_prefix', '/servo')).rstrip('/')
        override = rospy.get_param('~limits_override', {}) or {}

        if not 1 <= self.channel <= 8:
            raise servo_map.ServoConfigError('~channel 必须在 1..8，当前 %d' % self.channel)
        if not 0.5 <= self.publish_hz <= 50.0:
            raise servo_map.ServoConfigError('~publish_hz 必须在 [0.5, 50]，当前 %r' % self.publish_hz)
        if self.hold_s < 0.0:
            raise servo_map.ServoConfigError('~hold_s 不能为负')
        if self.arming_gate not in ('any', 'armed', 'disarmed'):
            raise servo_map.ServoConfigError(
                "~arming_gate 只能是 'any' / 'armed' / 'disarmed'，当前 %r" % self.arming_gate)
        self.override = {key: float(override.get(key, 0.0) or 0.0)
                         for key in ('min_us', 'max_us')}

        self.state = None
        self.index = None
        self.function_value = None
        self.min_us = None
        self.max_us = None
        self.center_us = None
        self.disarmed_us = None
        self.limits = None
        self.ready_error = None
        self.rest_ok = False
        self.push_ok = False

        self.command_us = self.rest_us
        self.revert_at = None
        self.revert_to = self.rest_us
        self.last_send = 0.0
        self.last_result = None
        self.acks = {'ok': 0, 'fail': 0}
        self._lock = threading.Lock()
        self._disarmed_warned = False
        self._dry_run_last = None

        rospy.Subscriber(self.ns + '/state', State, self._on_state, queue_size=1)
        self.param_srv = None
        self.cmd_srv = None
        self.state_pub = rospy.Publisher(self.topic_prefix + '/state', String, queue_size=1)
        rospy.Subscriber(self.topic_prefix + '/set_pulse_us', Float32, self._on_set_pulse, queue_size=1)
        rospy.Service(self.topic_prefix + '/push', Trigger, self._on_push)
        rospy.Service(self.topic_prefix + '/retract', Trigger, self._on_retract)
        rospy.Service(self.topic_prefix + '/reload', Trigger, self._on_reload)

    # ---------------------------------------------------------------- 飞控参数

    def _get_param(self, name):
        if self.param_srv is None:
            return None
        try:
            response = self.param_srv(param_id=name)
        except rospy.ServiceException as exc:
            rospy.logwarn('读取参数 %s 失败：%s', name, exc)
            return None
        if not response.success:
            return None
        integer, real = response.value.integer, response.value.real
        return float(integer) if integer not in (0, 0.0) else float(real)

    def _set_limits(self, min_us, max_us):
        """校验并记录脉宽区间；中点由 PX4 的 (min+max)/2 规则算出。"""
        min_us, max_us = servo_map.validate_limits(min_us, max_us)
        self.min_us = min_us
        self.max_us = max_us
        self.center_us = servo_map.center_us(min_us, max_us)
        self.limits = (min_us, max_us)

    def _probe_output_params(self):
        """FUNC 读不到时，列出飞控上真实存在的输出功能参数名，便于直接改 param_prefix / channel。"""
        existing = []
        for prefix in ('PWM_AUX', 'PWM_MAIN'):
            for channel in range(1, 9):
                name = '%s_FUNC%d' % (prefix, channel)
                value = self._get_param(name)
                if value is not None:
                    existing.append('%s=%d' % (name, int(round(value))))
        if not existing:
            return ('PWM_AUX_FUNC*/PWM_MAIN_FUNC* 一个都读不到：可能飞控不是 PX4、'
                    'MAVROS 参数还没同步（试 /mavros/param/pull），或该机型没有这些输出')
        return '飞控上实际存在：' + ', '.join(existing)

    def reload(self):
        """重新连接 MAVROS 并读取输出功能 / 脉宽区间，失败时记录原因但不抛异常。"""
        self.index = None
        self.limits = None
        self.min_us = None
        self.max_us = None
        self.center_us = None
        self.disarmed_us = None
        self.function_value = None
        self.ready_error = None
        with self._lock:
            self.rest_ok = self.push_ok = False
            self.command_us = self.rest_us
            self.revert_at = None

        if self.dry_run:
            # 不连飞控时按本机实测配置兜底，保证 dry_run:=true 直接可用
            min_us = self.override['min_us'] or servo_map.FIELD_MIN_US
            max_us = self.override['max_us'] or servo_map.FIELD_MAX_US
            try:
                self._set_limits(min_us, max_us)
                self.index = int(rospy.get_param('~dry_run_index', self.channel))
                self.function_value = servo_map.PERIPHERAL_SET_FIRST + self.index - 1
                self._check_targets()
                rospy.logwarn('dry_run：只打印指令，不调用 MAVROS；序号按 %d 假定', self.index)
            except servo_map.ServoConfigError as exc:
                self.ready_error = str(exc)
            return

        try:
            rospy.wait_for_service(self.ns + '/cmd/command', timeout=self.param_timeout)
            rospy.wait_for_service(self.ns + '/param/get', timeout=self.param_timeout)
        except rospy.ROSException:
            self.ready_error = '连不上 %s 的 /cmd/command 或 /param/get，先确认 MAVROS 已启动并连上飞控' % self.ns
            return
        self.cmd_srv = rospy.ServiceProxy(self.ns + '/cmd/command', CommandLong)
        self.param_srv = rospy.ServiceProxy(self.ns + '/param/get', ParamGet)

        func_name = '%s_FUNC%d' % (self.prefix, self.channel)
        self.function_value = self._get_param(func_name)
        if self.function_value is None:
            self.ready_error = '读不到飞控参数 %s（飞控上没有这个名字）；%s；先把该通道的输出功能设成 %d..%d 中的一项' % (
                func_name, self._probe_output_params(),
                servo_map.PERIPHERAL_SET_FIRST,
                servo_map.PERIPHERAL_SET_FIRST + servo_map.PERIPHERAL_SET_COUNT - 1)
            return
        self.index = servo_map.actuator_set_index(self.function_value)
        if self.index is None:
            self.ready_error = servo_map.describe_output_function(
                self.function_value, self.channel, self.prefix)
            return

        # PX4 用 (min+max)/2 作为归一化量 0 的脉宽，没有独立 trim 参数
        values = {}
        for key, suffix in (('min_us', 'MIN'), ('max_us', 'MAX')):
            name = '%s_%s%d' % (self.prefix, suffix, self.channel)
            values[key] = self.override[key] or self._get_param(name)
            if values[key] is None:
                self.ready_error = '读不到飞控参数 %s，无法把 µs 换算成归一化量' % name
                return
        try:
            self._set_limits(values['min_us'], values['max_us'])
        except servo_map.ServoConfigError as exc:
            self.ready_error = str(exc)
            return
        # 上锁输出（DIS）只用于核对，缺失不影响动作
        self.disarmed_us = self._get_param('%s_DIS%d' % (self.prefix, self.channel))
        if self.disarmed_us is not None and abs(self.disarmed_us - self.rest_us) > 1.0:
            rospy.logwarn('%s_DIS%d = %.0f us，与收起位 rest_us = %.0f us 不一致：'
                          '上锁时舵机会停在前者而不是收起位',
                          self.prefix, self.channel, self.disarmed_us, self.rest_us)
        self._check_targets()

    def _check_targets(self):
        """分别判断 rest / push 是否能被当前脉宽区间输出，不合格的只禁用对应动作。"""
        for name, us in (('rest', self.rest_us), ('push', self.push_us)):
            try:
                servo_map.check_pulse_width(us, self.min_us, self.max_us,
                                            self.channel, self.prefix)
                setattr(self, name + '_ok', True)
            except servo_map.ServoConfigError as exc:
                setattr(self, name + '_ok', False)
                rospy.logerr('%s 位不可用：%s', '推出' if name == 'push' else '收起', exc)
        summary = ('AUX%d：%s；区间 %.0f..%.0f us（中点 %.0f）；rest %.0f us；push %.0f us'
                   % (self.channel,
                      servo_map.describe_output_function(
                          self.function_value, self.channel, self.prefix),
                      self.min_us, self.max_us, self.center_us,
                      self.rest_us, self.push_us))
        if self.push_ok:
            normalized = servo_map.pwm_to_normalized(self.push_us, self.min_us, self.max_us,
                                                     self.channel, self.prefix)
            summary += '（归一化 %.3f）' % normalized
        if self.rest_ok:
            summary += '；收起归一化 %.3f' % servo_map.pwm_to_normalized(
                self.rest_us, self.min_us, self.max_us, self.channel, self.prefix)
        if self.disarmed_us is not None:
            summary += '；上锁输出 %.0f us' % self.disarmed_us
        rospy.loginfo(summary)

    # ---------------------------------------------------------------- 回调

    def _on_state(self, message):
        self.state = message

    def _on_push(self, request):
        if not self._gate('push'):
            return TriggerResponse(success=False, message=self.ready_error or '飞控状态不满足')
        return self._apply(self.push_us, 'push')

    def _on_retract(self, request):
        if not self.rest_ok:
            return TriggerResponse(success=False, message=self.ready_error or '收起位不可用')
        return self._apply(self.rest_us, 'retract')

    def _on_set_pulse(self, message):
        if not self._gate('set_pulse'):
            rospy.logwarn('忽略手动脉宽 %.0f us：%s', message.data, self.ready_error or '飞控状态不满足')
            return
        self._apply(float(message.data), 'set_pulse')

    def _on_reload(self, request):
        self.reload()
        return TriggerResponse(success=self.ready_error is None,
                               message=self.ready_error or '参数已重新读取')

    def _gate(self, action):
        """动作前的健康检查；不通过时返回 False 并给出原因。"""
        if self.ready_error:
            rospy.logwarn_throttle(5.0, '拒绝 %s：%s', action, self.ready_error)
            return False
        if not self.dry_run and (self.state is None or not self.state.connected):
            rospy.logwarn_throttle(5.0, '拒绝 %s：尚未收到 /mavros/state 的 connected=True', action)
            return False
        if self.arming_gate != 'any' and self.state is not None:
            if self.arming_gate == 'armed' and not self.state.armed:
                rospy.logwarn_throttle(5.0, '拒绝 %s：~arming_gate=armed 但飞控未解锁', action)
                return False
            if self.arming_gate == 'disarmed' and self.state.armed:
                rospy.logwarn_throttle(5.0, '拒绝 %s：~arming_gate=disarmed 但飞控已解锁', action)
                return False
        return True

    def _apply(self, pulse_us, action):
        if not self.limits:
            rospy.logerr('拒绝 %s：还没读到飞控的脉宽参数', action)
            return TriggerResponse(success=False, message=self.ready_error or '参数未就绪')
        try:
            pulse_us = servo_map.check_pulse_width(pulse_us, self.min_us, self.max_us,
                                                   self.channel, self.prefix)
        except servo_map.ServoConfigError as exc:
            rospy.logerr('拒绝 %s：%s', action, exc)
            return TriggerResponse(success=False, message=str(exc))
        with self._lock:
            self.command_us = pulse_us
            if self.hold_s > 0.0 and pulse_us != self.rest_us:
                self.revert_at = time.monotonic() + self.hold_s
                self.revert_to = self.rest_us
            else:
                self.revert_at = None
        normalized = servo_map.pwm_to_normalized(pulse_us, self.min_us, self.max_us,
                                                 self.channel, self.prefix)
        rospy.loginfo('%s：发 %.0f us（归一化 %.3f），保持 %s',
                      action, pulse_us, normalized,
                      '%.2fs 后自动回 %.0f us' % (self.hold_s, self.rest_us)
                      if self.revert_at else '直到下一次指令')
        self._send(normalized)
        return TriggerResponse(success=True, message='%s -> %.0f us' % (action, pulse_us))

    # ---------------------------------------------------------------- 下发

    def _send(self, normalized):
        if not self.dry_run and self.cmd_srv is None:
            rospy.logwarn_throttle(5.0, '还没连上 %s/cmd/command，跳过本次下发', self.ns)
            return
        params = servo_map.command_params(self.index, normalized)
        if self.dry_run:
            # 持续下发时只在本值变化时打印，避免 5 Hz 刷屏
            if normalized != self._dry_run_last:
                self._dry_run_last = normalized
                rospy.loginfo('[dry_run] MAV_CMD_DO_SET_ACTUATOR(187) params=%s param7=%.1f',
                              ['nan' if math.isnan(p) else round(p, 4) for p in params],
                              self.param7)
            return
        if self.state is not None and not self.state.armed and self.warn_when_disarmed \
                and not self._disarmed_warned:
            self._disarmed_warned = True
            actual = ('PWM_AUX_DIS%d = %.0f us' % (self.channel, self.disarmed_us)
                      if self.disarmed_us is not None
                      else 'PWM_AUX_DIS%d（没读到）' % self.channel)
            rospy.logwarn('飞控未解锁：AUX 输出此时是 %s，不是本节点算出的位置；收起位 %.0f us',
                          actual, self.rest_us)
        try:
            response = self.cmd_srv(command=servo_map.MAV_CMD_DO_SET_ACTUATOR,
                                    confirmation=0,
                                    param1=params[0], param2=params[1], param3=params[2],
                                    param4=params[3], param5=params[4], param6=params[5],
                                    param7=self.param7)
        except rospy.ServiceException as exc:
            self.acks['fail'] += 1
            if rospy.is_shutdown():
                # Ctrl-C 时调用正卡在飞控应答上，连接被拆掉会抛 "returned no response"；
                # 这不是故障，静默退出即可
                return
            rospy.logwarn_throttle(5.0, 'DO_SET_ACTUATOR 调用失败：%s'
                                   '（MAVROS 没应答：多半是它到飞控的链路断了）', exc)
            return
        self.last_result = int(response.result)
        if response.success and self.last_result == servo_map.MAV_RESULT_ACCEPTED:
            self.acks['ok'] += 1
        else:
            self.acks['fail'] += 1
            rospy.logwarn_throttle(5.0, 'DO_SET_ACTUATOR 被拒绝：success=%s result=%d'
                                   '（常见原因：%s_FUNC%d 不是 Offboard Actuator Set，'
                                   '或飞控固件不支持 187）',
                                   response.success, self.last_result, self.prefix, self.channel)

    def _tick(self, event):
        if rospy.is_shutdown():
            return
        now = time.monotonic()
        with self._lock:
            if self.revert_at is not None and now >= self.revert_at:
                self.command_us = self.revert_to
                self.revert_at = None
            target = self.command_us
        if not self._gate('tick'):
            return
        try:
            normalized = servo_map.pwm_to_normalized(target, self.min_us, self.max_us,
                                                     self.channel, self.prefix)
        except servo_map.ServoConfigError:
            return
        self._send(normalized)

    def _publish_state(self, event):
        with self._lock:
            target = self.command_us
            revert_at = self.revert_at
        normalized = None
        limits = None
        if self.limits:
            limits = [int(round(value)) for value in
                      (self.min_us, self.center_us, self.max_us)]
            try:
                normalized = round(servo_map.pwm_to_normalized(
                    target, *self.limits, self.channel, self.prefix), 4)
            except servo_map.ServoConfigError:
                normalized = None
        payload = {
            'ready': self.ready_error is None,
            'reason': self.ready_error,
            'channel': self.channel,
            'function': None if self.function_value is None else int(round(self.function_value)),
            'actuator_set': self.index,
            'limits_us': limits,
            'disarmed_us': None if self.disarmed_us is None else int(round(self.disarmed_us)),
            'disarmed_matches_rest': (None if self.disarmed_us is None
                                      else abs(self.disarmed_us - self.rest_us) <= 1.0),
            'commanded_us': int(round(target)),
            'normalized': normalized,
            'holding': revert_at is not None,
            'rest_ok': self.rest_ok,
            'push_ok': self.push_ok,
            'connected': None if self.state is None else bool(self.state.connected),
            'armed': None if self.state is None else bool(self.state.armed),
            'acks': dict(self.acks),
            'last_result': self.last_result,
            'dry_run': self.dry_run,
        }
        self.state_pub.publish(String(data=json.dumps(payload, ensure_ascii=False,
                                                      separators=(',', ':'))))

    # ---------------------------------------------------------------- 生命周期

    def run(self):
        self.reload()
        if self.ready_error:
            rospy.logerr('节点已启动但当前不可动作：%s（修好后调用 %s/reload）',
                         self.ready_error, self.topic_prefix)
        rospy.Timer(rospy.Duration(1.0 / self.publish_hz), self._tick)
        rospy.Timer(rospy.Duration(1.0), self._publish_state)
        self._publish_state(None)

    def shutdown(self):
        if self.shutdown_retract and self.limits and self.rest_ok and not self.dry_run:
            try:
                normalized = servo_map.pwm_to_normalized(self.rest_us, *self.limits,
                                                channel=self.channel, prefix=self.prefix)
                self._send(normalized)
                rospy.loginfo('退出前已下发收起位 %.0f us（尽力而为，不保证送达）', self.rest_us)
            except (servo_map.ServoConfigError, rospy.ServiceException) as exc:
                rospy.logwarn('退出前回位失败：%s', exc)


def main():
    rospy.init_node('servo_aux_pwm')
    try:
        node = AuxServo()
    except servo_map.ServoConfigError as exc:
        rospy.logfatal('参数配置错误：%s', exc)
        return 1
    rospy.on_shutdown(node.shutdown)
    node.run()
    rospy.spin()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
