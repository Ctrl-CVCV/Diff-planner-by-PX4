#!/usr/bin/env python3
"""AUX 舵机脉宽与 PX4 归一化量之间的纯函数换算。

不导入 rospy，方便在没有 ROS 的机器上直接跑单元测试。

PX4 侧的对应关系：
  * 输出功能 301..306 = "Offboard Actuator Set 1..6"，分别由
    MAV_CMD_DO_SET_ACTUATOR 的 param1..param6 驱动；
  * 输出换算（``output_limit_calc``）是围绕 min/max 中点的对称映射，**没有 trim 项**：
    ``pulse = v * (max - min) / 2 + (min + max) / 2``。
    所以 ``PWM_AUX_MIN6=500`` / ``PWM_AUX_MAX6=2500`` 时：``500 ↔ -1.0``、``1500 ↔ 0.0``
    （中点）、``2200 ↔ +0.7``、``2500 ↔ +1.0``。
  * 部分固件还带 ``PWM_AUX_TRIMn``，那是**归一化小偏置**（约 ±0.2），不是 µs 中位；
    本包不读它，使用时保持 0。
"""

import math

#: MAVLink 命令号：直接设置执行机构输出
MAV_CMD_DO_SET_ACTUATOR = 187

#: "Offboard Actuator Set 1" 的输出功能号，1..6 连续
PERIPHERAL_SET_FIRST = 301
PERIPHERAL_SET_COUNT = 6

#: 本机 AUX6 实测配置（µs）：解锁后 500..2500，中点 1500；收起 500，推出 1400（归一化 -0.1）。
#: 正常运行时这些值必须从飞控参数读，只有 dry_run（无飞控）才拿它当兜底。
FIELD_MIN_US = 500.0
FIELD_MAX_US = 2500.0

#: MAV_RESULT：命令被接受
MAV_RESULT_ACCEPTED = 0


class ServoConfigError(ValueError):
    """参数或飞控配置不满足要求，节点应当拒绝动作并给出修复提示。"""


def _require_finite(name, value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ServoConfigError('%s 必须是数值，当前 %r' % (name, value))
    if not math.isfinite(value):
        raise ServoConfigError('%s 必须是有限数值，当前 %r' % (name, value))
    return float(value)


def validate_limits(min_us, max_us):
    """校验脉宽区间，返回 (min, max)；不合法时抛 ServoConfigError。"""
    min_us = _require_finite('min_us', min_us)
    max_us = _require_finite('max_us', max_us)
    if not min_us < max_us:
        raise ServoConfigError(
            '脉宽区间必须满足 min_us < max_us，当前 %.0f/%.0f' % (min_us, max_us))
    return min_us, max_us


def center_us(min_us, max_us):
    """归一化量 0 对应的脉宽：PX4 用 (min+max)/2，没有独立 trim。"""
    min_us, max_us = validate_limits(min_us, max_us)
    return (min_us + max_us) / 2.0


def half_span_us(min_us, max_us):
    """归一化量 ±1 对应的脉宽偏移量。"""
    min_us, max_us = validate_limits(min_us, max_us)
    return (max_us - min_us) / 2.0


def check_pulse_width(us, min_us, max_us, channel=None, prefix='PWM_AUX'):
    """确认目标脉宽能被当前 PWM 参数输出；超出区间时抛 ServoConfigError。"""
    us = _require_finite('pulse_us', us)
    min_us, max_us = validate_limits(min_us, max_us)
    if us < min_us or us > max_us:
        name = '%s_MIN%s/%s_MAX%s' % (prefix, channel, prefix, channel) if channel else (
            '<prefix>_MIN<ch>/<prefix>_MAX<ch>')
        raise ServoConfigError(
            '目标脉宽 %.0f us 超出当前输出区间 [%.0f, %.0f] us；'
            '要让舵机到该位置，先把 %s 调到覆盖它'
            % (us, min_us, max_us, name))
    return us


def pwm_to_normalized(us, min_us, max_us, channel=None, prefix='PWM_AUX'):
    """脉宽 -> PX4 归一化量，与 PX4 的对称映射互为逆运算。"""
    us = check_pulse_width(us, min_us, max_us, channel, prefix)
    return (us - center_us(min_us, max_us)) / half_span_us(min_us, max_us)


def normalized_to_pwm(value, min_us, max_us):
    """PX4 归一化量 -> 脉宽，用来把飞控实际会输出的脉宽回算给日志。"""
    value = _require_finite('normalized', value)
    return center_us(min_us, max_us) + value * half_span_us(min_us, max_us)


def actuator_set_index(function_value):
    """把输出功能号（301..306）换算成 DO_SET_ACTUATOR 的序号（1..6）。

    其它功能号（0=关闭、101..=电机、400=起落架、401=伞、430=夹爪……）一律返回 None，
    因为这些功能不接受 MAV_CMD_DO_SET_ACTUATOR 的直接驱动。
    """
    if isinstance(function_value, bool) or not isinstance(function_value, (int, float)):
        return None
    if not math.isfinite(float(function_value)):
        return None
    value = int(round(float(function_value)))
    if PERIPHERAL_SET_FIRST <= value < PERIPHERAL_SET_FIRST + PERIPHERAL_SET_COUNT:
        return value - PERIPHERAL_SET_FIRST + 1
    return None


def command_params(index, normalized):
    """构造 MAV_CMD_DO_SET_ACTUATOR 的 param1..param6。

    只填目标那一路，其余填 NaN：PX4 对非有限值不做处理，不会顺手把别人的输出
    （云台、其它外设）拉回中位。
    """
    if not 1 <= int(index) <= PERIPHERAL_SET_COUNT:
        raise ServoConfigError('actuator set 序号必须在 1..6，当前 %r' % (index,))
    value = _require_finite('normalized', normalized)
    if not -1.0 - 1e-9 <= value <= 1.0 + 1e-9:
        raise ServoConfigError('归一化量必须在 [-1, 1]，当前 %.6f' % value)
    value = max(-1.0, min(1.0, value))
    params = [float('nan')] * PERIPHERAL_SET_COUNT
    params[int(index) - 1] = value
    return params


def describe_output_function(function_value, channel, prefix='PWM_AUX'):
    """把读取到的输出功能号翻译成人能看懂的修复提示。"""
    index = actuator_set_index(function_value)
    if index is not None:
        return '输出功能 %d（Offboard Actuator Set %d），由 DO_SET_ACTUATOR 的 param%d 驱动' % (
            int(round(float(function_value))), index, index)
    hints = {
        0: 'Disabled，该通道不输出',
        400: 'Landing Gear，由起落架指令驱动',
        401: 'Parachute，由 MAV_CMD_DO_PARACHUTE 驱动',
        420: 'Gimbal Roll，由云台指令驱动',
        430: 'Gripper，由抓取指令驱动',
    }
    text = hints.get(int(round(float(function_value))), '不是 Offboard Actuator Set')
    return '输出功能 %d（%s）；请把 %s_FUNC%d 设成 %d..%d 中的一项（如 306 = Offboard Actuator Set 6）' % (
        int(round(float(function_value))), text, prefix, channel,
        PERIPHERAL_SET_FIRST, PERIPHERAL_SET_FIRST + PERIPHERAL_SET_COUNT - 1)
