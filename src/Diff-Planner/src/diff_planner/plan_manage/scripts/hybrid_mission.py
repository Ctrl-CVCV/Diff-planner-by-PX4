#!/usr/bin/env python3
"""Automatic takeoff; planner through B to C; direct PX4Ctrl to D; planner to E and F;
direct PX4Ctrl to G and H; planner back to I; hover, descend close to the ground, then land.

Planner legs: A->B, B->C, D->E, E->F, H->I. Direct PX4Ctrl legs: C->D, F->G, G->H.
B, C, D, E, F and G dwell mission/arrival_dwell (3 s) after arrival; I dwells
mission/hover_seconds before landing.

H is a pass-through waypoint: the next goal is sent as soon as the vehicle enters
mission/pass_radius around it, without a dwell, while the script keeps tracking the
previous direct reference instead of freezing at the waypoint.

危险品投放（hazard_drop/enabled，2026-10-31 起实机配置默认开启，可用 --no-hazard-drop 关）：
在 hazard_drop/points（默认 C、D）连续稳定悬停 hover_seconds 秒、且最近 vote_frames 帧里
"离图像中心最近的目标是 label 且置信度达标"的比例达到 vote_ratio 时，先在位置控制下降到
离地 height，调用 hazard_drop/service（默认 /servo/push）推出舵机（px4_servo 用 hold_s=0，
推出后不自动收回），保持 push_wait 秒后爬回该航点高度继续任务。到点后最多等 wait_timeout 秒，
超时只告警并继续，不中止飞行。启用时预检要求舵机服务可用。

Before landing the script descends under position control to landing/approach_height
above the takeoff ground (x, y still commanded to I) and only then asks PX4Ctrl to land,
so the AUTO_LAND ramp starts from a pose that is already the landing pose.
Offline by default.
"""

import argparse
import copy
from collections import deque
import json
import math
from pathlib import Path
import threading
import time

import yaml


class FlightError(RuntimeError):
    def __init__(self, message, land_allowed=False):
        super().__init__(message)
        self.land_allowed = land_allowed


def norm(values):
    return math.sqrt(sum(value * value for value in values))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def vector(message):
    return (message.x, message.y, message.z)


def yaw_of(quaternion):
    return math.atan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      1 - 2 * (quaternion.y ** 2 + quaternion.z ** 2))


def hover_point(config):
    point = copy.deepcopy(config['points']['A'])
    if config.get('coordinate_mode', 'world') == 'takeoff_relative':
        point['xyz'][2] += config['takeoff']['height']
    return point


def resolve_config(config, origin):
    resolved = copy.deepcopy(config)
    if config.get('coordinate_mode', 'world') == 'takeoff_relative':
        for point in resolved['points'].values():
            point['xyz'] = [offset + start for offset, start in zip(point['xyz'], origin)]
    return resolved


def unmeasured_points(config):
    """尚未实测的航点（配置里 measured: false）；有此类点时禁止实机执行。"""
    return sorted(label for label, point in config['points'].items()
                  if point.get('measured', True) is False)


def direct_duration(start, start_yaw, target, limits):
    distance = math.dist(start, target['xyz'])
    yaw_distance = abs(wrap(math.radians(target['yaw_deg']) - start_yaw))
    return max(limits['min_duration'], 1.875 * distance / limits['max_speed'],
               math.sqrt((10 / math.sqrt(3)) * distance / limits['max_acceleration']),
               (60 * distance / limits['max_jerk']) ** (1 / 3),
               1.875 * yaw_distance / math.radians(limits['max_yaw_rate_deg']))


def direct_sample(start, start_yaw, target, duration, elapsed):
    phase = max(0.0, min(1.0, elapsed / duration))
    blend = 10 * phase ** 3 - 15 * phase ** 4 + 6 * phase ** 5
    speed = (30 * phase ** 2 - 60 * phase ** 3 + 30 * phase ** 4) / duration
    acceleration = (60 * phase - 180 * phase ** 2 + 120 * phase ** 3) / duration ** 2
    jerk = (60 - 360 * phase + 360 * phase ** 2) / duration ** 3 if 0 < elapsed < duration else 0.0
    delta = tuple(end - begin for begin, end in zip(start, target['xyz']))
    yaw_delta = wrap(math.radians(target['yaw_deg']) - start_yaw)
    return dict(position=tuple(begin + change * blend for begin, change in zip(start, delta)),
                velocity=tuple(change * speed for change in delta),
                acceleration=tuple(change * acceleration for change in delta),
                jerk=tuple(change * jerk for change in delta),
                yaw=wrap(start_yaw + yaw_delta * blend), yaw_dot=yaw_delta * speed)


def mission_legs():
    """固定航段表。pass_through 的航点一进入 pass_radius 就切下一段，不悬停。"""
    return (
        # B 不再是经过点：到点后稳定悬停 arrival_dwell 秒再发 C。
        # 原因：B→C 是一个 139° 的折返，带着过 B 的速度直接规划会让优化器在转弯处超速
        # （2026-10-31 多次触发 checkDynamicFeasibility 连拒 → replan fail too much → 急停，
        # 任务因此以"无有效位移"中止）。从静止重新规划，起点速度为零，转弯不必先消掉惯性。
        dict(target='B', mode='planner'),
        # C 接在 B 之后：等待 C 轨迹期间继续转发上一条 Planner 指令（B 的悬停），不中断控制。
        dict(target='C', mode='planner', bridge='planner'),
        dict(target='D', mode='direct'),
        dict(target='E', mode='planner'),
        # F 是 Planner 与直控的切换点：到点后同样稳定悬停，再开始直控。
        dict(target='F', mode='planner'),
        dict(target='G', mode='direct'),
        # H 也是经过点：进入 pass_radius 就发 I 目标；等待期间继续跟踪 G→H 的直控参考。
        dict(target='H', mode='direct', pass_through=True),
        dict(target='I', mode='planner'),
    )


def previous_anchor(label, config):
    """某航段的起点：首个航段从起飞悬停点出发，其余从上一航点出发。"""
    legs = mission_legs()
    index = [leg['target'] for leg in legs].index(label)
    if index == 0:
        return hover_point(config)['xyz']
    return config['points'][legs[index - 1]['target']]['xyz']


def arrived(position, velocity, yaw, point, safety):
    return (math.dist(position, point['xyz']) <= safety['position_tolerance']
            and norm(velocity) <= safety['arrival_speed']
            and abs(wrap(yaw - math.radians(point['yaw_deg']))) <= math.radians(safety['yaw_tolerance_deg']))


class MotionWatchdog:
    def __init__(self, now, position):
        self.time = now
        self.position = position

    def check(self, now, position, target, safety):
        if math.dist(position, target) <= safety['position_tolerance']:
            self.time, self.position = now, position
        elif math.dist(position, self.position) >= safety['motion_distance']:
            self.time, self.position = now, position
        elif now - self.time >= safety['motion_timeout']:
            raise FlightError('目标发出后 %.1fs 内无有效位移' % safety['motion_timeout'], True)


class Mission:
    def __init__(self, ros, config, read_only=False):
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import ExtendedState, RCIn, State
        from nav_msgs.msg import Odometry
        from quadrotor_msgs.msg import PositionCommand, Px4ctrlState, TakeoffLand
        from std_msgs.msg import Empty, String

        self.ros, self.config = ros, config
        self.source_config = copy.deepcopy(config)
        self.origin = None
        self.read_only = read_only
        self.safety, self.settings = config['safety'], config['mission']
        self.types = dict(goal=PoseStamped, command=PositionCommand, land=TakeoffLand, stop=Empty, status=String)
        self.lock = threading.Lock()
        self.data = {}
        self.sustained = {}
        self.stage = 'PREFLIGHT'
        self.active = False
        self.takeoff_sent = False
        self.land_sent = False
        self.stopped = False
        self.publishers = {}
        if not read_only:
            self.publishers = {key: ros.Publisher(config['topics'][topic], self.types[key], queue_size=1,
                                                latch=(key == 'status'))
                               for key, topic in (('goal', 'goal'), ('command', 'output_cmd'),
                                                  ('land', 'land'), ('stop', 'stop'), ('status', 'status'))}
        message_types = dict(odom=Odometry, lio=Odometry, fcu=State, extended=ExtendedState, rc=RCIn,
                             ctrl=Px4ctrlState, planner_cmd=PositionCommand)
        # 投放关闭时完全不订阅检测话题：默认任务流与之前一致，YOLO 是否在跑都不影响任务。
        if config.get('hazard_drop', {}).get('enabled', False):
            message_types['detections'] = String
        self.subscribers = [ros.Subscriber(config['topics'][key], message_type, self.receive,
                                           callback_args=key, queue_size=1, tcp_nodelay=True)
                            for key, message_type in message_types.items()]
        # 最近若干帧的检测投票：(到达时刻, 该帧是否命中)。是否确认真危险品由投票决定。
        self.hazard_votes = deque()
        self.hazard_last = None
        self.drop_done = False

    def receive(self, message, key):
        now = time.monotonic()
        with self.lock:
            self.data[key] = (message, now)
            if key == 'detections':
                votes = getattr(self, 'hazard_votes', None)
                if votes is None:
                    votes = self.hazard_votes = deque()
                votes.append((now, self.hazard_frame(message)))
                limit = self.vote_limit()
                while len(votes) > limit:
                    votes.popleft()
                self.hazard_last = now

    def get(self, key, timeout, stamped=False):
        with self.lock:
            entry = self.data.get(key)
        topic = self.config['topics'][key]
        if entry is None:
            raise FlightError('%s 未收到：话题=%s，阶段=%s，超时阈值=%.3fs' %
                              (key, topic, self.stage, timeout))
        age = time.monotonic() - entry[1]
        if age > timeout:
            detail = ''
            if key == 'fcu':
                detail = '，最近状态 connected=%s armed=%s mode=%s' % (
                    entry[0].connected, entry[0].armed, entry[0].mode)
            raise FlightError('%s 断更：话题=%s，阶段=%s，距上次接收=%.3fs > 阈值=%.3fs%s' %
                              (key, topic, self.stage, age, timeout, detail))
        message = entry[0]
        if stamped:
            age = self.ros.Time.now().to_sec() - message.header.stamp.to_sec()
            if message.header.stamp.to_sec() <= 0 or not -self.safety['max_future_stamp'] <= age <= self.safety['max_header_age']:
                raise FlightError(key + ' 时间戳过期或超前')
        return message

    def graph(self):
        publishers, subscribers, _ = self.ros.get_master().getSystemState()[2]
        publishers, subscribers = dict(publishers), dict(subscribers)
        topics, own = self.config['topics'], self.ros.get_name()
        expected = set() if self.read_only else {own}
        for key in ('output_cmd', 'goal', 'land', 'stop'):
            if set(publishers.get(topics[key], [])) != expected:
                raise FlightError(topics[key] + ' 存在其他发布者或本节点未注册，拒绝争抢控制')
        if set(publishers.get(topics['planner_cmd'], [])) != {self.config['traj_server_node']}:
            raise FlightError('Planner 输入源不唯一/未隔离，请使用 run_exp_hybrid_mission.launch')
        if self.config['controller_node'] not in subscribers.get(topics['output_cmd'], []):
            raise FlightError('PX4Ctrl 未订阅任务控制出口')
        if self.config['planner_node'] not in subscribers.get(topics['goal'], []):
            raise FlightError('Planner 未订阅目标')
        if self.config['planner_node'] not in subscribers.get(topics['stop'], []):
            raise FlightError('Planner 未订阅 mandatory stop')
        if self.config['controller_node'] not in subscribers.get(topics['land'], []):
            raise FlightError('PX4Ctrl 未订阅降落请求')

    def health(self, landing=False, phase='flight'):
        if self.ros.is_shutdown():
            raise FlightError('ROS 已关闭')
        safety = self.safety
        fcu = self.get('fcu', safety['fcu_timeout'])
        ctrl = self.get('ctrl', safety['telemetry_timeout'], True)
        rc = self.get('rc', safety['rc_timeout'])
        if not fcu.connected:
            raise FlightError('飞控连接丢失')
        if phase == 'ground' and (fcu.armed or ctrl.state != 1):
            raise FlightError('程控起飞要求未解锁且 PX4Ctrl 为 MANUAL_CTRL(1)，当前=%s；禁止重复起飞' % ctrl.state)
        if phase == 'flight' and (not fcu.armed or fcu.mode != 'OFFBOARD'):
            raise FlightError('需要已解锁且 OFFBOARD；模式退出后不会强行切回')
        if len(rc.channels) < 8 or min(rc.channels[4:6]) <= 1750 or any(abs(channel - 1500) > 125 for channel in rc.channels[:4]):
            raise FlightError('RC 接管/状态异常：CH5、CH6 应高位，四个摇杆居中')
        allowed = (1,) if phase == 'ground' else (1, 2, 4) if phase == 'takeoff' else (2, 3, 5) if landing else (2, 3)
        if ctrl.state not in allowed:
            raise FlightError('PX4Ctrl 状态异常/飞手接管，当前=%s' % ctrl.state)
        odom = self.get('odom', safety['telemetry_timeout'], True)
        if odom.header.frame_id != self.config['frame_id']:
            raise FlightError('EKF frame_id 与控制出口不符；脚本不做 TF 坐标系转换')
        position, velocity = vector(odom.pose.pose.position), vector(odom.twist.twist.linear)
        quaternion = odom.pose.pose.orientation
        components = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        if not all(math.isfinite(value) for value in (*position, *velocity, *components)) or not 0.9 <= norm(components) <= 1.1:
            raise FlightError('EKF 包含非有限值/无效姿态')
        if norm(velocity) > safety['max_measured_speed']:
            raise FlightError('实测速度超限，怀疑定位异常，立即人工接管')
        self.check_estimator(position)
        self.check_fence(position)
        return position, velocity, yaw_of(quaternion)

    def estimator_sample(self):
        """读取 /Odometry 并返回 world 位置。

        LIO 的 header.stamp 不参与任何中止判定（2026-10-26 决定，2026-10-29 再次确认）：
        即使该字段为零、过旧或超前，只要消息持续及时到达就继续任务。这里只检查位置数值有效。
        """
        # LIO 是 10 Hz 话题，不能套用高频遥测的接收超时：2026-10-29 实测间隔中位数
        # 0.098s、p99 0.204s、最大 0.346s，用 0.30s 会在正常抖动时误判断更。
        lio = self.get('lio', self.safety.get('lio_timeout', 1.0))
        position = vector(lio.pose.pose.position)
        if not all(math.isfinite(value) for value in position):
            raise FlightError('激光里程计 %s 位置包含非有限值' % self.config['topics']['lio'])
        return position

    def elapsed_bad(self, name, bad, grace):
        """连续超限计时：bad 持续超过 grace 秒返回 True。"""
        now = time.monotonic()
        tracker = getattr(self, 'sustained', None)
        if tracker is None:
            tracker = self.sustained = {}
        if not bad:
            tracker.pop(name, None)
            return False, 0.0
        since = tracker.setdefault(name, now)
        return now - since >= grace, now - since

    def divergence_limit(self):
        """EKF/激光水平偏差上限；<=0 表示不启用。"""
        return max(float(self.safety.get('max_ekf_lio_divergence', 0.0)), 0.0)

    def check_estimator(self, position):
        """EKF 与激光里程计一致性。持续超限就停止下发，交还飞手。"""
        limit = self.divergence_limit()
        if limit <= 0.0:
            return
        lio_position = self.estimator_sample()
        divergence = math.dist(position[:2], lio_position[:2])
        triggered, duration = self.elapsed_bad('divergence', divergence > limit,
                                               self.safety.get('divergence_grace', 0.5))
        if triggered:
            raise FlightError('EKF 与激光里程计水平偏差 %.2fm 超过 %.2fm（位置估计不可信），'
                              '已持续 %.1fs；不再发送位置指令，请飞手立即接管' % (divergence, limit, duration))

    def check_fence(self, position):
        """距起飞原点的水平围栏；max_distance_from_origin<=0 时不启用。"""
        limit = self.safety.get('max_distance_from_origin', 0.0)
        if not limit or self.origin is None:
            return
        distance = math.dist(position[:2], self.origin[:2])
        if distance > limit:
            raise FlightError('水平距起飞原点 %.2fm 超过围栏 %.2fm；不再发送位置指令，请飞手立即接管'
                              % (distance, limit))

    def verify_estimator(self):
        """起飞前连续采样 EKF/激光一致性，持续超限就拒绝起飞。"""
        safety = self.safety
        limit = self.divergence_limit()
        if limit <= 0.0:
            return
        duration = safety.get('preflight_estimator_seconds', 0.0)
        started = time.monotonic()
        while time.monotonic() - started < duration:
            if self.ros.is_shutdown():
                raise FlightError('ROS 已关闭，起飞前估计器检查未完成')
            odom = self.get('odom', safety['telemetry_timeout'])
            position = vector(odom.pose.pose.position)
            lio_position = self.estimator_sample()
            divergence = math.dist(position[:2], lio_position[:2])
            if divergence > limit:
                raise FlightError('起飞前估计器不达标：EKF/激光偏差 %.2fm（阈值 %.2fm）；'
                                  '先排查建图与 EKF，不要强行起飞' % (divergence, limit))
            time.sleep(0.05)

    def landing_settings(self):
        """降落参数：发 LAND 前的落点参考判据 + 可选的低空进近。

        PX4Ctrl 收到 LAND 时把那一刻的实测位姿冻结成降落参考，因此 target_tolerance /
        target_speed 直接决定落点精度，比 safety/position_tolerance、arrival_speed 更紧；
        收不紧时最多等 target_timeout 秒，然后按当前位姿降落并告警。

        approach_height > 0 时，发 LAND 前先在位置控制下降落到离地该高度（x、y 仍指向终点），
        只缩短冻结参考被估计漂移影响的窗口；默认 0 表示不做进近。

        on_ground_height 是"实测离地高度 ≤ 该值即视为已在地面"的收尾判据：PX4 在本机没有外部
        定位融合时可能长期不报 landed_state=ON_GROUND，此时等 PX4Ctrl 交还飞手或用实测高度收尾，
        不再把这种情况当成"状态异常"中止。
        """
        settings = self.config.get('landing', {})
        values = dict(height=settings.get('approach_height', 0.0),
                      speed=settings.get('approach_speed', 0.15),
                      tolerance=settings.get('approach_tolerance', 0.02),
                      timeout=settings.get('approach_timeout', 20.0),
                      target_tolerance=settings.get('target_tolerance', 0.03),
                      target_speed=settings.get('target_speed', 0.05),
                      target_timeout=settings.get('target_timeout', 8.0),
                      on_ground_height=settings.get('on_ground_height', 0.20))
        for name, value in zip(('landing/approach_height', 'landing/approach_speed',
                                'landing/approach_tolerance', 'landing/approach_timeout',
                                'landing/target_tolerance', 'landing/target_speed',
                                'landing/target_timeout', 'landing/on_ground_height'), values.values()):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise FlightError(name + ' 必须是有限数值')
        if values['height'] < 0:
            raise FlightError('landing/approach_height 不能为负')
        if values['on_ground_height'] <= 0 or values['on_ground_height'] >= self.config['takeoff']['height']:
            raise FlightError('landing/on_ground_height 必须为正且小于 takeoff/height')
        if values['height'] >= self.config['takeoff']['height']:
            raise FlightError('landing/approach_height 必须小于 takeoff/height，否则进近没有意义')
        if min(values['speed'], values['tolerance'], values['timeout'],
               values['target_tolerance'], values['target_speed'], values['target_timeout']) <= 0:
            raise FlightError('landing 下的 approach_*、target_* 各项必须为正数')
        if values['height'] > 0 and values['speed'] > self.config['direct']['max_speed']:
            raise FlightError('landing/approach_speed 不能大于 direct/max_speed')
        return values

    def drop_settings(self):
        """危险品投放参数与校验。enabled=false 时以下所有投放逻辑都不参与任务流。"""
        settings = self.config.get('hazard_drop') or {}
        label = settings.get('label', '危险品')
        if not isinstance(label, str) or not label.strip():
            raise FlightError('hazard_drop/label 必须是非空字符串')
        points = settings.get('points', ['C', 'D'])
        if not isinstance(points, (list, tuple)) or not points:
            raise FlightError('hazard_drop/points 必须是非空航点列表')
        unknown = [str(point) for point in points if point not in ('C', 'D')]
        if unknown:
            raise FlightError('hazard_drop/points 只允许 C/D，当前含 %s' % '/'.join(unknown))
        values = dict(enabled=settings.get('enabled', False),
                      points=tuple(points),
                      label=label.strip(),
                      min_confidence=settings.get('min_confidence', 0.70),
                      vote_frames=settings.get('vote_frames', 15),
                      vote_ratio=settings.get('vote_ratio', 0.60),
                      hover_seconds=settings.get('hover_seconds', 3.0),
                      wait_timeout=settings.get('wait_timeout', 10.0),
                      height=settings.get('height', 0.15),
                      speed=settings.get('speed', 0.15),
                      tolerance=settings.get('tolerance', 0.02),
                      timeout=settings.get('timeout', 20.0),
                      push_wait=settings.get('push_wait', 2.5),
                      max_age=settings.get('max_age', 0.5),
                      once=settings.get('once', True),
                      service=settings.get('service', '/servo/push'))
        if not isinstance(values['enabled'], bool) or not isinstance(values['once'], bool):
            raise FlightError('hazard_drop/enabled 与 hazard_drop/once 必须是 true/false')
        if not isinstance(values['service'], str) or not values['service'].startswith('/'):
            raise FlightError('hazard_drop/service 必须是以 / 开头的服务名')
        for name in ('min_confidence', 'vote_frames', 'vote_ratio', 'hover_seconds', 'wait_timeout',
                     'height', 'speed', 'tolerance', 'timeout', 'push_wait', 'max_age'):
            value = values[name]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise FlightError('hazard_drop/%s 必须是有限数值' % name)
        if isinstance(values['vote_frames'], float) and not values['vote_frames'].is_integer():
            raise FlightError('hazard_drop/vote_frames 必须是整数')
        values['vote_frames'] = int(values['vote_frames'])
        if values['vote_frames'] < 1:
            raise FlightError('hazard_drop/vote_frames 必须 ≥ 1')
        if not 0.0 < values['min_confidence'] <= 1.0:
            raise FlightError('hazard_drop/min_confidence 必须位于 (0, 1]')
        if not 0.0 < values['vote_ratio'] <= 1.0:
            raise FlightError('hazard_drop/vote_ratio 必须位于 (0, 1]')
        if values['height'] <= 0 or values['height'] >= self.config['takeoff']['height']:
            raise FlightError('hazard_drop/height 必须大于 0 且小于 takeoff/height')
        if values['speed'] > self.config['direct']['max_speed']:
            raise FlightError('hazard_drop/speed 不能大于 direct/max_speed')
        for name in ('hover_seconds', 'wait_timeout', 'tolerance',
                     'timeout', 'push_wait', 'max_age'):
            if values[name] <= 0:
                raise FlightError('hazard_drop/%s 必须为正数' % name)
        if values['wait_timeout'] < values['hover_seconds']:
            raise FlightError('hazard_drop/wait_timeout 不能小于 hover_seconds，否则永远等不到投放')
        return values

    @staticmethod
    def box_center(detection):
        """检测框中心（像素）。优先用 YOLO 节点给的 cx/cy，否则由 x1..y2 求中点。"""
        try:
            return float(detection['cx']), float(detection['cy'])
        except (KeyError, TypeError, ValueError):
            pass
        try:
            return ((float(detection['x1']) + float(detection['x2'])) / 2.0,
                    (float(detection['y1']) + float(detection['y2'])) / 2.0)
        except (KeyError, TypeError, ValueError):
            return None

    def hazard_frame(self, message):
        """单帧判定：画面里可能有多个图案，只认离图像中心最近的那一个。

        无人机悬停在目标正上方，所以中心目标才是投放对象；边缘的相似框（别的
        危险品、误检）不参与判定。选中后还要看它的类别与置信度。坏消息/空帧
        按"未命中"参与投票。缺图像尺寸或框坐标时退化为"任一命中即算命中"。
        """
        settings = self.config.get('hazard_drop') or {}
        label = str(settings.get('label', '危险品')).strip()
        try:
            threshold = float(settings.get('min_confidence', 0.70))
        except (TypeError, ValueError):
            threshold = 0.70
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, AttributeError):
            return False
        if not isinstance(payload, dict):
            return False
        candidates = []
        for detection in payload.get('detections') or []:
            if not isinstance(detection, dict):
                continue
            try:
                text, confidence = str(detection['label']), float(detection['confidence'])
            except (KeyError, TypeError, ValueError):
                continue
            candidates.append((self.box_center(detection), text, confidence))
        if not candidates:
            return False
        try:
            width = float(payload.get('image_width') or 0.0)
            height = float(payload.get('image_height') or 0.0)
        except (TypeError, ValueError):
            width = height = 0.0
        located = [item for item in candidates if item[0] is not None]
        if located and width > 0.0 and height > 0.0:
            cx, cy = width / 2.0, height / 2.0
            _, text, confidence = min(
                located, key=lambda item: math.hypot(item[0][0] - cx, item[0][1] - cy))
            return label in text and confidence >= threshold
        return any(label in text and confidence >= threshold
                   for _, text, confidence in candidates)

    def vote_limit(self):
        """保留多少帧参与投票；坏配置按默认值处理，不让回调线程抛异常。"""
        settings = self.config.get('hazard_drop') or {}
        try:
            return max(1, int(settings.get('vote_frames', 15)))
        except (TypeError, ValueError):
            return 15

    def hazard_stable(self, now, values):
        """最近 vote_frames 帧里命中比例达到 vote_ratio，且最近一帧未过期。"""
        votes = getattr(self, 'hazard_votes', None) or ()
        last = getattr(self, 'hazard_last', None)
        if last is None or now - last > values['max_age'] or len(votes) < values['vote_frames']:
            return False
        hits = sum(1 for _, hit in votes if hit)
        return hits >= values['vote_ratio'] * len(votes)

    def drop_decision(self, now, label, settled_since):
        """到点悬停期间的投放判断：'wait' 继续等 / 'drop' 开始下降 / 'continue' 按原任务流走。"""
        values = self.drop_settings()
        done = getattr(self, 'drop_done', False)
        if not values['enabled'] or (values['once'] and done) or label not in values['points']:
            return 'continue'
        if now - settled_since < values['hover_seconds']:
            return 'wait'
        if self.hazard_stable(now, values):
            return 'drop'
        if now - settled_since >= values['wait_timeout']:
            votes = getattr(self, 'hazard_votes', None) or ()
            hits = sum(1 for _, hit in votes if hit)
            self.ros.logwarn('%s 点悬停 %.1fs 仍未确认危险品：最近 %d 帧命中 %d 帧'
                             '（需要 %d 帧里命中 ≥%.0f%%、置信度 ≥%.2f），不投放，继续任务',
                             label, now - settled_since, len(votes), hits,
                             values['vote_frames'], values['vote_ratio'] * 100.0,
                             values['min_confidence'])
            return 'continue'
        return 'wait'

    def start_drop(self, position, yaw, point, now):
        """从航点悬停高度平滑降到离地 height；x、y 仍指向该航点。"""
        values = self.drop_settings()
        limits = dict(self.config['direct'])
        limits['max_speed'] = min(limits['max_speed'], values['speed'])
        low = dict(xyz=[point['xyz'][0], point['xyz'][1], self.origin[2] + values['height']],
                   yaw_deg=math.degrees(yaw))
        duration = direct_duration(position, yaw, low, limits)
        self.status('DROP_DESCEND')
        self.ros.loginfo('已稳定检测到"%s"，下降到离地 %.2fm（world z=%.2f），参考时长 %.1fs；'
                         '到位后调用 %s 投放', values['label'], values['height'], low['xyz'][2],
                         duration, values['service'])
        return dict(started=now, start=tuple(position), yaw=yaw, target=low, duration=duration,
                    watchdog=MotionWatchdog(now, position), push_time=None, push_ok=None)

    def start_ascent(self, position, motion, point, now):
        """投放后爬回原航点高度，航向保持不变；到位后回到正常任务流。"""
        values = self.drop_settings()
        limits = dict(self.config['direct'])
        limits['max_speed'] = min(limits['max_speed'], values['speed'])
        goal = dict(xyz=list(point['xyz']), yaw_deg=math.degrees(motion['yaw']))
        duration = direct_duration(position, motion['yaw'], goal, limits)
        self.status('DROP_ASCEND')
        self.ros.loginfo('投放结束，爬回 world z=%.2f，参考时长 %.1fs', point['xyz'][2], duration)
        return dict(started=now, start=tuple(position), yaw=motion['yaw'], target=goal, duration=duration,
                    watchdog=MotionWatchdog(now, position), push_time=None,
                    push_ok=motion.get('push_ok'))

    def servo_push(self):
        """同步调用投放服务（默认 /servo/push，std_srvs/Trigger），返回 (成功, 说明)。

        投放失败（服务不存在、PX4 侧未就绪、指令被拒）只记录、不中止飞行：此时飞行本身
        仍然安全，硬中止反而把一次"没投下去"放大成空中险情。
        """
        values = self.drop_settings()
        try:
            from std_srvs.srv import Trigger
            self.ros.wait_for_service(values['service'], timeout=2.0)
            response = self.ros.ServiceProxy(values['service'], Trigger)()
        except Exception as error:
            return False, '调用失败：%s' % error
        return bool(response.success), response.message

    def begin_push(self):
        """在后台线程推舵机：服务调用最多等 2s，不能阻塞 50Hz 位置指令循环。

        返回结果字典；ok 为 None 表示线程还没回填结果（push_wait 期间通常已完成）。
        """
        values = self.drop_settings()
        outcome = dict(ok=None, message='等待服务响应')

        def call():
            ok, message = self.servo_push()
            outcome.update(ok=ok, message=message)
            if ok:
                self.ros.logwarn('舵机推出成功（%s）：%s', values['service'], message)
            else:
                self.ros.logerr('舵机推出失败（%s）：%s；载荷可能未投放，爬回后继续任务',
                                values['service'], message)

        threading.Thread(target=call, daemon=True, name='servo_push').start()
        return outcome

    def start_approach(self, position, yaw, goal):
        """从终点悬停高度平滑降到离地 approach_height，水平方向顺带收掉残留到点误差。"""
        values = self.landing_settings()
        limits = dict(self.config['direct'])
        limits['max_speed'] = min(limits['max_speed'], values['speed'])
        target = dict(xyz=[goal[0], goal[1], self.origin[2] + values['height']], yaw_deg=math.degrees(yaw))
        duration = direct_duration(position, yaw, target, limits)
        self.status('DESCEND')
        self.ros.loginfo('降落前进近：降到离地 %.2fm（world z=%.2f），参考时长 %.1fs；到位后交给 PX4Ctrl 降落',
                         values['height'], target['xyz'][2], duration)
        return dict(started=time.monotonic(), start=tuple(position), yaw=yaw, target=target,
                    duration=duration, watchdog=MotionWatchdog(time.monotonic(), position))

    def status(self, stage):
        self.stage = stage
        message = self.types['status']()
        message.data = json.dumps(dict(stage=stage, frame_id=self.config['frame_id'],
                                       origin_world=self.origin, configured_points=self.source_config['points'],
                                       points=self.config['points'], takeoff_hover=hover_point(self.config),
                                       config=self.config), ensure_ascii=False)
        self.publishers['status'].publish(message)
        self.ros.loginfo('任务阶段：%s', stage)

    def preflight(self):
        if self.ros.get_param('/use_sim_time', False):
            raise FlightError('拒绝在 /use_sim_time=true 下执行实机任务')
        controller = self.config['controller_node']
        planner = self.config['planner_node']
        for name, expected in ((controller + '/auto_takeoff_land/enable', True),
                               (controller + '/auto_takeoff_land/no_RC', False),
                               (controller + '/auto_takeoff_land/enable_auto_arm', True),
                               (self.config['traj_server_node'] + '/traj_server/hold_at_end', True),
                               (planner + '/fsm/flight_type', 1)):
            if self.ros.get_param(name, None) != expected:
                raise FlightError(name + ' 必须为 ' + str(expected))
        height = self.ros.get_param(controller + '/auto_takeoff_land/takeoff_height', None)
        if isinstance(height, bool) or not isinstance(height, (int, float)) or not math.isfinite(height) or abs(height - self.config['takeoff']['height']) > 0.01:
            raise FlightError('PX4Ctrl takeoff_height 与任务 takeoff/height 不一致；只读检查，不自动改控制器参数')
        self.graph()
        position, velocity, yaw = self.health(phase='ground')
        if self.get('extended', self.safety['fcu_timeout']).landed_state != 1:
            raise FlightError('起飞前 PX4 必须报告 ON_GROUND(1)')
        self.verify_estimator()
        resolved = resolve_config(self.source_config, position)
        limit = self.safety.get('max_distance_from_origin', 0.0)
        if limit:
            outside = sorted(label for label, point in resolved['points'].items()
                             if math.dist(point['xyz'][:2], position[:2]) > limit)
            if outside:
                raise FlightError('航点 %s 超出 safety/max_distance_from_origin=%.2fm 围栏'
                                  % ('/'.join(outside), limit))
        safety = resolved['safety']
        hover = hover_point(resolved)
        pass_radius = self.settings.get('pass_radius')
        if (isinstance(pass_radius, bool) or not isinstance(pass_radius, (int, float))
                or not math.isfinite(pass_radius) or pass_radius <= 0):
            raise FlightError('mission/pass_radius 必须为正数')
        for index, leg in enumerate(mission_legs()):
            if not leg.get('pass_through'):
                continue
            anchor = previous_anchor(leg['target'], resolved)
            if math.dist(anchor, resolved['points'][leg['target']]['xyz']) <= pass_radius:
                raise FlightError('mission/pass_radius 不小于 %s 到经过点 %s 的距离，%s 会被立即跳过；请检查航点与 pass_radius'
                                  % ('起飞悬停点' if index == 0 else '上一航点', leg['target'], leg['target']))
        self.landing_settings()
        drop = self.drop_settings()
        if drop['enabled']:
            # 服务缺失时宁可在地面报错，也不要飞完一圈才发现投放根本没发生。
            try:
                self.ros.wait_for_service(drop['service'], timeout=2.0)
            except Exception as error:
                raise FlightError('hazard_drop 已启用但投放服务 %s 不可用（%s）；先启动 px4_servo'
                                  % (drop['service'], error)) from error
            with self.lock:
                seen = self.data.get('detections')
            if seen is None:
                self.ros.logwarn('hazard_drop 已启用，但尚未收到检测话题 %s；确认 YOLO 节点已启动，'
                                 '否则到点后只会等满 %.1fs 并跳过投放',
                                 self.config['topics']['detections'], drop['wait_timeout'])
            self.ros.logwarn('危险品投放已启用：%s 中先连续稳定悬停 %.1fs，且最近 %d 帧里'
                             '命中中心最近目标的比例 ≥%.0f%%（单帧置信度 ≥ %.2f）后，'
                             '降到离地 %.2fm 调用 %s，再爬回该点高度继续任务',
                             '/'.join(drop['points']), drop['hover_seconds'], drop['vote_frames'],
                             drop['vote_ratio'] * 100.0, drop['min_confidence'], drop['height'],
                             drop['service'])
        predicted = (position[0], position[1], position[2] + height)
        if norm(velocity) > 0.1 or not arrived(predicted, velocity, yaw, hover, safety):
            raise FlightError('原地垂直起飞终点/当前航向不匹配 A 的起飞设置，或地面未静止；检查 A 和 takeoff/height')
        with self.lock:
            command = self.data.get('planner_cmd')
        if command and time.monotonic() - command[1] <= 0.5:
            raise FlightError('Planner 正在输出旧轨迹；请重启隔离 Planner 后再试')
        self.config, self.safety = resolved, safety
        self.origin = tuple(position)
        self.ros.loginfo('起飞原点 world=%s；实际 world 目标：hover=%s, B=%s(悬停), C=%s, D=%s, E=%s, F=%s(切换点), G=%s, H=%s(经过点), I=%s(终点)；轴方向与 yaw 仍按 world',
                         self.origin, hover['xyz'], resolved['points']['B']['xyz'], resolved['points']['C']['xyz'],
                         resolved['points']['D']['xyz'], resolved['points']['E']['xyz'], resolved['points']['F']['xyz'],
                         resolved['points']['G']['xyz'], resolved['points']['H']['xyz'], resolved['points']['I']['xyz'])
        return position, velocity, yaw

    def takeoff(self):
        if self.takeoff_sent or self.read_only:
            raise FlightError('禁止重复起飞或在只读检查中起飞')
        position, _, _ = self.preflight()
        self.active = True
        self.status('TAKEOFF')
        message = self.types['land']()
        message.takeoff_land_cmd = message.TAKEOFF
        self.takeoff_sent = True
        sent = time.monotonic()
        self.publishers['land'].publish(message)
        self.ros.logwarn('已发送一次 TAKEOFF，由 PX4Ctrl 切换 OFFBOARD、自动解锁并起飞；不会自动重试')
        return self.wait_takeoff(position, sent)

    def wait_takeoff(self, start, sent):
        limits = self.config['takeoff']
        stable_since = confirm_started = None
        seen_takeoff = confirmed = False
        while not self.ros.is_shutdown():
            position, velocity, yaw = self.health(phase='takeoff')
            now = time.monotonic()
            ctrl = self.get('ctrl', self.safety['telemetry_timeout'])
            fcu = self.get('fcu', self.safety['fcu_timeout'])
            extended = self.get('extended', self.safety['fcu_timeout'])
            if math.dist(position[:2], start[:2]) > limits['corridor_radius'] or position[2] < start[2] - limits['max_sink']:
                raise FlightError('起飞越过垂直通道/异常下沉，不发送规划目标，请飞手接管')
            with self.lock:
                command = self.data.get('planner_cmd')
            if command and now - command[1] <= 0.5:
                raise FlightError('起飞阶段出现旧 Planner 指令，拒绝放行')
            if ctrl.state == 4:
                seen_takeoff = True
            elif ctrl.state == 2 and not seen_takeoff:
                raise FlightError('未观察到 AUTO_TAKEOFF(4) 却进入悬停，拒绝继续任务')
            elif ctrl.state == 1 and (seen_takeoff or confirmed):
                raise FlightError('起飞过程中返回 MANUAL_CTRL，请飞手接管')
            if fcu.armed and fcu.mode == 'OFFBOARD' and ctrl.state in (4, 2):
                confirmed = True
            elif confirmed or ctrl.state == 2:
                raise FlightError('起飞过程中退出 armed+OFFBOARD，请飞手接管')
            else:
                if ctrl.state == 4:
                    confirm_started = now if confirm_started is None else confirm_started
                    if now - confirm_started >= limits['fcu_confirm_timeout']:
                        raise FlightError('起飞 armed+OFFBOARD 状态确认超时，不重发 TAKEOFF')
                if math.dist(position, start) > 0.1 or norm(velocity) > 0.1:
                    raise FlightError('未确认 armed+OFFBOARD 但机体已移动，请飞手接管')
            if not seen_takeoff and now - sent >= limits['ack_timeout']:
                raise FlightError('未确认 AUTO_TAKEOFF(4)，请查看 PX4Ctrl 拒绝原因；不重试起飞')
            if seen_takeoff and confirmed and ctrl.state == 2 and extended.landed_state == 2 and arrived(
                    position, velocity, yaw, hover_point(self.config), self.safety):
                if self.stage != 'TAKEOFF_HOVER':
                    self.status('TAKEOFF_HOVER')
                stable_since = now if stable_since is None else stable_since
                if now - stable_since >= limits['hover_seconds']:
                    self.ros.loginfo('程控起飞完成，A 上方连续稳定 %.1fs，允许发送 B 点目标', limits['hover_seconds'])
                    return position, velocity, yaw
            else:
                stable_since = None
            if now - sent >= limits['timeout']:
                raise FlightError('起飞/稳定悬停超时，未发送 B 点目标，请飞手接管')
            time.sleep(0.02)
        raise FlightError('ROS 已关闭，起飞请求不能由脚本退出撤销')

    def stop_planner(self):
        if not self.stopped:
            self.publishers['stop'].publish(self.types['stop']())
            self.stopped = True

    def send_goal(self, label='B'):
        point = self.config['points'][label]
        message = self.types['goal']()
        message.header.stamp = self.ros.Time.now()
        message.header.frame_id = self.config['frame_id']
        message.pose.position.x, message.pose.position.y, message.pose.position.z = point['xyz']
        yaw = math.radians(point['yaw_deg'])
        message.pose.orientation.z, message.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.publishers['goal'].publish(message)

    def start_planner(self, label):
        if self.stopped:
            raise FlightError('Planner 已被 mandatory stop 锁定，禁止在空中强行恢复')
        self.status('WAIT_PLANNER_' + label)
        with self.lock:
            entry = self.data.get('planner_cmd')
            previous_id = entry[0].trajectory_id if entry else 0
        sent = time.monotonic()
        stamp = self.ros.Time.now().to_sec()
        self.send_goal(label)
        return sent, previous_id, stamp

    def planner_ready(self, sent, previous_id, stamp):
        with self.lock:
            entry = self.data.get('planner_cmd')
        return (entry is not None and entry[1] >= sent and entry[0].trajectory_id > previous_id
                and entry[0].header.stamp.to_sec() >= stamp)

    def hold(self, position, yaw):
        return dict(position=position, velocity=(0, 0, 0), acceleration=(0, 0, 0), jerk=(0, 0, 0), yaw=yaw, yaw_dot=0)

    def command(self, sample):
        message = self.types['command']()
        message.trajectory_flag = message.TRAJECTORY_STATUS_READY
        message.trajectory_id = 1
        for key in ('position', 'velocity', 'acceleration', 'jerk'):
            getattr(message, key).x, getattr(message, key).y, getattr(message, key).z = sample[key]
        message.yaw, message.yaw_dot = sample['yaw'], sample['yaw_dot']
        return message

    def publish(self, message, position):
        values = [value for key in ('position', 'velocity', 'acceleration', 'jerk') for value in vector(getattr(message, key))]
        if not all(math.isfinite(value) for value in (*values, message.yaw, message.yaw_dot)):
            raise FlightError('控制指令包含非有限值', True)
        if math.dist(vector(message.position), position) > self.safety['max_tracking_error']:
            raise FlightError('位置跟踪误差超限', True)
        for key, limit in (('velocity', 'max_command_speed'), ('acceleration', 'max_command_acceleration'), ('jerk', 'max_command_jerk')):
            if norm(vector(getattr(message, key))) > self.safety[limit] + 1e-6:
                raise FlightError('控制指令超限：' + key, True)
        message.header.stamp = self.ros.Time.now()
        message.header.frame_id = self.config['frame_id']
        self.publishers['command'].publish(message)

    def request_land(self):
        if self.land_sent:
            return
        self.graph()
        self.health()
        self.stop_planner()
        self.status('LANDING')
        message = self.types['land']()
        message.takeoff_land_cmd = message.LAND
        self.publishers['land'].publish(message)
        self.land_sent = True
        self.ros.logwarn('已请求 PX4Ctrl 降落一次；不会强制解锁/停桨或重发降落')

    def measured_height(self):
        """离地高度（相对起飞原点，取 /ekf/ekf_odom）。

        降落收尾用：PX4Ctrl 自己也用这条里程计判断下沉，零点与本脚本记录的起飞原点一致。
        """
        if self.origin is None:
            raise FlightError('起飞原点未记录，无法判定离地高度')
        odom = self.get('odom', self.safety['telemetry_timeout'], True)
        height = vector(odom.pose.pose.position)[2] - self.origin[2]
        if not math.isfinite(height):
            raise FlightError('离地高度不是有限数值')
        return height

    def finish_landed(self, height, elapsed, reason):
        """落地收尾：不再下发指令，也不强制解锁/停桨，只提示飞手杀桨。"""
        self.status('DONE')
        self.active = False
        self.ros.logwarn('降落结束（%.1fs，%s，实测离地 %.2fm）：请杀桨（RC kill switch）收尾；'
                         '脚本不会强制解锁或停桨', elapsed, reason, height)

    def wait_landing(self):
        started = time.monotonic()
        acknowledged = False
        settings = self.landing_settings()
        while not self.ros.is_shutdown():
            elapsed = time.monotonic() - started
            fcu = self.get('fcu', self.safety['fcu_timeout'])
            extended = self.get('extended', self.safety['fcu_timeout'])
            if fcu.connected and not fcu.armed and extended.landed_state == 1:
                self.status('DONE')
                self.active = False
                return
            ctrl = self.get('ctrl', self.safety['telemetry_timeout'])
            if ctrl.state == 1:
                # PX4Ctrl 的落地判据成立后若 PX4 一直不报 ON_GROUND，它会把控制交还飞手；
                # 飞手空中接管也会走到这里，所以必须用实测高度区分。
                height = self.measured_height()
                if height <= settings['on_ground_height']:
                    self.finish_landed(height, elapsed, 'PX4Ctrl 已交还飞手')
                    return
                raise FlightError('PX4Ctrl 交还飞手（MANUAL_CTRL），实测离地 %.2fm > %.2fm，'
                                  '未确认落地，请立即接管' % (height, settings['on_ground_height']))
            self.health(landing=True)
            acknowledged = acknowledged or ctrl.state == 5
            if not acknowledged and elapsed > self.settings['landing_ack_timeout']:
                raise FlightError('PX4Ctrl 未确认 AUTO_LAND，请立即接管')
            if elapsed > self.settings['landing_timeout']:
                height = self.measured_height()
                if height <= settings['on_ground_height']:
                    self.finish_landed(height, elapsed, '降落超时但实测已在地面')
                    return
                raise FlightError('降落超时（已 %.1fs，实测离地 %.2fm），未确认落地，请立即接管'
                                  % (elapsed, height))
            time.sleep(0.02)
        raise FlightError('ROS 关闭，未确认降落完成')

    def step_yaw(self, commanded_yaw, target, interval):
        error = wrap(math.radians(target['yaw_deg']) - commanded_yaw)
        limit = math.radians(self.config['direct']['max_yaw_rate_deg']) * interval
        change = max(-limit, min(limit, error))
        return wrap(commanded_yaw + change), change

    def bridge_command(self, bridge, commanded_yaw, target, interval, now):
        """经过点切换期间继续跟踪上一段的指令源，避免在切换点急停。

        B 之后的下一段是 Planner 段：继续转发 Planner 仍在发布的旧轨迹。
        H 之后切换前是直控段：继续跟踪上一段直控参考轨迹（到终点后自然变成定点悬停）。
        """
        if bridge['kind'] == 'direct':
            sample = direct_sample(bridge['start'], bridge['yaw'], bridge['target'], bridge['duration'],
                                   now - bridge['started'])
            return self.command(sample), sample['yaw']
        return self.planner_bridge_command(commanded_yaw, target, interval)

    def planner_bridge_command(self, commanded_yaw, target, interval):
        """继续转发上一段 Planner 轨迹，直到新目标的轨迹到达。"""
        with self.lock:
            entry = self.data.get('planner_cmd')
        if entry is None or time.monotonic() - entry[1] > self.safety['planner_command_timeout']:
            raise FlightError('经过点切换期间没有可继续执行的 Planner 指令', True)
        message = copy.deepcopy(entry[0])
        if message.header.frame_id != self.config['frame_id'] or message.trajectory_flag != message.TRAJECTORY_STATUS_READY:
            raise FlightError('Planner 指令坐标系/状态无效', True)
        commanded_yaw, change = self.step_yaw(commanded_yaw, target, interval)
        message.yaw, message.yaw_dot = commanded_yaw, change / max(interval, 1e-6)
        return message, commanded_yaw

    def begin_leg(self, leg, now, position, commanded_yaw, bridge=None):
        if leg['mode'] == 'planner':
            started, previous_id, goal_stamp = self.start_planner(leg['target'])
            if bridge is None and leg.get('bridge'):
                bridge = dict(kind='planner')
            return dict(started=started, previous_id=previous_id, goal_stamp=goal_stamp,
                        direct_start=None, direct_yaw=0.0, duration=None, bridge=bridge,
                        watchdog=MotionWatchdog(now, position))
        self.status('DIRECT')
        duration = direct_duration(position, commanded_yaw, self.config['points'][leg['target']], self.config['direct'])
        if duration + self.settings['arrival_dwell'] >= self.settings['direct_timeout']:
            raise FlightError('直控到 %s 所需时长超过 direct_timeout' % leg['target'], True)
        self.ros.loginfo('已关闭 Planner 指令通路（不发送 mandatory stop）；直控到 %s 参考轨迹时长 %.2fs',
                         leg['target'], duration)
        return dict(started=now, previous_id=0, goal_stamp=0.0,
                    direct_start=position, direct_yaw=commanded_yaw, duration=duration, bridge=None,
                    watchdog=MotionWatchdog(now, position))

    def next_leg(self, legs, index, now, position, commanded_yaw):
        """进入下一航段；返回 run() 循环里需要同步更新的状态。"""
        index += 1
        motion = self.begin_leg(legs[index], now, position, commanded_yaw)
        return index, motion, position, None, index == len(legs) - 1

    def run(self):
        position, _, yaw = self.takeoff()
        self.active = True
        # H 是经过点：进入 pass_radius 就发下一段，不悬停。B 现在会悬停 arrival_dwell 秒。
        legs = mission_legs()
        if legs[-1].get('pass_through'):
            raise FlightError('最后一个航点不能设置为经过点', True)
        index = 0
        hold_position = position
        commanded_yaw = yaw
        settled_since = None
        lock_logged = False
        now = time.monotonic()
        motion = self.begin_leg(legs[0], now, position, commanded_yaw)
        last_tick = now
        while not self.ros.is_shutdown():
            position, velocity, yaw = self.health()
            now = time.monotonic()
            interval = now - last_tick
            if interval > self.safety['planner_command_timeout']:
                raise FlightError('任务发布循环卡顿，停止继续跟踪', True)
            last_tick = now
            leg = legs[index]
            last_leg = index == len(legs) - 1
            target = self.config['points'][leg['target']]
            waiting_stage = 'WAIT_PLANNER_' + leg['target']
            planning_stage = 'PLANNER_' + leg['target']
            if self.stage == waiting_stage:
                if motion.get('bridge'):
                    message, commanded_yaw = self.bridge_command(motion['bridge'], commanded_yaw, target,
                                                                 interval, now)
                else:
                    message = self.command(self.hold(hold_position, commanded_yaw))
                if self.planner_ready(motion['started'], motion['previous_id'], motion['goal_stamp']):
                    self.status(planning_stage)
                elif now - motion['started'] > self.settings['planner_first_command_timeout']:
                    raise FlightError('发送 %s 点后未收到新轨迹 ID 的 Planner 指令；旧轨迹不放行' % leg['target'], True)
            if self.stage == planning_stage:
                deadline = self.settings['planner_timeout']
                if motion.get('drop_waiting'):
                    # 已经到点、正在等稳定检测：到点悬停不计入规划超时，否则长航段会刚好撞线。
                    deadline += self.drop_settings()['wait_timeout']
                if now - motion['started'] > deadline:
                    raise FlightError('Planner 段到达 %s 超时（可能被改终点/阻挡）' % leg['target'], True)
                try:
                    message = copy.deepcopy(self.get('planner_cmd', self.safety['planner_command_timeout'], True))
                except FlightError as error:
                    raise FlightError(str(error), True) from error
                if message.header.frame_id != self.config['frame_id'] or message.trajectory_flag != message.TRAJECTORY_STATUS_READY:
                    raise FlightError('Planner 指令坐标系/状态无效', True)
                if message.trajectory_id <= motion['previous_id'] or message.header.stamp.to_sec() < motion['goal_stamp']:
                    raise FlightError('Planner 指令退回切换前的旧轨迹', True)
                commanded_yaw, change = self.step_yaw(commanded_yaw, target, interval)
                message.yaw, message.yaw_dot = commanded_yaw, change / max(interval, 1e-6)
                if leg.get('pass_through'):
                    distance = math.dist(position, target['xyz'])
                    if distance <= self.settings['pass_radius']:
                        self.ros.loginfo('%s 点作为经过点（距中心 %.2fm），立即发送 %s 目标，不悬停',
                                         leg['target'], distance, legs[index + 1]['target'])
                        index += 1
                        hold_position = position
                        motion = self.begin_leg(legs[index], now, position, commanded_yaw)
                        settled_since = None
                        last_leg = index == len(legs) - 1
                else:
                    endpoint_hold = (math.dist(vector(message.position), target['xyz']) <= self.safety['position_tolerance']
                                     and all(norm(vector(getattr(message, key))) <= 1e-6
                                             for key in ('velocity', 'acceleration', 'jerk')))
                    if endpoint_hold and arrived(position, velocity, yaw, target, self.safety):
                        if settled_since is None:
                            settled_since = now
                            self.ros.loginfo('%s 点已到达，开始连续稳定悬停计时', leg['target'])
                    else:
                        settled_since = None
                    if settled_since is not None and now - settled_since >= self.settings['arrival_dwell']:
                        if last_leg:
                            hold_position = position
                            self.status('HOLD')
                            motion = dict(started=now, watchdog=MotionWatchdog(now, position))
                            settled_since = None
                            self.ros.loginfo('%s 点已连续稳定悬停 %.1fs，进入最终悬停后降落',
                                             leg['target'], self.settings['arrival_dwell'])
                        else:
                            action = self.drop_decision(now, leg['target'], settled_since)
                            if action == 'drop':
                                motion = self.start_drop(position, commanded_yaw, target, now)
                                settled_since = None
                            elif action == 'wait':
                                motion['drop_waiting'] = True
                            else:
                                self.ros.loginfo('%s 点已连续稳定悬停 %.1fs，切换控制源飞往 %s',
                                                 leg['target'], self.settings['arrival_dwell'], legs[index + 1]['target'])
                                index, motion, hold_position, settled_since, last_leg = self.next_leg(
                                    legs, index, now, position, commanded_yaw)
            # 位移看门狗只在"规划器确实在指挥飞机运动"时才有意义。规划器紧急停止、目标点被地图
            # 占据判为不可达、或轨迹已到头时，它会给一条停在原地的参考轨迹并保持；这时报
            # "目标发出后 3.0s 内无有效位移"会把规划器的问题说成飞机的问题（2026-10-31 两次：
            # B→C 被速度检查连拒后急停；F 点距实测障碍仅 0.13m，Planner 停在 0.18m 外不再给参考）。
            # 判据看**指令位置**而不是指令速度：紧急停止/到头的轨迹速度本来就是 0，
            # 而"参考在推进、飞机没跟上"才是真的跟踪故障，那个人仍交给 motion_timeout 看门狗。
            # WAIT_PLANNER_* 阶段不设看门狗：planner_first_command_timeout 已覆盖"没收到新轨迹"。
            if self.stage == planning_stage:
                dist_to_target = math.dist(position, target['xyz'])
                command_position = vector(message.position) if message is not None else position
                anchor = motion.get('command_anchor')
                if dist_to_target <= self.safety['position_tolerance']:
                    # 已经到达目标：参考停在目标点是正常的，不判失速
                    motion['command_anchor'], motion['command_time'] = command_position, now
                elif anchor is None or math.dist(command_position, anchor) >= self.safety['motion_distance']:
                    motion['command_anchor'], motion['command_time'] = command_position, now
                elif now - motion.get('command_time', now) >= self.settings.get('stalled_command_timeout', 2.0):
                    raise FlightError('Planner 只给出静止指令，未继续飞向 %s：参考停在距目标 %.2fm 处'
                                      '（实测距目标 %.2fm）；多为目标点被占据/不可达或规划器已紧急停止'
                                      % (leg['target'], math.dist(command_position, target['xyz']),
                                         dist_to_target), True)
                motion['watchdog'].check(now, position, target['xyz'], self.safety)
            if self.stage == 'DIRECT':
                target = self.config['points'][leg['target']]
                elapsed = now - motion['started']
                message = self.command(direct_sample(motion['direct_start'], motion['direct_yaw'], target,
                                                     motion['duration'], elapsed))
                if elapsed > self.settings['direct_timeout']:
                    raise FlightError('直控到达 %s 超时' % leg['target'], True)
                motion['watchdog'].check(now, position, target['xyz'], self.safety)
                if leg.get('pass_through') and math.dist(position, target['xyz']) <= self.settings['pass_radius']:
                    # 经过点：立刻发下一段目标；等待期间继续跟踪本段直控参考，不在经过点急停。
                    self.ros.loginfo('%s 点作为经过点（距中心 %.2fm），立即发送 %s 目标，不悬停',
                                     leg['target'], math.dist(position, target['xyz']), legs[index + 1]['target'])
                    bridge = dict(kind='direct', start=motion['direct_start'], yaw=motion['direct_yaw'],
                                  target=target, duration=motion['duration'], started=motion['started'])
                    index += 1
                    hold_position = position
                    motion = self.begin_leg(legs[index], now, position, commanded_yaw, bridge)
                    settled_since = None
                    last_leg = index == len(legs) - 1
                elif elapsed >= motion['duration'] and arrived(position, velocity, yaw, target, self.safety):
                    hold_position, commanded_yaw = position, message.yaw
                    self.status('HOLD' if last_leg else 'DWELL')
                    motion = dict(started=now, watchdog=MotionWatchdog(now, position))
                    settled_since = None
                    self.ros.loginfo('%s 点直控到达，进入%s', leg['target'],
                                     '最终悬停计时' if last_leg else '连续稳定悬停计时')
                else:
                    settled_since = None
            if self.stage == 'DWELL':
                target = self.config['points'][leg['target']]
                message = self.command(self.hold(target['xyz'], math.radians(target['yaw_deg'])))
                if arrived(position, velocity, yaw, target, self.safety):
                    settled_since = settled_since if settled_since is not None else now
                    if now - settled_since >= self.settings['arrival_dwell']:
                        action = self.drop_decision(now, leg['target'], settled_since)
                        if action == 'drop':
                            motion = self.start_drop(position, commanded_yaw, target, now)
                            settled_since = None
                        elif action == 'continue':
                            self.ros.loginfo('%s 点已连续稳定悬停 %.1fs，切换控制源飞往 %s',
                                             leg['target'], self.settings['arrival_dwell'], legs[index + 1]['target'])
                            index, motion, hold_position, settled_since, last_leg = self.next_leg(
                                legs, index, now, position, commanded_yaw)
                else:
                    settled_since = None
                if now - motion['started'] > self.settings['arrival_dwell'] + self.settings['direct_timeout']:
                    raise FlightError('%s 点无法连续稳定悬停' % leg['target'], True)
            if self.stage == 'HOLD':
                target = self.config['points'][legs[-1]['target']]
                message = self.command(self.hold(target['xyz'], math.radians(target['yaw_deg'])))
                if arrived(position, velocity, yaw, target, self.safety):
                    settled_since = settled_since if settled_since is not None else now
                    if now - settled_since >= self.settings['hover_seconds']:
                        landing = self.landing_settings()
                        error, speed = math.dist(position, target['xyz']), norm(velocity)
                        converged = error <= landing['target_tolerance'] and speed <= landing['target_speed']
                        waited = now - settled_since - self.settings['hover_seconds']
                        if not converged and waited < landing['target_timeout']:
                            if not lock_logged:
                                lock_logged = True
                                self.ros.loginfo('%s 点已稳定悬停，等待落点参考收敛：当前误差 %.3fm、速度 %.3fm/s，'
                                                 '阈值 %.2fm/%.2fm，最多再等 %.1fs',
                                                 legs[-1]['target'], error, speed, landing['target_tolerance'],
                                                 landing['target_speed'], landing['target_timeout'])
                        else:
                            if converged:
                                self.ros.loginfo('%s 点落点参考已收敛：误差 %.3fm、速度 %.3fm/s（阈值 %.2fm/%.2fm）；'
                                                 'PX4Ctrl 以此刻位姿为落点参考',
                                                 legs[-1]['target'], error, speed, landing['target_tolerance'],
                                                 landing['target_speed'])
                            else:
                                self.ros.logwarn('%s 点 %.1fs 内落点参考未收敛（误差 %.3fm、速度 %.3fm/s，'
                                                 '阈值 %.2fm/%.2fm），按当前位姿请求降落',
                                                 legs[-1]['target'], waited, error, speed,
                                                 landing['target_tolerance'], landing['target_speed'])
                            if landing['height'] > 0 and landing['height'] < target['xyz'][2] - self.origin[2]:
                                self.ros.loginfo('%s 点先进近到离地 %.2fm 再请求降落', legs[-1]['target'],
                                                 landing['height'])
                                motion = self.start_approach(position, math.radians(target['yaw_deg']), target['xyz'])
                                settled_since = None
                                lock_logged = False
                            else:
                                self.request_land()
                                self.wait_landing()
                                return
                else:
                    settled_since = None
                    lock_logged = False
                if now - motion['started'] > self.settings['hover_seconds'] + self.settings['direct_timeout']:
                    raise FlightError('%s 点无法连续稳定悬停' % legs[-1]['target'], True)
            if self.stage == 'DESCEND':
                landing = self.landing_settings()
                target_z = motion['target']['xyz'][2]
                elapsed = now - motion['started']
                message = self.command(direct_sample(motion['start'], motion['yaw'], motion['target'],
                                                     motion['duration'], elapsed))
                commanded_yaw = message.yaw
                motion['watchdog'].check(now, position, motion['target']['xyz'], self.safety)
                if position[2] <= target_z + landing['tolerance']:
                    # 到达进近高度（或地面比预期高、已经贴近地面）：交给 PX4Ctrl 完成最后一段。
                    self.ros.loginfo('已降到离地 %.2fm（world z=%.2f，目标 %.2f），落点水平误差 %.3fm；请求 PX4Ctrl 降落',
                                     position[2] - self.origin[2], position[2], target_z,
                                     math.dist(position[:2], motion['target']['xyz'][:2]))
                    self.request_land()
                    self.wait_landing()
                    return
                if elapsed > landing['timeout']:
                    self.ros.logwarn('进近 %.1fs 仍未降到 %.2fm（当前 %.2fm），直接请求 PX4Ctrl 降落',
                                     elapsed, target_z, position[2])
                    self.request_land()
                    self.wait_landing()
                    return
            if self.stage == 'DROP_DESCEND':
                values = self.drop_settings()
                point = self.config['points'][leg['target']]
                elapsed = now - motion['started']
                message = self.command(direct_sample(motion['start'], motion['yaw'], motion['target'],
                                                     motion['duration'], elapsed))
                commanded_yaw = message.yaw
                motion['watchdog'].check(now, position, motion['target']['xyz'], self.safety)
                if position[2] <= motion['target']['xyz'][2] + values['tolerance']:
                    # 到达投放高度（或地面比预计高、已经贴近地面）：立刻推舵机，
                    # 再保持低空 push_wait 秒让载荷脱离（与舵机 hold_s 无关，hold_s=0 会一直保持）。
                    motion['push_time'] = now
                    motion['push_ok'] = self.begin_push()
                    self.status('DROP_PUSH')
                    self.ros.loginfo('已降到离地 %.2fm（world z=%.2f），水平误差 %.3fm；保持 %.1fs 后爬回 %s',
                                     position[2] - self.origin[2], position[2],
                                     math.dist(position[:2], motion['target']['xyz'][:2]),
                                     values['push_wait'], point['xyz'][2])
                elif elapsed > values['timeout']:
                    self.ros.logwarn('下降 %.1fs 仍未到离地 %.2fm（当前 %.2fm），放弃本次投放并爬回；'
                                     '若 %s 也在 points 里会重新尝试',
                                     elapsed, values['height'], position[2] - self.origin[2], leg['target'])
                    motion = self.start_ascent(position, motion, point, now)
            if self.stage == 'DROP_PUSH':
                values = self.drop_settings()
                point = self.config['points'][leg['target']]
                message = self.command(self.hold(motion['target']['xyz'], motion['yaw']))
                if now - motion['push_time'] >= values['push_wait']:
                    if values['once']:
                        self.drop_done = True
                    motion = self.start_ascent(position, motion, point, now)
            if self.stage == 'DROP_ASCEND':
                values = self.drop_settings()
                elapsed = now - motion['started']
                message = self.command(direct_sample(motion['start'], motion['yaw'], motion['target'],
                                                     motion['duration'], elapsed))
                commanded_yaw = message.yaw
                motion['watchdog'].check(now, position, motion['target']['xyz'], self.safety)
                if elapsed >= motion['duration'] and arrived(position, velocity, yaw, target, self.safety):
                    outcome = (motion.get('push_ok') or {}).get('ok')
                    self.ros.loginfo('%s 点投放流程结束（舵机推出%s），已爬回离地 %.2fm，继续飞往 %s',
                                     leg['target'], {True: '成功', False: '失败'}.get(outcome, '未确认'),
                                     position[2] - self.origin[2], legs[index + 1]['target'])
                    index, motion, hold_position, settled_since, last_leg = self.next_leg(
                        legs, index, now, position, commanded_yaw)
                elif elapsed > values['timeout']:
                    # 投完放不回去比不投放更危险：按保护降落处理，不让飞机卡在低空。
                    raise FlightError('投放后爬回 %s 超时（当前离地 %.2fm）' %
                                      (leg['target'], position[2] - self.origin[2]), True)
            self.publish(message, position)
            time.sleep(max(0, 1 / self.settings['publish_hz'] - (time.monotonic() - now)))
        raise FlightError('ROS 已关闭；退出不等于飞机已停止')

    def abort(self, error):
        self.ros.logerr('任务停止：%s', error)
        if self.takeoff_sent:
            self.ros.logerr('TAKEOFF 已发出：退出不会撤销起飞、自动降落或停桨；请立即判断飞行状态')
        if self.active and not self.land_sent and getattr(error, 'land_allowed', False):
            try:
                self.request_land()
                self.wait_landing()
                return
            except Exception as landing_error:
                self.ros.logerr('保护降落未完成：%s', landing_error)
        self.ros.logerr('不再发送位置指令；不会强行切模式或停桨。请飞手立即接管！')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[1] / 'config/hybrid_mission.yaml')
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--check', action='store_true', help='地面未解锁时只读检查程控起飞条件，不发飞行指令')
    action.add_argument('--execute', action='store_true', help='实机执行，默认仅离线显示配置')
    parser.add_argument('--map-verified', action='store_true', help='人工确认 A→B、B→C、D→E、E→F、H→I 地图正确')
    parser.add_argument('--direct-path-verified', action='store_true',
                        help='人工确认 C→D、F→G、G→H 直控路径和 I 下方降落区无障碍')
    parser.add_argument('--hazard-drop', dest='hazard_drop', action='store_true', default=None,
                        help='启用危险品投放（覆盖 hazard_drop/enabled；默认跟随配置）')
    parser.add_argument('--no-hazard-drop', dest='hazard_drop', action='store_false', default=None,
                        help='禁用危险品投放（覆盖 hazard_drop/enabled）')
    args = parser.parse_args(argv)
    try:
        with args.config.expanduser().open(encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        if args.hazard_drop is not None:
            config.setdefault('hazard_drop', {})['enabled'] = args.hazard_drop
        for label in ('A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'):
            point = config['points'][label]
            xyz = point['xyz']
            yaw = point.get('yaw_deg')
            finite = lambda value: isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
            if len(xyz) != 3 or not all(finite(value) for value in xyz):
                raise ValueError('points/%s/xyz 必须是三个有限数值' % label)
            if not finite(yaw):
                raise ValueError('points/%s/yaw_deg 必须是有限数值' % label)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        parser.error('参数无效：' + str(error))
    pending = unmeasured_points(config)
    if config.get('coordinate_mode', 'world') == 'takeoff_relative':
        print('A=[0,0,0] 为确认后锁定的地面起飞原点；B/C/D/E/F/G/H/I 相对该原点，轴方向平行 %s。' % config['frame_id'])
        print('起飞悬停位置相对 A=%s；脚本仅将坐标平移到 world，不重置 EKF/TF。' % hover_point(config)['xyz'])
    else:
        print('坐标为 %s 下绝对米制坐标，A 为起飞后的悬停点。' % config['frame_id'])
    print('yaw_deg 仍为 world 绕 +Z 的绝对航向角，不是相对起飞机头方向。')
    print('任务脚本默认不限制坐标范围、不预检 Planner 虚拟边界；仍检查有限数值、定位和指令健康。'
          'safety/max_distance_from_origin>0 时按距起飞原点的水平半径围栏中止。Planner 原生地图/避障限制不变。')
    print(json.dumps(config['points'], ensure_ascii=False, indent=2))
    print('程控原地起飞 %.2fm → 起飞悬停点稳定 %.1fs → Planner 到 B 并悬停 %.1fs → Planner 到 C 并悬停 %.1fs'
          ' → 纯 PX4Ctrl 到 D 并悬停 %.1fs → Planner 到 E 并悬停 %.1fs → Planner 到 F 并悬停 %.1fs'
          ' → 纯 PX4Ctrl 到 G 并悬停 %.1fs → 纯 PX4Ctrl 到 H（经过点，不悬停）'
          ' → Planner 到 I（终点）并悬停 %.1fs → 降到离地 %.2fm → 降落。' %
          (config['takeoff']['height'], config['takeoff']['hover_seconds'],
           config['mission']['arrival_dwell'], config['mission']['arrival_dwell'],
           config['mission']['arrival_dwell'], config['mission']['arrival_dwell'],
           config['mission']['arrival_dwell'], config['mission']['arrival_dwell'],
           config['mission']['hover_seconds'],
           config.get('landing', {}).get('approach_height', 0.0)))
    drop = config.get('hazard_drop') or {}
    if drop.get('enabled', False):
        print('危险品投放已启用：%s 点连续稳定悬停 %.1fs，且最近 %d 帧里命中"离图像中心最近的'
              '目标"的比例 ≥%.0f%%（该目标为"%s"且置信度≥%.2f）后，降到离地 %.2fm 调用 %s，'
              '保持 %.1fs 后爬回该点高度继续任务；到点后最多等 %.1fs，超时只告警并继续。'
              % ('/'.join(drop.get('points', ['C', 'D'])), drop.get('hover_seconds', 3.0),
                 drop.get('vote_frames', 15), drop.get('vote_ratio', 0.60) * 100.0,
                 drop.get('label', '危险品'), drop.get('min_confidence', 0.70),
                 drop.get('height', 0.15), drop.get('service', '/servo/push'),
                 drop.get('push_wait', 2.5), drop.get('wait_timeout', 10.0)))
    else:
        print('危险品投放未启用（hazard_drop/enabled=false）：不订阅检测话题、不调用舵机。')
    if pending:
        print('注意：%s 点标记为 measured: false（尚未实测）；可预览/--check，但 --execute 会被拒绝。' % '/'.join(pending))
    if not args.execute and not args.check:
        print('离线预览完成：未连接 ROS、未发布指令。实机检查用 --check，执行用 --execute。')
        return 0
    if args.execute and (not args.map_verified or not args.direct_path_verified):
        parser.error('实机执行必须同时指定 --map-verified --direct-path-verified')
    if args.execute and pending:
        parser.error('航点 %s 尚未实测（measured: false）；先更新 config/hybrid_mission.yaml 再执行' % '/'.join(pending))
    import rospy

    rospy.init_node('hybrid_mission', anonymous=True, disable_signals=True)
    mission = None
    try:
        mission = Mission(rospy, config, read_only=not args.execute)
        deadline = time.monotonic() + config['mission']['startup_timeout']
        while True:
            try:
                mission.preflight()
                break
            except FlightError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.1)
        if not args.execute:
            rospy.loginfo('只读检查通过：不解锁、不起飞、不发目标/位置指令/降落请求')
            return 0
        print('警告：确认后将自动切 OFFBOARD、解锁并起飞！起飞垂直通道和 C→D、F→G、G→H 直控段不做 Planner 避障。定位断更时不会盲目降落；飞手必须可立即接管。')
        if input('确认 A/B/C/D/E/F/G/H/I、坐标系、航向和 I 降落区，输入 MISSION：').strip() != 'MISSION':
            print('已取消，未发送飞行指令。')
            return 0
        mission.run()
        return 0
    except KeyboardInterrupt:
        if mission is not None:
            mission.abort(FlightError('用户中断；停止任务流不等于安全降落'))
        return 130
    except Exception as error:
        if mission is not None:
            mission.abort(error)
        else:
            rospy.logerr('任务未启动：%s', error)
        return 1
    finally:
        rospy.signal_shutdown('mission process finished')


if __name__ == '__main__':
    raise SystemExit(main())
