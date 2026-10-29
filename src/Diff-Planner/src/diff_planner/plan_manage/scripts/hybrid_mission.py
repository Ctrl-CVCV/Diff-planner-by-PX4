#!/usr/bin/env python3
"""Automatic takeoff; planner through B to C; direct PX4Ctrl to D; planner to E and F;
direct PX4Ctrl to G and H; planner back to I; hover, then land.

Planner legs: A->B (B is a pass-through), B->C, D->E, E->F, H->I.
Direct PX4Ctrl legs: C->D, F->G, G->H. C, D, E, F, G and H dwell mission/arrival_dwell.

B is a pass-through waypoint: the next goal is sent as soon as the vehicle enters
mission/pass_radius around it, without a dwell. Offline by default.
"""

import argparse
import copy
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
        self.subscribers = [ros.Subscriber(config['topics'][key], message_type, self.receive,
                                           callback_args=key, queue_size=1, tcp_nodelay=True)
                            for key, message_type in message_types.items()]

    def receive(self, message, key):
        with self.lock:
            self.data[key] = (message, time.monotonic())

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
        """读取 /Odometry，返回 (world 位置, 时间戳延迟秒)；只校验数值有效性，不判阈值。"""
        safety, lio = self.safety, self.get('lio', self.safety['telemetry_timeout'])
        stamp = lio.header.stamp.to_sec()
        if not math.isfinite(stamp) or stamp <= 0:
            raise FlightError('激光里程计 %s 时间戳无效' % self.config['topics']['lio'])
        age = self.ros.Time.now().to_sec() - stamp
        if age < -safety['max_future_stamp']:
            raise FlightError('激光里程计 %s 时间戳超前 %.3fs：时间同步异常' % (self.config['topics']['lio'], -age))
        position = vector(lio.pose.pose.position)
        if not all(math.isfinite(value) for value in position):
            raise FlightError('激光里程计 %s 位置包含非有限值' % self.config['topics']['lio'])
        return position, age

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

    def estimator_limits(self):
        """(激光里程计延迟上限, EKF/激光偏差上限)；任一 <=0 表示该项不启用。"""
        return (max(float(self.safety.get('max_lio_age', 0.0)), 0.0),
                max(float(self.safety.get('max_ekf_lio_divergence', 0.0)), 0.0))

    def check_estimator(self, position):
        """里程计延迟与 EKF/激光一致性。任一项持续超限就停止下发，交还飞手。"""
        safety = self.safety
        age_limit, divergence_limit = self.estimator_limits()
        if age_limit <= 0.0 and divergence_limit <= 0.0:
            return
        lio_position, age = self.estimator_sample()
        divergence = math.dist(position[:2], lio_position[:2])
        for name, bad, grace, detail in (
                ('lio_age', age_limit > 0.0 and age > age_limit, safety.get('lio_grace', 1.0),
                 '激光里程计延迟 %.2fs 超过 %.2fs（建图跟不上，EKF 时间同步会失效）' % (age, age_limit)),
                ('divergence', divergence_limit > 0.0 and divergence > divergence_limit, safety.get('divergence_grace', 0.5),
                 'EKF 与激光里程计水平偏差 %.2fm 超过 %.2fm（位置估计不可信）'
                 % (divergence, divergence_limit))):
            triggered, duration = self.elapsed_bad(name, bad, grace)
            if triggered:
                raise FlightError(detail + '，已持续 %.1fs；不再发送位置指令，请飞手立即接管' % duration)

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
        """起飞前连续采样：延迟和偏差都必须持续达标，否则拒绝起飞。"""
        safety = self.safety
        age_limit, divergence_limit = self.estimator_limits()
        if age_limit <= 0.0 and divergence_limit <= 0.0:
            return
        duration = safety.get('preflight_estimator_seconds', 0.0)
        started = time.monotonic()
        while time.monotonic() - started < duration:
            if self.ros.is_shutdown():
                raise FlightError('ROS 已关闭，起飞前估计器检查未完成')
            odom = self.get('odom', safety['telemetry_timeout'])
            position = vector(odom.pose.pose.position)
            lio_position, age = self.estimator_sample()
            divergence = math.dist(position[:2], lio_position[:2])
            if ((age_limit > 0.0 and age > age_limit)
                    or (divergence_limit > 0.0 and divergence > divergence_limit)):
                raise FlightError('起飞前估计器不达标：激光里程计延迟 %.2fs（阈值 %.2fs）、EKF/激光偏差 %.2fm'
                                  '（阈值 %.2fm）；先排查建图延迟与时间同步'
                                  % (age, age_limit, divergence, divergence_limit))
            time.sleep(0.05)

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
        if math.dist(hover['xyz'], resolved['points']['B']['xyz']) <= pass_radius:
            raise FlightError('pass_radius 不小于起飞悬停点到 B 的距离，B 会被立即跳过；请检查 A/B 与 pass_radius')
        predicted = (position[0], position[1], position[2] + height)
        if norm(velocity) > 0.1 or not arrived(predicted, velocity, yaw, hover, safety):
            raise FlightError('原地垂直起飞终点/当前航向不匹配 A 的起飞设置，或地面未静止；检查 A 和 takeoff/height')
        with self.lock:
            command = self.data.get('planner_cmd')
        if command and time.monotonic() - command[1] <= 0.5:
            raise FlightError('Planner 正在输出旧轨迹；请重启隔离 Planner 后再试')
        self.config, self.safety = resolved, safety
        self.origin = tuple(position)
        self.ros.loginfo('起飞原点 world=%s；实际 world 目标：hover=%s, B=%s(经过点), C=%s, D=%s, E=%s, F=%s(切换点), G=%s, H=%s, I=%s(终点)；轴方向与 yaw 仍按 world',
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

    def wait_landing(self):
        started = time.monotonic()
        acknowledged = False
        while not self.ros.is_shutdown():
            elapsed = time.monotonic() - started
            fcu = self.get('fcu', self.safety['fcu_timeout'])
            extended = self.get('extended', self.safety['fcu_timeout'])
            if fcu.connected and not fcu.armed and extended.landed_state == 1:
                self.status('DONE')
                self.active = False
                return
            self.health(landing=True)
            acknowledged = acknowledged or self.get('ctrl', self.safety['telemetry_timeout']).state == 5
            if not acknowledged and elapsed > self.settings['landing_ack_timeout']:
                raise FlightError('PX4Ctrl 未确认 AUTO_LAND，请立即接管')
            if elapsed > self.settings['landing_timeout']:
                raise FlightError('降落超时，未确认落地且锁桨，请立即接管')
            time.sleep(0.02)
        raise FlightError('ROS 关闭，未确认降落完成')

    def step_yaw(self, commanded_yaw, target, interval):
        error = wrap(math.radians(target['yaw_deg']) - commanded_yaw)
        limit = math.radians(self.config['direct']['max_yaw_rate_deg']) * interval
        change = max(-limit, min(limit, error))
        return wrap(commanded_yaw + change), change

    def bridge_command(self, commanded_yaw, target, interval):
        """经过点切换期间继续转发上一段 Planner 轨迹，避免在切换点急停。"""
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

    def begin_leg(self, leg, now, position, commanded_yaw):
        if leg['mode'] == 'planner':
            started, previous_id, goal_stamp = self.start_planner(leg['target'])
            return dict(started=started, previous_id=previous_id, goal_stamp=goal_stamp,
                        direct_start=None, direct_yaw=0.0, duration=None,
                        watchdog=MotionWatchdog(now, position))
        self.status('DIRECT')
        duration = direct_duration(position, commanded_yaw, self.config['points'][leg['target']], self.config['direct'])
        if duration + self.settings['arrival_dwell'] >= self.settings['direct_timeout']:
            raise FlightError('直控到 %s 所需时长超过 direct_timeout' % leg['target'], True)
        self.ros.loginfo('已关闭 Planner 指令通路（不发送 mandatory stop）；直控到 %s 参考轨迹时长 %.2fs',
                         leg['target'], duration)
        return dict(started=now, previous_id=0, goal_stamp=0.0,
                    direct_start=position, direct_yaw=commanded_yaw, duration=duration,
                    watchdog=MotionWatchdog(now, position))

    def run(self):
        position, _, yaw = self.takeoff()
        self.active = True
        # B 仅作为 Planner 经过点：进入 pass_radius 就发下一个目标，不悬停。
        legs = (
            dict(target='B', mode='planner', pass_through=True),
            dict(target='C', mode='planner', bridge=True),
            dict(target='D', mode='direct'),
            dict(target='E', mode='planner'),
            # F 是 Planner 与直控的切换点：到点后同样稳定悬停，再开始直控。
            dict(target='F', mode='planner'),
            dict(target='G', mode='direct'),
            dict(target='H', mode='direct'),
            dict(target='I', mode='planner'),
        )
        if legs[-1].get('pass_through'):
            raise FlightError('最后一个航点不能设置为经过点', True)
        index = 0
        hold_position = position
        commanded_yaw = yaw
        settled_since = None
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
                if leg.get('bridge'):
                    message, commanded_yaw = self.bridge_command(commanded_yaw, target, interval)
                else:
                    message = self.command(self.hold(hold_position, commanded_yaw))
                if self.planner_ready(motion['started'], motion['previous_id'], motion['goal_stamp']):
                    self.status(planning_stage)
                elif now - motion['started'] > self.settings['planner_first_command_timeout']:
                    raise FlightError('发送 %s 点后未收到新轨迹 ID 的 Planner 指令；旧轨迹不放行' % leg['target'], True)
            if self.stage == planning_stage:
                if now - motion['started'] > self.settings['planner_timeout']:
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
                            self.ros.loginfo('%s 点已连续稳定悬停 %.1fs，切换控制源飞往 %s',
                                             leg['target'], self.settings['arrival_dwell'], legs[index + 1]['target'])
                            index += 1
                            hold_position = position
                            motion = self.begin_leg(legs[index], now, position, commanded_yaw)
                            settled_since = None
                            last_leg = index == len(legs) - 1
            if self.stage in (waiting_stage, planning_stage):
                motion['watchdog'].check(now, position, target['xyz'], self.safety)
            if self.stage == 'DIRECT':
                target = self.config['points'][leg['target']]
                elapsed = now - motion['started']
                message = self.command(direct_sample(motion['direct_start'], motion['direct_yaw'], target,
                                                     motion['duration'], elapsed))
                if elapsed > self.settings['direct_timeout']:
                    raise FlightError('直控到达 %s 超时' % leg['target'], True)
                motion['watchdog'].check(now, position, target['xyz'], self.safety)
                if elapsed >= motion['duration'] and arrived(position, velocity, yaw, target, self.safety):
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
                        self.ros.loginfo('%s 点已连续稳定悬停 %.1fs，切换控制源飞往 %s',
                                         leg['target'], self.settings['arrival_dwell'], legs[index + 1]['target'])
                        index += 1
                        hold_position = position
                        motion = self.begin_leg(legs[index], now, position, commanded_yaw)
                        settled_since = None
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
                        self.ros.loginfo('%s 点已连续稳定悬停 %.1fs，请求降落',
                                         legs[-1]['target'], self.settings['hover_seconds'])
                        self.request_land()
                        self.wait_landing()
                        return
                else:
                    settled_since = None
                if now - motion['started'] > self.settings['hover_seconds'] + self.settings['direct_timeout']:
                    raise FlightError('%s 点无法连续稳定悬停' % legs[-1]['target'], True)
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
    args = parser.parse_args(argv)
    try:
        with args.config.expanduser().open(encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
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
    print('程控原地起飞 %.2fm → 起飞悬停点稳定 %.1fs → Planner 到 B（经过点，不悬停）→ Planner 到 C 并悬停 %.1fs'
          ' → 纯 PX4Ctrl 到 D 并悬停 %.1fs → Planner 到 E 并悬停 %.1fs → Planner 到 F 并悬停 %.1fs'
          ' → 纯 PX4Ctrl 到 G 并悬停 %.1fs → 纯 PX4Ctrl 到 H 并悬停 %.1fs'
          ' → Planner 到 I（终点）并悬停 %.1fs → 降落。' %
          (config['takeoff']['height'], config['takeoff']['hover_seconds'], config['mission']['arrival_dwell'],
           config['mission']['arrival_dwell'], config['mission']['arrival_dwell'],
           config['mission']['arrival_dwell'], config['mission']['arrival_dwell'],
           config['mission']['arrival_dwell'], config['mission']['hover_seconds']))
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
