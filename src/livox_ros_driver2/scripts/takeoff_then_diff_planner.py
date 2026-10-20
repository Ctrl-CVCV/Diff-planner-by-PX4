#!/usr/bin/env python3
"""ROS1 PX4Ctrl takeoff -> confirmed hover -> one Diff-Planner goal.

Default: read-only preflight. No arming/mode services, no direct setpoints,
no retries, no automatic landing/disarm. Uses wall-clock deadlines.
"""
import argparse
from collections import deque
import math
import sys
import threading
import time

PLANNER = '/drone_0_diff_planner_node'
TRAJ_SERVER = '/drone_0_traj_server'
TOPICS = {
    'odom': '/ekf/ekf_odom', 'imu': '/mavros/imu/data',
    'fcu': '/mavros/state', 'land': '/mavros/extended_state',
    'rc': '/mavros/rc/in', 'ctrl': '/px4ctrl/state',
    'cloud': '/cloud_registered',
    'map': PLANNER + '/grid_map/occupancy_inflate',
    'cmd': '/setpoints_cmd',
}
MANUAL, HOVER, COMMAND, TAKEOFF = 1, 2, 3, 4


class SafetyError(RuntimeError):
    pass


def finite(values):
    return all(math.isfinite(v) for v in values)


def rc_error(channels):
    # Mirrors THIS workspace's input.cpp: dead zone 0.25, mode/gear >0.75.
    if len(channels) < 8:
        return 'RC通道不足8个'
    if channels[4] <= 1750 or channels[5] <= 1750:
        return 'CH5/CH6需均高于1750（auto-hover与command-control）'
    if any(abs(v - 1500) > 125 for v in channels[:4]):
        return 'CH1~CH4需全部居中，包括油门（本机源码使用中油门起飞）'
    return ''


def goal_for_hover(position, dx, dy):
    # World/EKF axes, NOT body axes. Keep measured hover altitude.
    return (position[0] + dx, position[1] + dy, position[2])


def check_vertical(z, ground, ceil, margin=0.2):
    if not finite((z, ground, ceil)) or ceil <= ground:
        raise SafetyError('高度参数无效')
    if not ground + margin < z < ceil - margin:
        raise SafetyError('目标z=%.3f，未留足上下边界余量：%.3f < z < %.3f'
                          % (z, ground + margin, ceil - margin))


def hover_ok(position, velocity, start, target_z):
    return (finite((*position, *velocity, *start, target_z))
            and math.dist(position[:2], start[:2]) <= 0.25
            and abs(position[2] - target_z) <= 0.12
            and math.sqrt(sum(v*v for v in velocity)) <= 0.12)


class Monitor:
    def __init__(self, rospy):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, PointCloud2
        from mavros_msgs.msg import State, ExtendedState, RCIn
        from quadrotor_msgs.msg import Px4ctrlState, PositionCommand
        types = dict(odom=Odometry, imu=Imu, fcu=State, land=ExtendedState,
                     rc=RCIn, ctrl=Px4ctrlState, cloud=PointCloud2,
                     map=PointCloud2, cmd=PositionCommand)
        self.lock = threading.Lock()
        self.data = {}
        self.times = {key: deque(maxlen=3000) for key in TOPICS}
        self.subs = [rospy.Subscriber(topic, types[key], self.callback,
                                     callback_args=key, queue_size=10,
                                     tcp_nodelay=True)
                     for key, topic in TOPICS.items()]

    def callback(self, msg, key):
        now = time.monotonic()
        with self.lock:
            self.data[key] = (msg, now)
            self.times[key].append(now)

    def get(self, key, age):
        with self.lock:
            entry = self.data.get(key)
        if entry is None or time.monotonic() - entry[1] > age:
            raise SafetyError('话题缺失/超时：' + TOPICS[key])
        return entry[0]

    def quiet(self, seconds=2.0):
        with self.lock:
            entry = self.data.get('cmd')
        return entry is None or time.monotonic() - entry[1] >= seconds

    def rate(self, key):
        now = time.monotonic()
        with self.lock:
            xs = [t for t in self.times[key] if now - t <= 2.0]
        return (len(xs)-1)/(xs[-1]-xs[0]) if len(xs) > 1 and xs[-1] > xs[0] else 0.0

    def command_after(self, after):
        with self.lock:
            entry = self.data.get('cmd')
        return entry is not None and entry[1] >= after and time.monotonic()-entry[1] < 0.3


def pose(odom):
    p, v = odom.pose.pose.position, odom.twist.twist.linear
    return (p.x, p.y, p.z), (v.x, v.y, v.z)


class Sequencer:
    def __init__(self, rospy, args):
        self.ros, self.args = rospy, args
        self.mon = Monitor(rospy)
        self.flight_requested = False
        self.goal_sent = False

    def param(self, name):
        if not self.ros.has_param(name):
            raise SafetyError('缺少参数：' + name)
        return self.ros.get_param(name)

    def sleep(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if self.ros.is_shutdown():
                raise SafetyError('ROS已关闭')
            time.sleep(0.05)

    def live(self):
        fcu = self.mon.get('fcu', 1.5)
        odom = self.mon.get('odom', 0.3)
        imu = self.mon.get('imu', 0.3)
        ctrl = self.mon.get('ctrl', 0.3)
        rc = self.mon.get('rc', 0.5)
        if not fcu.connected:
            raise SafetyError('MAVROS未连接飞控')
        error = rc_error(rc.channels)
        if error:
            raise SafetyError(error)
        p, v = pose(odom)
        q = odom.pose.pose.orientation
        if not finite((*p, *v, q.x, q.y, q.z, q.w)):
            raise SafetyError('EKF位置/速度/姿态有NaN或Inf')
        if not 0.9 <= math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w) <= 1.1:
            raise SafetyError('EKF姿态四元数无效')
        now = self.ros.Time.now().to_sec()
        for msg, label in ((odom, 'EKF'), (imu, 'IMU')):
            stamp = msg.header.stamp.to_sec()
            if stamp <= 0 or not -0.1 <= now-stamp <= 0.5:
                raise SafetyError(label + '时间戳过期或在未来，检查时间同步')
        return fcu, odom, ctrl

    def map_ready(self):
        self.mon.get('cloud', 2.0)
        self.mon.get('map', 2.0)
        # Visualization cloud is forward/height clipped. Fresh != verified free.

    def graph_ready(self):
        pubs, subs, _ = self.ros.get_master().getSystemState()[2]
        pubs, subs = dict(pubs), dict(subs)
        if set(pubs.get(TOPICS['cmd'], [])) != {TRAJ_SERVER}:
            raise SafetyError('/setpoints_cmd必须且只能由%s发布；先停止其他控制指令源' % TRAJ_SERVER)
        if '/px4ctrl' not in subs.get('/px4ctrl/takeoff_land', []):
            raise SafetyError('没有找到PX4Ctrl起飞订阅者')
        if '/px4ctrl' not in subs.get(TOPICS['cmd'], []):
            raise SafetyError('PX4Ctrl没有订阅/setpoints_cmd')
        if PLANNER not in subs.get('/goal', []):
            raise SafetyError('Diff-Planner未订阅/goal；检查flight_type=1')
        others = set(pubs.get('/goal', [])) - {self.ros.get_name()}
        if others:
            raise SafetyError('/goal存在其他发布者，请先关闭multipoint/其他目标源：' + str(sorted(others)))

    def preflight(self):
        if self.ros.get_param('/use_sim_time', False):
            raise SafetyError('本脚本用于实机，拒绝/use_sim_time=true')
        for name, expected in (
            ('/px4ctrl/auto_takeoff_land/enable', True),
            ('/px4ctrl/auto_takeoff_land/no_RC', False),
            (PLANNER+'/fsm/flight_type', 1),
            (PLANNER+'/grid_map/enable_virtual_wall', True),
        ):
            if self.param(name) != expected:
                raise SafetyError('%s必须为%s' % (name, expected))
        self.height = float(self.param('/px4ctrl/auto_takeoff_land/takeoff_height'))
        if not math.isfinite(self.height) or not 0.3 <= self.height <= 1.2:
            raise SafetyError('首次测试要求起飞相对高度在0.3~1.2m；请检查PX4Ctrl配置')
        self.ground = float(self.param(PLANNER+'/grid_map/virtual_ground'))
        self.ceil = float(self.param(PLANNER+'/grid_map/virtual_ceil'))
        max_vel = float(self.param(PLANNER+'/manager/max_vel'))
        if not math.isfinite(max_vel) or not 0 < max_vel <= 0.3:
            raise SafetyError('首次测试要求planner manager/max_vel在(0,0.3]m/s')
        self.graph_ready()
        fcu, odom, ctrl = self.live()
        if ctrl.state != MANUAL:
            raise SafetyError('起飞前PX4Ctrl需为MANUAL_CTRL(1)，当前=%s；不要在已悬停时重复起飞' % ctrl.state)
        if fcu.armed:
            raise SafetyError('此流程要求从未解锁状态开始，避免意外重复起飞')
        if not self.param('/px4ctrl/auto_takeoff_land/enable_auto_arm'):
            raise SafetyError('当前enable_auto_arm=false，与本脚本未解锁起飞流程不匹配；不要为脚本盲改参数')
        if self.mon.get('land', 1.5).landed_state != 1:
            raise SafetyError('PX4未报告ON_GROUND')
        p, v = pose(odom)
        if math.sqrt(sum(x*x for x in v)) > 0.1:
            raise SafetyError('机体不静止，速度超过0.1m/s')
        if not self.mon.quiet():
            raise SafetyError('起飞前/setpoints_cmd仍有消息；重启规划器/轨迹服务器并且不要提前给目标')
        for key in ('odom', 'imu'):
            hz = self.mon.rate(key)
            if hz < 100:
                raise SafetyError('%s频率仅%.1fHz，先解决低频问题' % (key, hz))
        self.map_ready()
        check_vertical(p[2]+self.height, self.ground, self.ceil)
        self.ros.loginfo('检查通过：EKF z=%.3f，相对起飞高度=%.3f，目标z≈%.3f；ODOM %.1fHz / IMU %.1fHz',
                         p[2], self.height, p[2]+self.height,
                         self.mon.rate('odom'), self.mon.rate('imu'))
        return p

    def wait_hover(self, start, sent):
        stable_since = None
        seen_takeoff = False
        deadline = sent + self.args.takeoff_timeout
        while time.monotonic() < deadline:
            fcu, odom, ctrl = self.live()
            now = time.monotonic()
            if not self.mon.quiet(0.3):
                raise SafetyError('起飞/悬停阶段提前收到规划指令，拒绝继续发送目标')
            if ctrl.state in (TAKEOFF, HOVER):
                seen_takeoff = True
                if not fcu.armed or fcu.mode != 'OFFBOARD':
                    raise SafetyError('起飞阶段未保持armed+OFFBOARD，请飞手接管')
            elif seen_takeoff or ctrl.state != MANUAL:
                raise SafetyError('起飞状态异常，当前PX4Ctrl=%s' % ctrl.state)
            if not seen_takeoff and now-sent > 5:
                raise SafetyError('5秒内未进入AUTO_TAKEOFF；查看PX4Ctrl拒绝原因，不自动重试')
            p, v = pose(odom)
            if ctrl.state == HOVER and hover_ok(p, v, start, start[2]+self.height):
                stable_since = stable_since or now
                if now-stable_since >= self.args.hover_seconds:
                    self.ros.loginfo('已确认AUTO_HOVER，连续稳定%.1f秒', self.args.hover_seconds)
                    return odom
            else:
                stable_since = None
            self.sleep(0.05)
        raise SafetyError('起飞/稳定悬停等待超时，不发送规划目标；请飞手接管')

    def handoff(self, goal_pub):
        from geometry_msgs.msg import PoseStamped
        fcu, odom, ctrl = self.live()
        if ctrl.state != HOVER or not fcu.armed or fcu.mode != 'OFFBOARD':
            raise SafetyError('发送目标前已不处于AUTO_HOVER+armed+OFFBOARD')
        self.map_ready()
        self.graph_ready()
        if not self.mon.quiet():
            raise SafetyError('发送目标前出现其他轨迹，不继续')
        p, _ = pose(odom)
        target = goal_for_hover(p, self.args.goal_dx, self.args.goal_dy)
        check_vertical(target[2], self.ground, self.ceil)
        if goal_pub.get_num_connections() < 1:
            raise SafetyError('/goal发布连接未建立')
        msg = PoseStamped()
        msg.header.stamp = self.ros.Time.now()
        msg.header.frame_id = 'world'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = target
        q = odom.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        sent = time.monotonic()
        goal_pub.publish(msg)  # Exactly one goal; never repeat/retry on failure.
        self.goal_sent = True
        self.ros.loginfo('已发送一次/goal=(%.3f,%.3f,%.3f)，偏移使用world/EKF轴，不是机头方向', *target)
        while time.monotonic()-sent < 10:
            fcu, _, ctrl = self.live()
            if not fcu.armed or fcu.mode != 'OFFBOARD' or ctrl.state not in (HOVER, COMMAND):
                raise SafetyError('交接期间退出悬停/指令控制，请飞手接管')
            if ctrl.state == COMMAND and self.mon.command_after(sent):
                self.ros.loginfo('交接确认：收到新的/setpoints_cmd，PX4Ctrl已进入CMD_CTRL(3)')
                return
            self.sleep(0.05)
        raise SafetyError('10秒内未确认CMD_CTRL。目标已发送，可能仍在规划，不能认为任务自动取消；请飞手接管')

    def run(self):
        self.sleep(3.0)  # Collect a real sample window; no publisher in check-only mode.
        self.preflight()
        if not self.args.execute:
            self.ros.loginfo('只检查完成：没有发布起飞/目标，没有调用解锁或模式服务')
            return
        if not self.args.map_verified:
            raise SafetyError('地图未确认：先解决空地有方块问题，再使用--map-verified明确确认')
        print('\n警告：接下来PX4Ctrl可能自动解锁并起飞，稳定悬停后自动发送一次短距离规划目标。\n'
              '请确认场地无人、地图正确、遥控器接管可用。脚本退出不代表飞机停止。', flush=True)
        if input('确实允许本次飞行请输入 TAKEOFF：').strip() != 'TAKEOFF':
            raise SafetyError('操作者取消，没有发送起飞')
        from quadrotor_msgs.msg import TakeoffLand
        from geometry_msgs.msg import PoseStamped
        takeoff_pub = self.ros.Publisher('/px4ctrl/takeoff_land', TakeoffLand, queue_size=1, latch=False)
        goal_pub = self.ros.Publisher('/goal', PoseStamped, queue_size=1, latch=False)
        deadline = time.monotonic()+5
        while takeoff_pub.get_num_connections() < 1 or goal_pub.get_num_connections() < 1:
            if time.monotonic() > deadline:
                raise SafetyError('起飞或目标发布连接未建立')
            self.sleep(0.05)
        start = self.preflight()  # Recheck AFTER operator input, before any actuation.
        msg = TakeoffLand()
        msg.takeoff_land_cmd = TakeoffLand.TAKEOFF
        sent = time.monotonic()
        self.flight_requested = True
        takeoff_pub.publish(msg)
        self.ros.logwarn('已发送一次起飞请求；不会自动重试')
        self.wait_hover(start, sent)
        if self.args.takeoff_only:
            self.ros.logwarn('按--takeoff-only结束：保持悬停，不给规划目标；降落由飞手负责')
            return
        self.handoff(goal_pub)
        self.ros.logwarn('脚本结束。轨迹服务器与PX4Ctrl继续运行；这不是到点/降落确认。请继续监护。')


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true', help='允许起飞及发送目标；否则只检查')
    p.add_argument('--map-verified', action='store_true', help='确认已人工验证地图、坐标、膨胀和路径空间')
    p.add_argument('--takeoff-only', action='store_true', help='仅起飞并确认悬停，不发送目标')
    p.add_argument('--goal-dx', type=float, default=0.5, help='相对稳定悬停点的world X偏移，默认0.5m')
    p.add_argument('--goal-dy', type=float, default=0.0, help='world Y偏移，默认0m；非机体系')
    p.add_argument('--hover-seconds', type=float, default=3.0, help='连续稳定悬停时间，至少3秒')
    p.add_argument('--takeoff-timeout', type=float, default=60.0, help='起飞等待超时秒数，范围15~120')
    return p


def main():
    p = parser()
    args = p.parse_args()
    if (not finite((args.goal_dx, args.goal_dy, args.hover_seconds, args.takeoff_timeout))
            or not 0.1 <= math.hypot(args.goal_dx, args.goal_dy) <= 2.0
            or not 3 <= args.hover_seconds <= 30
            or not 15 <= args.takeoff_timeout <= 120):
        p.error('目标水平距离需0.1~2m，稳定时间3~30秒，起飞超时15~120秒，数值必须有限')
    try:
        import rospy
        from quadrotor_msgs.msg import Px4ctrlState  # Require the current built interface.
    except ImportError as e:
        print('ROS接口导入失败：%s\n先执行 source ~/gcs_ws/devel/setup.bash；缺Px4ctrlState时需重新编译对应包。' % e, file=sys.stderr)
        return 2
    rospy.init_node('takeoff_then_diff_planner', anonymous=True)
    runner = Sequencer(rospy, args)
    try:
        runner.run()
        return 0
    except (SafetyError, EOFError, KeyboardInterrupt, rospy.ROSInterruptException) as e:
        rospy.logerr('流程停止：%s', e or '操作者中断')
        if runner.flight_requested:
            rospy.logerr('起飞请求已发出！退出不会撤销起飞或规划轨迹，也不会自动降落/停桨。请立即判断并用RC接管。')
        return 1
    except Exception as e:
        rospy.logerr('未预期异常：%s；若已起飞请立即用RC接管。脚本不会自动降落/停桨。', e)
        return 1


if __name__ == '__main__':
    sys.exit(main())
