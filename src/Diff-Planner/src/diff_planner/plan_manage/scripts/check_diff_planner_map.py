#!/usr/bin/env python3
"""Read-only ROS1 map and goal-handoff diagnostics; never publish flight commands."""

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time


def inflation_model(resolution, inflation):
    if not all(math.isfinite(value) and value > 0 for value in (resolution, inflation)):
        raise ValueError('地图分辨率和膨胀距离必须为有限正数')
    layers = math.ceil((inflation - 1e-5) / resolution)
    effective_resolution = inflation / 4 if layers > 4 else resolution
    layers = min(layers, 4)
    return effective_resolution, layers, effective_resolution * layers


class Monitor:
    def __init__(self, rospy, topics):
        self.lock = threading.Lock()
        self.messages = {}
        self.subscribers = [rospy.Subscriber(topic, message_type, self.callback,
                                            callback_args=key, queue_size=1)
                            for key, (topic, message_type) in topics.items()]

    def callback(self, message, key):
        now = time.monotonic()
        with self.lock:
            previous = self.messages.get(key)
            count = previous[0] + 1 if previous else 1
            first = previous[1] if previous else now
            self.messages[key] = (count, first, now, message)
        if key == 'goal':
            position = message.pose.position
            print('[EVENT] 收到 /goal=(%.3f, %.3f, %.3f)' %
                  (position.x, position.y, position.z), flush=True)
        elif key == 'ctrl' and (previous is None or previous[3].state != message.state):
            print('[EVENT] PX4Ctrl state=%s (2=悬停, 3=轨迹, 4=起飞)' % message.state,
                  flush=True)

    def snapshot(self):
        with self.lock:
            return dict(self.messages)


def cloud_summary(message, read_points, check_z, probes):
    lower = [math.inf] * 3
    upper = [-math.inf] * 3
    nearest = {name: math.inf for name in probes}
    count = 0
    slice_count = 0
    for point in read_points(message, field_names=('x', 'y', 'z'), skip_nans=True):
        if not all(math.isfinite(value) for value in point):
            continue
        count += 1
        for axis in range(3):
            lower[axis] = min(lower[axis], point[axis])
            upper[axis] = max(upper[axis], point[axis])
        if check_z is not None and abs(point[2] - check_z) <= 0.1:
            slice_count += 1
        for name, probe in probes.items():
            distance = max(abs(point[axis] - probe[axis]) for axis in range(3))
            nearest[name] = min(nearest[name], distance)
    return dict(points=count, minimum=lower if count else None,
                maximum=upper if count else None, slice_points=slice_count,
                nearest_center_linf={name: value if math.isfinite(value) else None
                                     for name, value in nearest.items()})


def inspect(rospy, args, topics, monitor, read_points):
    report = {'parameters': {}, 'topics': {}, 'checks': []}

    def check(level, text):
        report['checks'].append(dict(level=level, text=text))
        print('[%s] %s' % (level, text))

    planner = '/drone_%d_diff_planner_node' % args.drone_id
    names = ('grid_map/resolution', 'grid_map/obstacles_inflation',
             'grid_map/frame_id', 'grid_map/enable_virtual_wall',
             'grid_map/virtual_ground', 'grid_map/virtual_ceil', 'fsm/flight_type',
             'fsm/mondify_final_goal', 'optimization/obstacle_clearance',
             'optimization/obstacle_clearance_soft')
    for name in names:
        report['parameters'][name] = rospy.get_param(planner + '/' + name, None)
    parameters = report['parameters']
    print('\n运行参数:\n' + json.dumps(parameters, ensure_ascii=False, indent=2))
    try:
        resolution, layers, inflation = inflation_model(
            float(parameters['grid_map/resolution']),
            float(parameters['grid_map/obstacles_inflation']))
        report['effective_inflation'] = dict(resolution=resolution, layers=layers,
                                              axis_distance=inflation)
        check('INFO', '按源码推算: 有效分辨率 %.3fm, %d层, XYZ轴向膨胀 %.3fm' %
              (resolution, layers, inflation))
    except (TypeError, ValueError) as error:
        resolution = None
        check('FAIL', '地图参数缺失/无效: %s' % error)
    snapshot = monitor.snapshot()
    now = time.monotonic()
    for key, (topic, _) in topics.items():
        entry = snapshot.get(key)
        required = key in ('odom', 'cloud', 'occupancy', 'inflate', 'heartbeat')
        if entry is None:
            check('FAIL' if required else 'INFO', topic + ' 观察窗口内无消息')
            report['topics'][topic] = dict(messages=0)
            continue
        count, first, last, message = entry
        rate = (count - 1) / (last - first) if last > first else 0.0
        age = now - last
        header = getattr(message, 'header', None)
        frame = getattr(header, 'frame_id', '')
        stamp = header.stamp.to_sec() if header else None
        report['topics'][topic] = dict(messages=count, rate=rate, receive_age=age,
                                      frame=frame, stamp=stamp)
        check('FAIL' if required and age > 2 else 'INFO',
              '%s: %d条, %.1fHz, 接收距今%.2fs, frame=%s' %
              (topic, count, rate, age, frame or '-'))
    frames = {key: snapshot[key][3].header.frame_id
              for key in ('odom', 'cloud', 'occupancy', 'inflate') if key in snapshot}
    expected_frame = parameters['grid_map/frame_id']
    if any(not frame or frame != expected_frame for frame in frames.values()):
        check('WARN', 'frame标记不一致/为空: %s。地图直接使用XYZ，不会用TF自动对齐输入；需人工核对数值坐标。' % frames)
    check_z = args.check_z
    probes = {}
    if 'odom' in snapshot:
        position = snapshot['odom'][3].pose.pose.position
        if all(math.isfinite(value) for value in (position.x, position.y, position.z)):
            check_z = position.z if check_z is None else check_z
            probes = {'start': (position.x, position.y, check_z),
                      'goal': (position.x + args.goal_dx, position.y + args.goal_dy, check_z)}
            check('INFO', '仅诊断、不发送: 起点%s, 相对world目标%s' % (probes['start'], probes['goal']))
        else:
            check('FAIL', '里程计位置包含NaN/Inf')
    if args.check_z is None:
        check('WARN', '未指定--check-z，使用当前里程计z；地面检查请显式指定实际计划飞行的world/EKF高度。')
    if parameters['grid_map/enable_virtual_wall'] is True and check_z is not None:
        try:
            ground = float(parameters['grid_map/virtual_ground'])
            ceiling = float(parameters['grid_map/virtual_ceil'])
            if not all(math.isfinite(value) for value in (ground, ceiling)) or ceiling <= ground:
                raise ValueError('高度边界无效')
            if not ground < check_z < ceiling:
                check('FAIL', '检查高度%.3f不在虚拟边界(%.3f, %.3f)内' % (check_z, ground, ceiling))
            elif min(check_z - ground, ceiling - check_z) <= 0.2:
                check('WARN', '检查高度距虚拟地面/天花板不足或等于0.2m；起飞脚本会拒绝该目标。')
        except (TypeError, ValueError) as error:
            check('FAIL', '虚拟边界参数缺失/无效: %s' % error)
    else:
        check('WARN', '未启用虚拟边界或无检查高度，不能据此限制向上绕障。')
    for key in ('occupancy', 'inflate'):
        if key not in snapshot:
            continue
        try:
            summary = cloud_summary(snapshot[key][3], read_points, check_z, probes)
        except (ValueError, AssertionError) as error:
            check('FAIL', '%s点云格式无效: %s' % (key, error))
            continue
        report['topics'][topics[key][0]]['cloud'] = summary
        check('WARN' if not summary['points'] else 'INFO',
              '%s: 有限点%d, XYZ范围%s -> %s, 检查高度±0.1m点数%d' %
              (key, summary['points'], summary['minimum'], summary['maximum'], summary['slice_points']))
        if key == 'inflate' and resolution is not None:
            for name, distance in summary['nearest_center_linf'].items():
                if distance is not None:
                    check('WARN' if distance <= resolution * 0.5 + 1e-6 else 'INFO',
                          '%s距可视化膨胀栅格中心最近L∞距离%.3fm；仅作疑似占据线索，不是碰撞查询。' % (name, distance))
    publishers, subscribers, _ = rospy.get_master().getSystemState()[2]
    publishers, subscribers = dict(publishers), dict(subscribers)
    if planner not in subscribers.get('/goal', []):
        check('FAIL', '规划器未订阅/goal，检查flight_type=1与节点命名')
    expected_server = '/drone_%d_traj_server' % args.drone_id
    command_publishers = publishers.get('/setpoints_cmd', [])
    check('INFO' if set(command_publishers) == {expected_server} else 'FAIL',
          '/setpoints_cmd发布者: %s；预期仅%s' % (command_publishers, expected_server))
    check('INFO' if '/px4ctrl' in subscribers.get('/setpoints_cmd', []) else 'WARN',
          '/setpoints_cmd订阅者: %s；仅测地图时可不启动PX4Ctrl' % subscribers.get('/setpoints_cmd', []))
    if parameters['fsm/flight_type'] != 1:
        check('FAIL', '当前flight_type不是1，不能按此起飞脚本接收/goal')
    if parameters['fsm/mondify_final_goal'] is True:
        check('WARN', '自动修改碰撞目标已启用；收到0.5m目标不代表最终执行点不变。')
    if 'ctrl' in snapshot:
        check('INFO', '最新PX4Ctrl state=%s (2=悬停, 3=轨迹, 4=起飞)' % snapshot['ctrl'][3].state)
    check('WARN', '地图发布有前向和虚拟高度裁剪，空白不等于自由；零时间戳可能来自本机可视化发布代码。')
    check('INFO', '脚本不发布目标、起飞或控制指令，不改参数；检查完成也不构成实飞许可。')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=5.0, help='采样墙钟秒数，范围2~120')
    parser.add_argument('--drone-id', type=int, default=0)
    parser.add_argument('--check-z', type=float, help='检查高度，world/EKF绝对z，不是离地高度')
    parser.add_argument('--goal-dx', type=float, default=0.5, help='仅检查的world X偏移，不发送目标')
    parser.add_argument('--goal-dy', type=float, default=0.0, help='仅检查的world Y偏移，不发送目标')
    parser.add_argument('--output', type=Path, help='可选JSON报告路径')
    parser.add_argument('--rviz', action='store_true', help='检查后打开只含观察工具的专用RViz')
    args = parser.parse_args()
    values = (args.seconds, args.goal_dx, args.goal_dy) + (() if args.check_z is None else (args.check_z,))
    if not all(math.isfinite(value) for value in values) or not 2 <= args.seconds <= 120 or args.drone_id < 0:
        parser.error('参数必须有限，seconds为2~120，drone-id非负')
    try:
        import rospy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from quadrotor_msgs.msg import PositionCommand, Px4ctrlState
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs.point_cloud2 import read_points
        from std_msgs.msg import Empty

        rospy.get_master().getPid()
        rospy.init_node('check_diff_planner_map', anonymous=True)
        planner = '/drone_%d_diff_planner_node' % args.drone_id
        topics = dict(odom=('/ekf/ekf_odom', Odometry), cloud=('/cloud_registered', PointCloud2),
                      occupancy=(planner + '/grid_map/occupancy', PointCloud2),
                      inflate=(planner + '/grid_map/occupancy_inflate', PointCloud2),
                      goal=('/goal', PoseStamped), cmd=('/setpoints_cmd', PositionCommand),
                      ctrl=('/px4ctrl/state', Px4ctrlState),
                      trajectory=('/drone_%d_planning/trajectory' % args.drone_id, rospy.AnyMsg),
                      heartbeat=('/drone_%d_traj_server/heartbeat' % args.drone_id, Empty))
        monitor = Monitor(rospy, topics)
        print('只读采样%.1f秒；不会解锁/起飞/发布目标。' % args.seconds, flush=True)
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline and not rospy.is_shutdown():
            time.sleep(0.05)
        report = inspect(rospy, args, topics, monitor, read_points)
        for subscriber in monitor.subscribers:
            subscriber.unregister()
        if args.output:
            args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
            print('报告已保存: %s' % args.output)
        failed = any(check['level'] == 'FAIL' for check in report['checks'])
        if args.rviz:
            template = Path(__file__).resolve().parents[1] / 'launch/include/map_check.rviz'
            with tempfile.TemporaryDirectory(prefix='diff_map_rviz_') as directory:
                config = Path(directory) / 'map_check.rviz'
                config.write_text(template.read_text().replace('drone_0_', 'drone_%d_' % args.drone_id))
                subprocess.run(['rosrun', 'rviz', 'rviz', '-d', str(config)], check=True)
        return 1 if failed else 0
    except (ImportError, OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        print('检查失败: %s；请先source ~/gcs_ws/devel/setup.bash并确认ROS master可达。' % error,
              file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
