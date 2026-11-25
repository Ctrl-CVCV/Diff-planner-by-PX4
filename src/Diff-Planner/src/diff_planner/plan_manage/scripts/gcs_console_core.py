#!/usr/bin/env python3
"""gcs_console 的纯逻辑部分：读配置、算打点值、改 YAML、拼命令。

不依赖 PyQt / ROS，方便单元测试；GUI 只负责展示和进程管理。
"""

import math
import os
import re
import shutil
import time
from pathlib import Path

import yaml

POINT_NAMES = ('A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I')
MAX_Z_OFFSET = 0.30

LEVEL_OK = 'ok'
LEVEL_WARN = 'warn'
LEVEL_BAD = 'bad'

CTRL_STATES = {1: 'MANUAL_CTRL', 2: 'AUTO_HOVER', 3: 'CMD_CTRL',
               4: 'AUTO_TAKEOFF', 5: 'AUTO_LAND'}


def ctrl_state_name(state):
    return CTRL_STATES.get(int(state), 'UNKNOWN(%s)' % state)


def rate_from_stamps(stamps, now, window=2.0):
    """用最近 window 秒内的到达时间估算频率；样本不足返回 0。"""
    recent = [stamp for stamp in stamps if now - stamp <= window]
    if len(recent) < 2:
        return 0.0
    span = recent[-1] - recent[0]
    return (len(recent) - 1) / span if span > 1e-6 else 0.0


def topic_level(rate, age, expected_hz=None, warn_age=0.3, bad_age=1.0):
    """话题健康分级：频率过低或延迟过大分别给出 warn/bad。"""
    if age is None or age > bad_age:
        return LEVEL_BAD
    if expected_hz and rate < expected_hz * 0.5:
        return LEVEL_BAD
    if age > warn_age:
        return LEVEL_WARN
    if expected_hz and rate < expected_hz * 0.8:
        return LEVEL_WARN
    return LEVEL_OK


def motion_anomalies(xyz, prev_xyz, dt, speed, divergence,
                     speed_warn=0.65, speed_bad=1.0,
                     jump_warn=1.5, jump_bad=3.0,
                     divergence_warn=0.20, divergence_bad=0.35):
    """检查 EKF 位姿/速度/与 LIO 偏差，返回 [(level, message), ...]。"""
    alerts = []
    if xyz is None or any(not math.isfinite(float(value)) for value in xyz):
        return [(LEVEL_BAD, 'EKF 位姿含 NaN/无效值')]
    if speed is not None and math.isfinite(speed):
        if speed > speed_bad:
            alerts.append((LEVEL_BAD, '实测速度 %.2fm/s 超过 %.2f' % (speed, speed_bad)))
        elif speed > speed_warn:
            alerts.append((LEVEL_WARN, '实测速度 %.2fm/s 接近任务上限 %.2f' % (speed, speed_warn)))
    if prev_xyz is not None and dt and dt > 1e-3:
        jump = math.dist(tuple(float(v) for v in xyz), tuple(float(v) for v in prev_xyz)) / dt
        if jump > jump_bad:
            alerts.append((LEVEL_BAD, 'EKF 位置跳变 %.2fm/s（可能定位异常）' % jump))
        elif jump > jump_warn:
            alerts.append((LEVEL_WARN, 'EKF 位置变化 %.2fm/s 偏大' % jump))
    if divergence is not None and math.isfinite(divergence):
        if divergence > divergence_bad:
            alerts.append((LEVEL_BAD, 'EKF 与 LIO 偏差 %.2fm 超过 %.2f' % (divergence, divergence_bad)))
        elif divergence > divergence_warn:
            alerts.append((LEVEL_WARN, 'EKF 与 LIO 偏差 %.2fm 偏大' % divergence))
    return alerts


def workspace_root():
    return Path(os.environ.get('GCS_WS', str(Path.home() / 'gcs_ws')))


def mission_config_path(ws=None):
    ws = Path(ws) if ws else workspace_root()
    return ws / 'src/Diff-Planner/src/diff_planner/plan_manage/config/hybrid_mission.yaml'


def scripts_dir(ws=None):
    ws = Path(ws) if ws else workspace_root()
    return ws / 'src/Diff-Planner/src/diff_planner/plan_manage/scripts'


def load_points(path):
    """读取 points 段（只用于展示/计算，写回走 update_point_line 保留注释）。"""
    with Path(path).open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    points = config.get('points') or {}
    result = {}
    for label in POINT_NAMES:
        point = points.get(label)
        if not isinstance(point, dict):
            continue
        result[label] = dict(
            xyz=list(point.get('xyz') or [0.0, 0.0, 0.0]),
            yaw_deg=point.get('yaw_deg', 0.0),
            z_offset=point.get('z_offset', 0.0),
            measured=point.get('measured', True),
        )
    return result


CAPTURE_GROUND = 'ground'
CAPTURE_HOVER = 'hover'


def capture_values(point, pose, origin=(0.0, 0.0, 0.0), mode=CAPTURE_GROUND):
    """根据当前 EKF 位姿算打点结果。

    - x、y：相对任务原点的水平坐标（mission 用 takeoff_relative 时原点就是起飞点）；
    - 地面放置模式（默认，实际用法）：把飞机放在该点地面上读 EKF，
      z_offset = 实测 z - 原点 z，即该点地面相对起飞点的高度差；
    - 空中悬停模式：z_offset = 实测 z - 原点 z - 配置 z，即实测高度与设定高度的偏差。

    返回 dict(x, y, z, z_offset, configured_z, mode)；z_offset 超出 ±MAX_Z_OFFSET 抛 ValueError。
    """
    if mode not in (CAPTURE_GROUND, CAPTURE_HOVER):
        raise ValueError('未知的打点模式：%s' % mode)
    xyz = point.get('xyz') or [0.0, 0.0, 0.0]
    configured_z = float(xyz[2])
    x = float(pose[0]) - float(origin[0])
    y = float(pose[1]) - float(origin[1])
    z_rel = float(pose[2]) - float(origin[2])
    z_offset = z_rel if mode == CAPTURE_GROUND else z_rel - configured_z
    for name, value in (('x', x), ('y', y), ('z_offset', z_offset)):
        if not math.isfinite(value):
            raise ValueError('%s 不是有限数值' % name)
    if abs(z_offset) > MAX_Z_OFFSET:
        raise ValueError('z_offset %.3f 超出 ±%.2fm，检查原点/测量方式' % (z_offset, MAX_Z_OFFSET))
    return dict(x=round(x, 3), y=round(y, 3), z=round(z_rel, 3),
                z_offset=round(z_offset, 3), configured_z=configured_z, mode=mode)


def _point_line_pattern(label):
    return re.compile(r'^(?P<indent>\s*)(?P<key>%s\s*:)\s*\{(?P<body>.*)\}\s*$'
                      % re.escape(label), re.MULTILINE)


def update_point_line(text, label, x, y, z_offset, measured=None):
    """只改目标航点那一行，保留文件里其它内容和注释。

    - xyz 的 z 保持原值，只替换 x、y；
    - z_offset 已存在就替换，不存在就追加；
    - measured 为 True 且原来没有该键时追加。
    返回新文本；改完会重新解析校验，失败抛 ValueError。
    """
    match = _point_line_pattern(label).search(text)
    if not match:
        raise ValueError('在配置里找不到航点 %s 的单行定义' % label)
    body = match.group('body')

    xyz_match = re.search(r'xyz\s*:\s*\[([^\]]*)\]', body)
    if not xyz_match:
        raise ValueError('航点 %s 没有 xyz 字段' % label)
    old = [item.strip() for item in xyz_match.group(1).split(',')]
    configured_z = old[2] if len(old) >= 3 else '0.8'
    new_xyz = 'xyz: [%.3f, %.3f, %s]' % (x, y, configured_z)
    body = body[:xyz_match.start()] + new_xyz + body[xyz_match.end():]

    offset_text = 'z_offset: %.3f' % z_offset
    if re.search(r'z_offset\s*:', body):
        body = re.sub(r'z_offset\s*:\s*[-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?',
                      offset_text, body, count=1)
    else:
        body = body.rstrip().rstrip(',') + ', ' + offset_text

    if measured:
        if re.search(r'measured\s*:', body):
            body = re.sub(r'measured\s*:\s*(true|false)', 'measured: true', body, count=1)
        else:
            body = body.rstrip().rstrip(',') + ', measured: true'

    line = '%s%s {%s}' % (match.group('indent'), match.group('key'), body)
    updated = text[:match.start()] + line + text[match.end():]
    _verify_update(updated, label, x, y, z_offset)
    return updated


def _verify_update(text, label, x, y, z_offset):
    try:
        config = yaml.safe_load(text)
        point = config['points'][label]
        values = [float(point['xyz'][0]), float(point['xyz'][1]), float(point.get('z_offset', 0.0))]
    except (yaml.YAMLError, KeyError, TypeError, ValueError) as error:
        raise ValueError('改写后的 YAML 解析失败：%s' % error)
    expected = [float(x), float(y), float(z_offset)]
    if any(abs(a - b) > 1e-6 for a, b in zip(values, expected)):
        raise ValueError('改写校验不一致：%s != %s' % (values, expected))


def backup_config(path):
    """写回前备份原文件，返回备份路径。"""
    source = Path(path)
    target = source.with_name(source.name + '.bak_' + time.strftime('%Y%m%d_%H%M%S'))
    shutil.copy2(source, target)
    return target


def unique_path(path):
    """文件已存在时加时间戳后缀，避免覆盖上一次的录像/bag。"""
    path = Path(path)
    if not path.exists():
        return path
    stamp = time.strftime('%Y%m%d_%H%M%S')
    return path.with_name('%s_%s%s' % (path.stem, stamp, path.suffix))


def source_prefix(ws=None):
    ws = Path(ws) if ws else workspace_root()
    return ('source /opt/ros/noetic/setup.bash && '
            'source %s/devel/setup.bash' % ws)


def shell_command(ws, command):
    """source ROS 环境后 exec 目标命令；GUI 用 /bin/bash -c 执行它。"""
    return '%s && exec %s' % (source_prefix(ws), command)


def _quote(text):
    return "'" + text.replace("'", "'\"'\"'") + "'"


def cmd_lio(ws=None):
    ws = Path(ws) if ws else workspace_root()
    return shell_command(ws, 'bash %s/src/livox_ros_driver2/scripts/start_fastlio2_ekf.sh' % ws)


def cmd_px4ctrl(ws=None):
    return shell_command(ws, 'roslaunch px4ctrl run_ctrl_lio.launch')


def cmd_planner(ws=None):
    return shell_command(ws, 'roslaunch diff_planner run_exp_hybrid_mission.launch')


# /yolo/enable 的两种模式（仅 VLM+OpenCV 方案使用）：
#   0 = 危险标志，对应 C/D 点的危险品投放；
#   1 = 人像，对应 E/F/G 点的救助人员投放。
VLM_MODE_DANGER = 0
VLM_MODE_PERSON = 1


def cmd_yolo(ws=None, show_window=True, use_vlm=True):
    args = []
    if not show_window:
        args.append('show_window:=false')
    if not use_vlm:
        args.append('use_vlm:=false')
    suffix = (' ' + ' '.join(args)) if args else ''
    return shell_command(ws, 'roslaunch yolo_trt_ros yolo_trt.launch' + suffix)


def vlm_enable_value(mode):
    """把 GUI 的模式编号翻译成 /yolo/enable 的 Bool 值。"""
    if mode not in (VLM_MODE_DANGER, VLM_MODE_PERSON):
        raise ValueError('mode must be 0 (danger sign) or 1 (portrait)')
    return mode == VLM_MODE_PERSON


def cmd_vlm_trigger(mode, topic='/yolo/enable'):
    """命令行兜底：GUI 的 ROS 线程未连接时用 rostopic 发一次触发。"""
    value = 'true' if vlm_enable_value(mode) else 'false'
    return 'rostopic pub -1 %s std_msgs/Bool "data: %s"' % (topic, value)


def cmd_servo(ws=None):
    return shell_command(ws, 'roslaunch px4_servo payload_servos.launch')


def cmd_rviz_map(ws=None):
    return shell_command(ws, 'roslaunch diff_planner view_obstacle_map.launch')


def cmd_rviz_odom(ws=None):
    ws = Path(ws) if ws else workspace_root()
    config = ws / 'src/Diff-Planner/src/diff_planner/plan_manage/launch/include/lio_cloud_odom.rviz'
    return shell_command(ws, 'rviz -d %s' % config)


def mission_argv(ws=None, execute=False, map_verified=False,
                 direct_verified=False, no_hazard=False, no_rescue=False):
    ws = Path(ws) if ws else workspace_root()
    script = scripts_dir(ws) / 'hybrid_mission.py'
    argv = [str(script), '--execute' if execute else '--check']
    if execute and map_verified:
        argv.append('--map-verified')
    if execute and direct_verified:
        argv.append('--direct-path-verified')
    if no_hazard:
        argv.append('--no-hazard-drop')
    if no_rescue:
        argv.append('--no-rescue-drop')
    return argv


def cmd_mission(ws=None, **kwargs):
    ws = Path(ws) if ws else workspace_root()
    argv = mission_argv(ws, **kwargs)
    return shell_command(ws, 'python3 ' + ' '.join(_quote(arg) for arg in argv))


def cmd_run_bag(ws=None, output=None, raw_lidar=False):
    ws = Path(ws) if ws else workspace_root()
    output = output or (Path.home() / 'flight_logs' / ('console_' + time.strftime('%Y%m%d_%H%M%S')))
    script = scripts_dir(ws) / 'record_diff_planner_debug.py'
    suffix = ' --raw-lidar' if raw_lidar else ''
    return shell_command(ws, 'python3 %s --output %s%s'
                         % (_quote(str(script)), _quote(str(output)), suffix))


def debug_bag_topics(include_lidar=False):
    topics = ['/Odometry', '/cloud_registered', '/cloud_registered_body',
              '/ekf/ekf_odom', '/ekf/flight_path', '/path', '/tf', '/tf_static',
              '/mavros/state', '/mavros/extended_state',
              '/mavros/imu/data', '/mavros/imu/data_raw']
    if include_lidar:
        topics.append('/livox/lidar')
    return topics


def cmd_debug_bag(ws=None, output=None, include_lidar=False):
    ws = Path(ws) if ws else workspace_root()
    output = output or (Path.home() / 'flight_logs' / ('debug_' + time.strftime('%Y%m%d_%H%M%S') + '.bag'))
    output = unique_path(output)
    targets = debug_bag_topics(include_lidar) + ['/drone_0_diff_planner_node/grid_map/.*']
    regex = ' '.join(_quote(pattern) for pattern in targets)
    return shell_command(ws, 'rosbag record -O %s --regex %s' % (_quote(str(output)), regex))


def cmd_camera(ws=None, device='/dev/video0', fps=30.0, output=None):
    ws = Path(ws) if ws else workspace_root()
    output = output or (Path.home() / 'Videos' / ('usb_640x480_' + time.strftime('%Y%m%d_%H%M%S') + '.mp4'))
    output = unique_path(output)
    script = scripts_dir(ws) / 'usb_camera_record.py'
    return shell_command(ws, ' '.join([
        'python3', _quote(str(script)), '--device', _quote(device),
        '--width', '640', '--height', '480', '--fps', str(fps),
        '--output', _quote(str(output))]))
