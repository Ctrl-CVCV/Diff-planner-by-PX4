#!/usr/bin/env python3
"""Record diagnostics and runtime parameters without publishing flight commands."""

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import time


BASE_TOPICS = (
    '/goal', '/traj_start_trigger', '/mandatory_stop_to_planner',
    '/px4ctrl/takeoff_land', '/px4ctrl/state', '/debugPx4ctrl', '/setpoints_cmd',
    '/Odometry', '/cloud_registered', '/cloud_registered_body', '/path',
    '/tf', '/tf_static', '/rosout', '/rosout_agg', '/ground_height_measurement',
    '/mavros/state', '/mavros/extended_state', '/mavros/rc/in', '/mavros/rc/out',
    '/mavros/imu/data', '/mavros/imu/data_raw', '/mavros/battery',
    '/mavros/local_position/odom', '/mavros/local_position/pose',
    '/mavros/local_position/velocity_local', '/mavros/local_position/velocity_body',
    '/mavros/setpoint_raw/attitude', '/mavros/setpoint_raw/target_attitude',
    '/mavros/estimator_status', '/mavros/statustext/recv', '/mavros/timesync_status',
    '/mavros/time_reference', '/vins_estimator/extrinsic',
)


def topic_patterns(drone_id, raw_lidar=False):
    topics = BASE_TOPICS + (('/livox/lidar', '/livox/imu') if raw_lidar else ())
    patterns = ['^' + re.escape(topic) + '$' for topic in topics]
    patterns.extend((r'^/ekf/.*$',
                     r'^/drone_%d_(diff_planner_node|planning|traj_server|mission)/.*$' % drone_id))
    return patterns


def build_command(directory, drone_id, raw_lidar, duration):
    command = ['rosbag', 'record', '--lz4', '--repeat-latched', '-b', '256', '--size=1024',
               '-O', str(directory / 'flight.bag'), '--regex']
    if duration:
        command.append('--duration=%.3f' % duration)
    else:
        command.append('--split')
    return command + topic_patterns(drone_id, raw_lidar)


def coverage(paths, drone_id):
    import rosbag

    counts = {}
    for path in paths:
        with rosbag.Bag(str(path)) as bag:
            for topic, info in bag.get_type_and_topic_info().topics.items():
                counts[topic] = counts.get(topic, 0) + info.message_count
    prefix = '/drone_%d_diff_planner_node/grid_map/' % drone_id
    required = ('/Odometry', '/ekf/ekf_odom', '/mavros/imu/data', '/cloud_registered',
                prefix + 'occupancy', prefix + 'occupancy_inflate')
    return {'counts': counts, 'missing_inputs_or_maps': [topic for topic in required
                                                       if not counts.get(topic)]}


def snapshot(directory, drone_id):
    commands = {
        'runtime_all.yaml': ['rosparam', 'dump'],
        'nodes.txt': ['rosnode', 'list'],
        'topics_verbose.txt': ['rostopic', 'list', '-v'],
    }
    for node in ('/ekf', '/laserMapping', '/px4ctrl',
                 '/drone_%d_diff_planner_node' % drone_id,
                 '/drone_%d_traj_server' % drone_id):
        commands[node.strip('/') + '_node.txt'] = ['rosnode', 'info', node]
    errors = {}
    for filename, command in commands.items():
        try:
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    text=True, timeout=12)
            (directory / filename).write_text(result.stdout, encoding='utf-8')
            if result.returncode:
                errors[filename] = result.stderr.strip()
            elif filename.endswith('_node.txt') and 'Publications:' not in result.stdout:
                errors[filename] = '节点信息不可用：' + result.stdout.strip()
        except (OSError, subprocess.TimeoutExpired) as error:
            errors[filename] = str(error)
    return errors


def create_output_directory(directory):
    try:
        directory.mkdir(parents=True)
    except FileExistsError:
        print('拒绝覆盖已有目录：%s' % directory, file=sys.stderr)
        return False
    except OSError as error:
        print('无法创建录包目录 %s：%s；录包尚未启动。' % (directory, error), file=sys.stderr)
        if directory == Path('/recording'):
            print('路径 /recording 通常说明当前终端的 FLIGHT_DIR 未设置或为空。',
                  file=sys.stderr)
        print('不要使用 sudo；可省略 --output，自动保存到 ~/flight_logs 下的新目录。',
              file=sys.stderr)
        return False
    return True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='新目录；已存在时拒绝覆盖')
    parser.add_argument('--drone-id', type=int, default=0)
    parser.add_argument('--raw-lidar', action='store_true', help='同时录原始 Livox 消息，磁盘负载更高')
    parser.add_argument('--duration', type=float, default=0, help='有限录制秒数；0 表示 Ctrl+C 结束')
    parser.add_argument('--dry-run', action='store_true', help='只打印命令，不连接 ROS 或创建文件')
    args = parser.parse_args(argv)
    if args.drone_id < 0 or not 0 <= args.duration < float('inf'):
        parser.error('drone-id 必须非负，duration 必须为有限非负数')
    directory = (args.output or Path.home() / 'flight_logs' /
                 datetime.now().strftime('%Y%m%d_%H%M%S_debug')).expanduser().resolve()
    command = build_command(directory, args.drone_id, args.raw_lidar, args.duration)
    print('只录制/查询；不解锁、不切模式、不发目标。\n待执行命令（不代表已经开始录包）：\n' +
          shlex.join(command), flush=True)
    if args.dry_run:
        return 0

    import rosgraph
    import rospy
    from sensor_msgs.msg import PointCloud2

    try:
        rosgraph.Master('/diff_debug_record_check').getPid()
    except Exception as error:
        print('无法连接 ROS master：%s' % error, file=sys.stderr)
        return 2
    if not create_output_directory(directory):
        return 2
    manifest = {'started_local': datetime.now().astimezone().isoformat(),
                'command': command, 'raw_lidar': args.raw_lidar,
                'snapshot_errors': snapshot(directory, args.drone_id)}
    if 'runtime_all.yaml' in manifest['snapshot_errors']:
        print('警告：运行参数快照失败，请检查 manifest.json。', flush=True)
    manifest_path = directory / 'manifest.json'
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')

    rospy.init_node('diff_debug_record_monitor', anonymous=True, disable_signals=True)
    samples = {}

    def observe(message, key):
        previous = samples.get(key, (0, 0, 0))
        samples[key] = (previous[0] + 1, message.width * message.height, time.monotonic())

    prefix = '/drone_%d_diff_planner_node/grid_map/' % args.drone_id
    subscribers = [rospy.Subscriber(prefix + suffix, PointCloud2, observe,
                                    callback_args=key, queue_size=1)
                   for suffix, key in (('occupancy', 'RAW'), ('occupancy_inflate', 'INFLATE'))]
    print('目录：%s\n每 5 秒报告地图收包状态；0 点也可以是有效的空地图。' % directory, flush=True)
    process = subprocess.Popen(command, start_new_session=True)

    def stop_recording(signum, frame):
        raise KeyboardInterrupt

    previous_sigterm = signal.signal(signal.SIGTERM, stop_recording)
    try:
        while process.poll() is None:
            time.sleep(5)
            now = time.monotonic()
            status = []
            for key in ('RAW', 'INFLATE'):
                sample = samples.get(key)
                status.append('%s=%d 帧/%d 点，距上一帧 %.1fs' %
                              (key, sample[0], sample[1], now - sample[2]) if sample else
                              '%s=尚未收到，先检查规划器是否启动' % key)
            print('[地图监视] ' + '; '.join(status), flush=True)
    except KeyboardInterrupt:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        print('\n等待 rosbag 正常关闭索引，请勿强制杀进程。', flush=True)
        process.wait()
    finally:
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            process.wait()
        for subscriber in subscribers:
            subscriber.unregister()
        rospy.signal_shutdown('recording finished')
        signal.signal(signal.SIGTERM, previous_sigterm)

    end_directory = directory / 'end_state'
    end_directory.mkdir()
    manifest['end_snapshot_errors'] = snapshot(end_directory, args.drone_id)
    paths = sorted(directory.glob('*.bag'))
    report = coverage(paths, args.drone_id)
    manifest.update({'finished_local': datetime.now().astimezone().isoformat(),
                     'recorder_returncode': process.returncode,
                     'bags': [str(path) for path in paths], 'coverage': report})
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    print('录制覆盖检查：%s' % manifest_path, flush=True)
    if report['missing_inputs_or_maps']:
        print('缺少消息：' + ', '.join(report['missing_inputs_or_maps']), file=sys.stderr)
        return 2
    return 0 if process.returncode in (0, -signal.SIGINT, 128 + signal.SIGINT) else 2


if __name__ == '__main__':
    sys.exit(main())
