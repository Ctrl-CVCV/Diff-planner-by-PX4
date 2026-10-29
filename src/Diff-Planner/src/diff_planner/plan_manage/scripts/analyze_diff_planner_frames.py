#!/usr/bin/env python3
"""Offline ROS1 bag frame audit. Never start ROS nodes or replay messages."""

import argparse
import bisect
from collections import Counter, defaultdict, deque
import json
import math
from pathlib import Path
import sys


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * fraction)]


def summary(values):
    return {'count': len(values), 'median': percentile(values, 0.5),
            'p95': percentile(values, 0.95), 'max': max(values) if values else None}


def quaternion_angle(first, second):
    first_norm = math.sqrt(sum(value * value for value in first))
    second_norm = math.sqrt(sum(value * value for value in second))
    if first_norm < 1e-12 or second_norm < 1e-12:
        return None
    cosine = abs(sum(left * right for left, right in zip(first, second)) /
                 (first_norm * second_norm))
    return math.degrees(2 * math.acos(min(1.0, cosine)))


def pose_sample(message):
    pose = message.pose.pose
    position, orientation = pose.position, pose.orientation
    return (message.header.stamp.to_sec(),
            (position.x, position.y, position.z),
            (orientation.w, orientation.x, orientation.y, orientation.z))


def quaternion_multiply(first, second):
    first_w, first_x, first_y, first_z = first
    second_w, second_x, second_y, second_z = second
    return (first_w * second_w - first_x * second_x - first_y * second_y - first_z * second_z,
            first_w * second_x + first_x * second_w + first_y * second_z - first_z * second_y,
            first_w * second_y - first_x * second_z + first_y * second_w + first_z * second_x,
            first_w * second_z + first_x * second_y - first_y * second_x + first_z * second_w)


def rotate_point(quaternion, point):
    norm = math.sqrt(sum(value * value for value in quaternion))
    rotation = tuple(value / norm for value in quaternion)
    result = quaternion_multiply(quaternion_multiply(rotation, (0,) + tuple(point)),
                                 (rotation[0], -rotation[1], -rotation[2], -rotation[3]))
    return result[1:]


def matched_samples(reference, estimates, max_dt):
    estimates = sorted(estimates, key=lambda entry: entry[0])
    times = [entry[0] for entry in estimates]
    for sample in sorted(reference, key=lambda entry: entry[0]):
        index = bisect.bisect_left(times, sample[0])
        candidates = estimates[max(0, index - 1):index + 1]
        if candidates:
            candidate = min(candidates, key=lambda entry: abs(entry[0] - sample[0]))
            if abs(candidate[0] - sample[0]) <= max_dt:
                yield sample, candidate


def compare_poses(reference, estimates, max_dt):
    distances, angles, intervals, offsets = [], [], [], []
    for (stamp, position, orientation), candidate in matched_samples(reference, estimates, max_dt):
        interval = abs(candidate[0] - stamp)
        offset = [candidate[1][axis] - position[axis] for axis in range(3)]
        distance = math.sqrt(sum(value * value for value in offset))
        angle = quaternion_angle(orientation, candidate[2])
        offsets.append(offset)
        distances.append(distance)
        if angle is not None:
            angles.append(angle)
        intervals.append(interval)
    return {'reference_samples': len(reference), 'paired_samples': len(distances),
            'unpaired_samples': len(reference) - len(distances),
            'max_pair_dt_s': max_dt, 'position_error_m': summary(distances),
            'orientation_error_deg': summary(angles), 'pair_dt_s': summary(intervals),
            'median_estimate_minus_reference_xyz_m':
            [percentile([offset[axis] for offset in offsets], 0.5) for axis in range(3)]}


def compare_attitude_alignment(reference, estimates, max_dt):
    first_alignment = None
    variations, differences = [], []
    for sample, candidate in matched_samples(reference, estimates, max_dt):
        reference_quaternion, estimate_quaternion = sample[2], candidate[2]
        inverse = (reference_quaternion[0],) + tuple(-value for value in reference_quaternion[1:])
        alignment = quaternion_multiply(estimate_quaternion, inverse)
        if first_alignment is None:
            first_alignment = alignment
        variations.append(quaternion_angle(first_alignment, alignment))
        differences.append(quaternion_angle(reference_quaternion, estimate_quaternion))
    return {'paired_samples': len(variations), 'first_imu_times_inverse_ekf_wxyz': first_alignment,
            'absolute_attitude_difference_deg': summary(differences),
            'alignment_change_from_first_deg': summary(variations),
            'note': 'FCU 与 LIO 的世界航向可以不同；绝对角差非零不直接表示错误。'}


def sample_cloud(message):
    from sensor_msgs.point_cloud2 import read_points

    total = message.width * message.height
    indices = sorted({index * (total - 1) // 31 for index in range(32)}) if total else []
    pixels = [(index % message.width, index // message.width) for index in indices]
    points = list(read_points(message, field_names=('x', 'y', 'z'), uvs=pixels)) if total else []
    return message.header.stamp.to_sec(), (total, points)


def compare_cloud_transform(world_clouds, body_clouds, odometry):
    body_by_stamp = {sample[0]: sample[1] for sample in body_clouds}
    errors = []
    paired_frames = 0
    for (stamp, (total, world_points)), pose in matched_samples(world_clouds, odometry, 1e-6):
        body = body_by_stamp.get(stamp)
        if body is None or body[0] != total or not total:
            continue
        paired_frames += 1
        for world_point, body_point in zip(world_points, body[1]):
            if not all(math.isfinite(value) for value in world_point + body_point):
                continue
            rotated = rotate_point(pose[2], body_point)
            errors.append(math.sqrt(sum((rotated[axis] + pose[1][axis] - world_point[axis]) ** 2
                                        for axis in range(3))))
    return {'paired_frames': paired_frames, 'sampled_point_error_m': summary(errors),
            'note': '抽样检验 cloud_world = R_odom * cloud_body + p_odom；不证明物理外参标定正确。'}


def inventory_warnings(inventory, drone_id):
    prefix = '/drone_%d_diff_planner_node/grid_map/' % drone_id
    warnings = []
    for topic in ('/Odometry', '/ekf/ekf_odom', '/cloud_registered',
                  prefix + 'occupancy', prefix + 'occupancy_inflate', '/tf'):
        if not inventory.get(topic, {}).get('messages', 0):
            warnings.append('未录到 %s，无法补出对应历史证据。' % topic)
    return warnings


def recorded_tf_path(edges, start, end):
    neighbors = defaultdict(set)
    for edge in edges.values():
        neighbors[edge['parent']].add(edge['child'])
        neighbors[edge['child']].add(edge['parent'])
    pending = deque([(start, [start])])
    visited = {start}
    while pending:
        frame, path = pending.popleft()
        if frame == end:
            return path
        for neighbor in sorted(neighbors[frame] - visited):
            visited.add(neighbor)
            pending.append((neighbor, path + [neighbor]))
    return None


def analyze(bag_paths, drone_id, max_dt):
    import rosbag

    inventory = {}
    poses = defaultdict(list)
    imu_poses = []
    cloud_samples = defaultdict(list)
    frames = defaultdict(Counter)
    children = defaultdict(Counter)
    ages = defaultdict(list)
    zero_stamps = Counter()
    invalid = Counter()
    cloud_counts = defaultdict(list)
    tf_edges = {}
    goals = []
    temporal_coverage = []
    temporal_warnings = []
    prefix = '/drone_%d_diff_planner_node/grid_map/' % drone_id
    pose_topics = ('/Odometry', '/ekf/ekf_odom', '/mavros/local_position/odom')
    cloud_topics = ('/cloud_registered', '/cloud_registered_body',
                    prefix + 'occupancy', prefix + 'occupancy_inflate')
    selected = pose_topics + cloud_topics + ('/mavros/imu/data', '/tf', '/tf_static',
                                            '/goal', '/setpoints_cmd')
    for path in bag_paths:
        with rosbag.Bag(str(path)) as bag:
            receptions = defaultdict(list)
            bag_start = bag.get_start_time()
            for topic, info in bag.get_type_and_topic_info().topics.items():
                entry = inventory.setdefault(topic, {'messages': 0, 'type': info.msg_type})
                entry['messages'] += info.message_count
            for topic, message, receipt in bag.read_messages(topics=selected):
                receptions[topic].append(receipt.to_sec() - bag_start)
                if topic in ('/tf', '/tf_static'):
                    for transform in message.transforms:
                        parent, child = transform.header.frame_id, transform.child_frame_id
                        key = parent + ' -> ' + child
                        translation = transform.transform.translation
                        rotation = transform.transform.rotation
                        values = {'stamp': transform.header.stamp.to_sec(),
                                  'xyz': [translation.x, translation.y, translation.z],
                                  'wxyz': [rotation.w, rotation.x, rotation.y, rotation.z]}
                        edge = tf_edges.setdefault(key, {'count': 0, 'first': values,
                                                         'topics': [], 'parent': parent,
                                                         'child': child})
                        edge['count'] += 1
                        edge['last'] = values
                        if topic not in edge['topics']:
                            edge['topics'].append(topic)
                    continue
                frames[topic][message.header.frame_id] += 1
                stamp = message.header.stamp.to_sec()
                if stamp > 0:
                    ages[topic].append(receipt.to_sec() - stamp)
                else:
                    zero_stamps[topic] += 1
                if topic in pose_topics:
                    children[topic][message.child_frame_id] += 1
                    sample = pose_sample(message)
                    values = sample[1] + sample[2]
                    if (stamp > 0 and all(math.isfinite(value) for value in values)
                            and sum(value * value for value in sample[2]) > 1e-12):
                        poses[topic].append(sample)
                    else:
                        invalid[topic] += 1
                elif topic in cloud_topics:
                    cloud_counts[topic].append(message.width * message.height)
                    if topic in ('/cloud_registered', '/cloud_registered_body') and stamp > 0:
                        cloud_samples[topic].append(sample_cloud(message))
                elif topic == '/mavros/imu/data':
                    orientation = message.orientation
                    quaternion = (orientation.w, orientation.x, orientation.y, orientation.z)
                    if (stamp > 0 and all(math.isfinite(value) for value in quaternion)
                            and sum(value * value for value in quaternion) > 1e-12
                            and message.orientation_covariance[0] != -1):
                        imu_poses.append((stamp, (0, 0, 0), quaternion))
                elif topic == '/goal':
                    position = message.pose.position
                    goals.append({'stamp': stamp, 'frame': message.header.frame_id,
                                  'xyz': [position.x, position.y, position.z]})
            coverage = {
                topic: {'count': len(times), 'first_receipt_offset_s': times[0],
                        'last_receipt_offset_s': times[-1],
                        'max_receipt_gap_s': max((right - left for left, right in
                                                zip(times, times[1:])), default=0.0)}
                for topic, times in receptions.items()
            }
            lio_tail = None
            lio_gaps_with_ekf = []
            if receptions['/Odometry'] and receptions['/ekf/ekf_odom']:
                ekf_times = receptions['/ekf/ekf_odom']
                for left, right in zip(receptions['/Odometry'], receptions['/Odometry'][1:]):
                    if right - left <= 0.5:
                        continue
                    first = bisect.bisect_left(ekf_times, left)
                    last = bisect.bisect_right(ekf_times, right)
                    inside = ekf_times[first:last]
                    boundaries = [left] + inside + [right]
                    max_ekf_gap = max(end - start for start, end in
                                      zip(boundaries, boundaries[1:]))
                    if inside and max_ekf_gap <= 0.5:
                        lio_gaps_with_ekf.append({
                            'start_receipt_offset_s': left,
                            'end_receipt_offset_s': right,
                            'duration_s': right - left,
                            'ekf_messages_inside': len(inside),
                            'max_ekf_receipt_gap_s': max_ekf_gap})
                if lio_gaps_with_ekf:
                    longest = max(lio_gaps_with_ekf, key=lambda gap: gap['duration_s'])
                    temporal_warnings.append(
                        '%s：包内 /Odometry 中断后恢复，最长 %.3fs（相对 %.3f～%.3fs），'
                        '期间 /ekf/ekf_odom 持续发布；末尾恢复或配对误差小不代表中断期间定位可信。' %
                        (path, longest['duration_s'], longest['start_receipt_offset_s'],
                         longest['end_receipt_offset_s']))
                lio_tail = max(0.0, receptions['/ekf/ekf_odom'][-1] -
                               receptions['/Odometry'][-1])
                if lio_tail > 0.5:
                    temporal_warnings.append(
                        '%s：末尾 %.3fs 未再记录 /Odometry，但 /ekf/ekf_odom 继续发布；'
                        '配对误差仅覆盖 LIO 存在的时段，不能证明随后 EKF 未漂移。' %
                        (path, lio_tail))
            temporal_coverage.append({'bag': str(path), 'bag_start_s': bag_start,
                                      'bag_duration_s': bag.get_end_time() - bag_start,
                                      'topics': coverage,
                                      'lio_internal_gaps_while_ekf_continues': lio_gaps_with_ekf,
                                      'lio_missing_tail_while_ekf_s': lio_tail})

    warnings = inventory_warnings(inventory, drone_id) + temporal_warnings
    lio_frames = set(frames['/Odometry'])
    ekf_frames = set(frames['/ekf/ekf_odom'])
    if lio_frames and ekf_frames and lio_frames != ekf_frames:
        warnings.append('LIO 与 EKF 的 frame_id 不一致；下方数值比较未应用 TF，不能当成坐标变换已正确的证明。')
    frame_paths = []
    for start in sorted(lio_frames):
        for end in sorted(ekf_frames):
            path = recorded_tf_path(tf_edges, start, end)
            frame_paths.append({'from': start, 'to': end, 'path': path})
            if path is None:
                warnings.append('录包内没有连接 %s 与 %s 的 TF 链；这是显示/接口断链，不直接证明数值位置错误。' %
                                (start, end))
    for topic, count in zero_stamps.items():
        warnings.append('%s 有 %d 条零时间戳；不能按 header 与定位做可靠同时刻配对。' % (topic, count))
    for topic, count in invalid.items():
        warnings.append('%s 排除了 %d 条无效/零时间戳定位。' % (topic, count))
    for child in {edge['child'] for edge in tf_edges.values()}:
        parents = {edge['parent'] for edge in tf_edges.values() if edge['child'] == child}
        if len(parents) > 1:
            warnings.append('TF 子坐标系 %s 出现多个父坐标系：%s；需按时间确认冲突。' %
                            (child, ', '.join(sorted(parents))))
    comparison = compare_poses(poses['/Odometry'], poses['/ekf/ekf_odom'], max_dt)
    if comparison['position_error_m']['p95'] is not None:
        if comparison['position_error_m']['p95'] > 0.10:
            warnings.append('LIO/EKF 数值位置差 p95 > 0.10m；这是排查阈值，不是飞行安全认证。')
        if comparison['orientation_error_deg']['p95'] is not None and comparison['orientation_error_deg']['p95'] > 5:
            warnings.append('LIO/EKF 姿态差 p95 > 5deg，需核对机体系、外参与融合状态。')
    if not comparison['paired_samples']:
        warnings.append('没有可按 header 时间配对的 LIO/EKF 样本，不能判断二者数值对齐。')
    cloud_comparison = compare_cloud_transform(cloud_samples['/cloud_registered'],
                                               cloud_samples['/cloud_registered_body'],
                                               poses['/Odometry'])
    if not cloud_comparison['paired_frames']:
        warnings.append('缺少同时间戳的 world/body 点云或 LIO 位姿，无法验证点云变换一致性。')
    elif cloud_comparison['sampled_point_error_m']['p95'] is not None:
        if cloud_comparison['sampled_point_error_m']['p95'] > 0.01:
            warnings.append('world/body 点云变换抽样误差 p95 > 0.01m，请核对点顺序、机体系与发布配置。')
    return {'bags': [str(path) for path in bag_paths], 'inventory': inventory,
            'temporal_coverage_by_bag': temporal_coverage,
            'frames': {topic: dict(counts) for topic, counts in frames.items()},
            'child_frames': {topic: dict(counts) for topic, counts in children.items()},
            'receipt_minus_header_s': {topic: summary(values) for topic, values in ages.items()},
            'zero_header_stamps': dict(zero_stamps),
            'cloud_point_counts': {topic: summary(values) for topic, values in cloud_counts.items()},
            'tf_edges': tf_edges, 'goals': goals, 'lio_vs_ekf_numeric_only': comparison,
            'recorded_lio_ekf_tf_paths': frame_paths,
            'registered_cloud_transform': cloud_comparison,
            'ekf_vs_fcu_attitude_alignment': compare_attitude_alignment(
                poses['/ekf/ekf_odom'], imu_poses, max_dt),
            'warnings': warnings,
            'limitations': ['只读离线分析，未调用 TF 变换、未重新运行定位或规划。',
                            '地图话题是裁剪后的可视化，不是完整内部占据缓冲区。',
                            '参数快照请结合录包器保存的 runtime_all.yaml 分析。',
                            '若多个 bag 时间重叠，会重复计数；请只传同一会话的不重叠分片。']}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags', nargs='+', type=Path)
    parser.add_argument('--drone-id', type=int, default=0)
    parser.add_argument('--max-pair-dt', type=float, default=0.03)
    parser.add_argument('--params', type=Path, help='可选：录包器保存的 runtime_all.yaml')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.drone_id < 0 or not 0 < args.max_pair_dt < float('inf'):
        parser.error('drone-id 必须非负，max-pair-dt 必须为有限正数')
    paths = [path.expanduser().resolve() for path in args.bags]
    output = args.output.expanduser().resolve()
    missing = [str(path) for path in paths if not path.is_file()]
    if missing:
        parser.error('输入 bag 不存在或不是文件：' + ', '.join(missing))
    if args.params and not args.params.expanduser().is_file():
        parser.error('参数快照不存在：' + str(args.params))
    if not output.parent.is_dir():
        parser.error('输出目录不存在：' + str(output.parent))
    if output in paths or output.exists():
        parser.error('输出文件必须为新文件，不能覆盖 bag 或已有报告')
    report = analyze(paths, args.drone_id, args.max_pair_dt)
    if args.params:
        import yaml
        parameters = yaml.safe_load(args.params.expanduser().read_text(encoding='utf-8'))
        report['runtime_parameters'] = {
            'source': str(args.params.expanduser().resolve()),
            'common': parameters.get('common'), 'mapping': parameters.get('mapping'),
            'ekf': parameters.get('ekf'),
            'grid_map': parameters.get('drone_%d_diff_planner_node' % args.drone_id, {}).get('grid_map'),
            'note': '参数服务器可能残留已退出节点的参数，须结合 nodes.txt 与节点信息核实。'}
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print('LIO/EKF 同时间戳数值比较（不应用 TF）：')
    print(json.dumps(report['lio_vs_ekf_numeric_only'], ensure_ascii=False, indent=2))
    for warning in report['warnings']:
        print('[注意] ' + warning)
    print('完整报告：%s' % output)
    return 0


if __name__ == '__main__':
    sys.exit(main())
