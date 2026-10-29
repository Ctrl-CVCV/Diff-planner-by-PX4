#!/usr/bin/env python3
"""Offline tests; never start a ROS master or publish messages."""

import contextlib
import importlib.util
import io
import math
from pathlib import Path
import re
import tempfile
import unittest
from unittest import mock


def load_script(name):
    path = Path(__file__).resolve().parents[1] / (name + '.py')
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


recorder = load_script('record_diff_planner_debug')
audit = load_script('analyze_diff_planner_frames')


class RecorderRules(unittest.TestCase):
    def matches(self, topic, raw=False):
        return any(re.match(pattern, topic) for pattern in recorder.topic_patterns(0, raw))

    def test_maps_and_all_planner_namespace_are_recorded(self):
        for topic in ('/drone_0_diff_planner_node/grid_map/occupancy',
                      '/drone_0_diff_planner_node/grid_map/occupancy_inflate',
                      '/drone_0_diff_planner_node/new_debug_topic',
                      '/drone_0_planning/trajectory', '/drone_0_traj_server/heartbeat'):
            self.assertTrue(self.matches(topic), topic)

    def test_frames_and_imu_inputs_are_recorded(self):
        for topic in ('/Odometry', '/ekf/ekf_odom', '/tf_static', '/cloud_registered_body',
                      '/mavros/imu/data_raw', '/mavros/imu/data', '/mavros/timesync_status'):
            self.assertTrue(self.matches(topic), topic)

    def test_raw_lidar_is_opt_in(self):
        self.assertFalse(self.matches('/livox/lidar'))
        self.assertTrue(self.matches('/livox/lidar', raw=True))

    def test_other_drone_namespace_is_not_selected(self):
        self.assertFalse(self.matches('/drone_1_planning/trajectory'))

    def test_finite_recording_does_not_split_forever(self):
        command = recorder.build_command(Path('/tmp/example'), 0, False, 5)
        self.assertIn('--duration=5.000', command)
        self.assertNotIn('--split', command)

    def test_indefinite_recording_repeats_latched_tf_across_splits(self):
        command = recorder.build_command(Path('/tmp/example'), 0, False, 0)
        self.assertIn('--split', command)
        self.assertIn('--repeat-latched', command)

    def test_dry_run_needs_no_ros(self):
        self.assertEqual(recorder.main(['--dry-run']), 0)

    def test_output_directory_creates_missing_parents(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary) / 'session' / 'recording'
            self.assertTrue(recorder.create_output_directory(directory))
            self.assertTrue(directory.is_dir())

    def test_output_directory_refuses_existing_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            with contextlib.redirect_stderr(io.StringIO()) as stderr:
                self.assertFalse(recorder.create_output_directory(Path(temporary)))
            self.assertIn('拒绝覆盖', stderr.getvalue())

    def test_output_permission_error_explains_empty_flight_dir(self):
        with mock.patch.object(Path, 'mkdir', side_effect=PermissionError('denied')):
            with contextlib.redirect_stderr(io.StringIO()) as stderr:
                self.assertFalse(recorder.create_output_directory(Path('/recording')))
        self.assertIn('FLIGHT_DIR', stderr.getvalue())
        self.assertIn('录包尚未启动', stderr.getvalue())
        self.assertIn('省略 --output', stderr.getvalue())

    def test_output_other_os_error_is_reported_without_traceback(self):
        with mock.patch.object(Path, 'mkdir', side_effect=OSError('disk full')):
            with contextlib.redirect_stderr(io.StringIO()) as stderr:
                self.assertFalse(recorder.create_output_directory(Path('/tmp/recording')))
        self.assertIn('disk full', stderr.getvalue())


class GeometryRules(unittest.TestCase):
    def test_quaternion_sign_does_not_change_rotation(self):
        self.assertAlmostEqual(audit.quaternion_angle((1, 0, 0, 0), (-1, 0, 0, 0)), 0)

    def test_quaternion_rotation(self):
        angle = math.sqrt(0.5)
        actual = audit.rotate_point((angle, 0, 0, angle), (1, 0, 0))
        for value, expected in zip(actual, (0, 1, 0)):
            self.assertAlmostEqual(value, expected)

    def test_pair_by_header_time_and_reject_large_gap(self):
        reference = [(10, (0, 0, 0), (1, 0, 0, 0)), (20, (0, 0, 0), (1, 0, 0, 0))]
        estimates = [(10.01, (0.03, 0.04, 0), (1, 0, 0, 0))]
        result = audit.compare_poses(reference, estimates, 0.03)
        self.assertEqual(result['paired_samples'], 1)
        self.assertEqual(result['unpaired_samples'], 1)
        self.assertAlmostEqual(result['position_error_m']['max'], 0.05)

    def test_empty_comparison_is_not_a_pass(self):
        result = audit.compare_poses([], [], 0.03)
        self.assertEqual(result['paired_samples'], 0)
        self.assertIsNone(result['position_error_m']['p95'])

    def test_constant_world_yaw_offset_has_zero_alignment_drift(self):
        reference = [(10, (0, 0, 0), (1, 0, 0, 0)), (11, (0, 0, 0), (1, 0, 0, 0))]
        estimates = [(10, (0, 0, 0), (0, 0, 0, 1)), (11, (0, 0, 0), (0, 0, 0, -1))]
        result = audit.compare_attitude_alignment(reference, estimates, 0.01)
        self.assertAlmostEqual(result['alignment_change_from_first_deg']['max'], 0)
        self.assertAlmostEqual(result['absolute_attitude_difference_deg']['max'], 180)

    def test_cloud_body_to_world(self):
        angle = math.sqrt(0.5)
        pose = [(10, (2, 3, 4), (angle, 0, 0, angle))]
        body = [(10, (1, [(1, 0, 0)]))]
        world = [(10, (1, [(2, 4, 4)]))]
        result = audit.compare_cloud_transform(world, body, pose)
        self.assertEqual(result['paired_frames'], 1)
        self.assertAlmostEqual(result['sampled_point_error_m']['max'], 0)

    def test_tf_path_can_be_indirect_but_not_invented(self):
        edges = {'first': {'parent': 'world', 'child': 'map'},
                 'second': {'parent': 'camera_init', 'child': 'map'}}
        self.assertEqual(audit.recorded_tf_path(edges, 'world', 'camera_init'),
                         ['world', 'map', 'camera_init'])
        self.assertIsNone(audit.recorded_tf_path(edges, 'world', 'body'))


class BagRules(unittest.TestCase):
    def test_internal_lio_gap_is_not_hidden_by_recovery(self):
        import rosbag
        import rospy
        from nav_msgs.msg import Odometry

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'lio_recovered.bag'
            with rosbag.Bag(str(path), 'w') as bag:
                for index in range(17):
                    stamp = rospy.Time.from_sec(10 + index * 0.25)
                    message = Odometry()
                    message.header.stamp = stamp
                    message.header.frame_id = 'world'
                    message.pose.pose.orientation.w = 1
                    if index in (0, 16):
                        bag.write('/Odometry', message, stamp)
                    bag.write('/ekf/ekf_odom', message, stamp)
            report = audit.analyze([path], 0, 0.03)
            coverage = report['temporal_coverage_by_bag'][0]
            self.assertEqual(coverage['lio_missing_tail_while_ekf_s'], 0)
            gaps = coverage['lio_internal_gaps_while_ekf_continues']
            self.assertEqual(len(gaps), 1)
            self.assertEqual(gaps[0]['duration_s'], 4)
            self.assertEqual(gaps[0]['max_ekf_receipt_gap_s'], 0.25)
            self.assertTrue(any('中断后恢复' in warning for warning in report['warnings']))

    def test_joint_lio_ekf_outage_is_not_labelled_continuous_ekf(self):
        import rosbag
        import rospy
        from nav_msgs.msg import Odometry

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'both_stopped.bag'
            with rosbag.Bag(str(path), 'w') as bag:
                for seconds in (10, 14):
                    stamp = rospy.Time.from_sec(seconds)
                    message = Odometry()
                    message.header.stamp = stamp
                    message.header.frame_id = 'world'
                    message.pose.pose.orientation.w = 1
                    bag.write('/Odometry', message, stamp)
                    bag.write('/ekf/ekf_odom', message, stamp)
            report = audit.analyze([path], 0, 0.03)
            self.assertEqual(report['temporal_coverage_by_bag'][0]
                             ['lio_internal_gaps_while_ekf_continues'], [])
            self.assertFalse(any('中断后恢复' in warning for warning in report['warnings']))

    def test_lio_stops_while_ekf_continues_even_with_small_paired_errors(self):
        import rosbag
        import rospy
        from nav_msgs.msg import Odometry

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'lio_stopped.bag'
            with rosbag.Bag(str(path), 'w') as bag:
                for seconds in (10, 11, 14):
                    stamp = rospy.Time.from_sec(seconds)
                    message = Odometry()
                    message.header.stamp = stamp
                    message.header.frame_id = 'world'
                    message.pose.pose.orientation.w = 1
                    if seconds < 14:
                        bag.write('/Odometry', message, stamp)
                    else:
                        message.pose.pose.position.z = 10
                    bag.write('/ekf/ekf_odom', message, stamp)
            report = audit.analyze([path], 0, 0.03)
            self.assertEqual(report['lio_vs_ekf_numeric_only']['position_error_m']['max'], 0)
            self.assertEqual(report['temporal_coverage_by_bag'][0]
                             ['lio_missing_tail_while_ekf_s'], 3)
            self.assertTrue(any('末尾 3.000s' in warning for warning in report['warnings']))

    def test_temporal_coverage_does_not_mix_independent_bags(self):
        import rosbag
        import rospy
        from nav_msgs.msg import Odometry

        with tempfile.TemporaryDirectory() as directory:
            paths = [Path(directory) / ('part_%d.bag' % index) for index in range(2)]
            for index, path in enumerate(paths):
                with rosbag.Bag(str(path), 'w') as bag:
                    stamp = rospy.Time.from_sec(10 + 100 * index)
                    message = Odometry()
                    message.header.stamp = stamp
                    message.header.frame_id = 'world'
                    message.pose.pose.orientation.w = 1
                    bag.write('/Odometry', message, stamp)
                    bag.write('/ekf/ekf_odom', message, stamp)
            report = audit.analyze(paths, 0, 0.03)
            self.assertEqual(len(report['temporal_coverage_by_bag']), 2)
            self.assertTrue(all(coverage['lio_missing_tail_while_ekf_s'] == 0
                                for coverage in report['temporal_coverage_by_bag']))
            self.assertFalse(any('末尾' in warning for warning in report['warnings']))

    def test_missing_bag_has_clear_error_without_creating_report(self):
        with tempfile.TemporaryDirectory() as directory:
            report = Path(directory) / 'report.json'
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr), self.assertRaises(SystemExit) as error:
                audit.main([str(Path(directory) / 'missing.bag'), '--output', str(report)])
            self.assertEqual(error.exception.code, 2)
            self.assertIn('输入 bag 不存在', stderr.getvalue())
            self.assertFalse(report.exists())

    def test_missing_parameter_snapshot_has_clear_error(self):
        with tempfile.TemporaryDirectory() as directory:
            bag = Path(directory) / 'present.bag'
            bag.touch()
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr), self.assertRaises(SystemExit) as error:
                audit.main([str(bag), '--params', str(Path(directory) / 'missing.yaml'),
                            '--output', str(Path(directory) / 'report.json')])
            self.assertEqual(error.exception.code, 2)
            self.assertIn('参数快照不存在', stderr.getvalue())

    def test_real_bag_with_empty_maps_and_zero_map_stamps(self):
        import rosbag
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        from sensor_msgs.point_cloud2 import create_cloud_xyz32
        from std_msgs.msg import Header

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'test.bag'
            stamp = rospy.Time.from_sec(10)
            prefix = '/drone_0_diff_planner_node/grid_map/'
            with rosbag.Bag(str(path), 'w') as bag:
                for topic, frame in (('/Odometry', 'camera_init'), ('/ekf/ekf_odom', 'world')):
                    message = Odometry()
                    message.header = Header(stamp=stamp, frame_id=frame)
                    message.pose.pose.orientation.w = 1
                    bag.write(topic, message, stamp)
                for topic, frame in (('/cloud_registered', 'camera_init'),
                                     ('/cloud_registered_body', 'body')):
                    message = create_cloud_xyz32(Header(stamp=stamp, frame_id=frame), [(1, 0, 0)])
                    bag.write(topic, message, stamp)
                for suffix in ('occupancy', 'occupancy_inflate'):
                    message = create_cloud_xyz32(Header(frame_id='world'), [])
                    bag.write(prefix + suffix, message, stamp)
                message = Imu()
                message.header.stamp = stamp
                message.orientation.w = 1
                bag.write('/mavros/imu/data', message, stamp)
            report = audit.analyze([path], 0, 0.03)
            self.assertEqual(report['lio_vs_ekf_numeric_only']['paired_samples'], 1)
            self.assertEqual(report['registered_cloud_transform']['paired_frames'], 1)
            self.assertEqual(report['zero_header_stamps'][prefix + 'occupancy'], 1)
            self.assertEqual(report['cloud_point_counts'][prefix + 'occupancy']['max'], 0)
            self.assertTrue(any('frame_id' in warning for warning in report['warnings']))
            self.assertEqual(recorder.coverage([path], 0)['missing_inputs_or_maps'], [])


if __name__ == '__main__':
    unittest.main()
