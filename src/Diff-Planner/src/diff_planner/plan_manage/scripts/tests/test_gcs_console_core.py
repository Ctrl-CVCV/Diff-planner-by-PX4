#!/usr/bin/env python3
"""Unit tests for gcs_console_core (no Qt, no ROS)."""

import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import gcs_console_core as core  # noqa: E402

PACKAGE = Path(__file__).resolve().parents[2]
CONFIG = PACKAGE / 'config/hybrid_mission.yaml'

SAMPLE = '''# 顶部注释
points:
  A: {xyz: [0.0, 0.0, 0.0], yaw_deg: 0.0}
  C: {xyz: [0.43, 1.33, 0.80], yaw_deg: 180.0, z_offset: 0.0}
  G: {xyz: [2.66, 2.66, 0.8], yaw_deg: 90.0, measured: true}
  # I 的注释
  I                 : {xyz: [-0.06, -0.12, 0.8], yaw_deg: 180.0}
takeoff:
  height: 0.80
'''


class LoadPointsTest(unittest.TestCase):

    def test_reads_shipped_config(self):
        points = core.load_points(CONFIG)
        for label in ('A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J'):
            self.assertIn(label, points)
        self.assertEqual(points['J']['xyz'][2], 0.8)
        self.assertIsNone(points['I']['mirror'])
        self.assertEqual(points['I']['mirror_offset'], [0.0, 0.0])
        self.assertIsNone(core.mirror_link(points, 'F'))
        self.assertIsNone(core.mirror_link(points, 'I'))
        self.assertEqual(points['E']['mirror'], 'C')
        self.assertEqual(points['E']['mirror_offset'], [0.60, 0.0])
        self.assertEqual(core.mirror_link(points, 'C'), ('E', 0.6, 0.0))
        self.assertEqual(core.mirror_link(points, 'E'), ('C', -0.6, 0.0))

    def test_mirror_is_bidirectional(self):
        points = {'F': dict(xyz=[1.0, 2.0, 0.8], mirror=None),
                  'I': dict(xyz=[1.0, 2.0, 0.8], mirror='F')}
        self.assertEqual(core.mirror_of(points, 'I'), 'F')
        self.assertEqual(core.mirror_of(points, 'F'), 'I')
        self.assertIsNone(core.mirror_of(points, 'A'))

    def test_mirror_offset_links_c_and_e(self):
        points = {'C': dict(mirror=None, mirror_offset=[0.0, 0.0]),
                  'E': dict(mirror='C', mirror_offset=[0.6, 0.0])}
        self.assertEqual(core.mirror_link(points, 'C'), ('E', 0.6, 0.0))
        self.assertEqual(core.mirror_link(points, 'E'), ('C', -0.6, 0.0))

    def test_capture_records_f_and_i_independently(self):
        original = core.load_points(CONFIG)
        for label in ('F', 'I'):
            with self.subTest(label=label), tempfile.TemporaryDirectory() as directory:
                self.assertIsNone(core.mirror_link(original, label))
                updated = core.update_point_line(CONFIG.read_text(encoding='utf-8'),
                                                 label, 1.5, 2.5, 0.03, measured=True)
                path = Path(directory) / 'mission.yaml'
                path.write_text(updated, encoding='utf-8')
                points = core.load_points(path)
                self.assertEqual(points[label]['xyz'][:2], [1.5, 2.5])
                self.assertEqual(points[label]['xyz'][2], original[label]['xyz'][2])
                self.assertEqual(points[label]['yaw_deg'], original[label]['yaw_deg'])
                self.assertAlmostEqual(points[label]['z_offset'], 0.03)
                self.assertTrue(points[label]['measured'])
                self.assertIsNone(core.mirror_link(points, label))
                for other in original:
                    if other != label:
                        self.assertEqual(points[other], original[other])

    def test_capture_updates_mirror_without_touching_yaw(self):
        text = ('points:\n'
                '  F: {xyz: [1.0, 2.0, 0.8], yaw_deg: 90.0, z_offset: 0.0}\n'
                '  I: {xyz: [1.0, 2.0, 0.8], yaw_deg: 180.0, z_offset: 0.0, mirror: F}\n')
        updated = core.update_point_line(text, 'F', 1.5, 2.5, 0.03, measured=True)
        updated = core.update_point_line(updated, 'I', 1.5, 2.5, 0.03, measured=True)
        path = Path('/tmp/gcs_console_mirror_capture.yaml')
        path.write_text(updated, encoding='utf-8')
        points = core.load_points(path)
        self.assertEqual(points['F']['xyz'][:2], [1.5, 2.5])
        self.assertEqual(points['I']['xyz'][:2], [1.5, 2.5])
        self.assertAlmostEqual(points['F']['z_offset'], 0.03)
        self.assertAlmostEqual(points['I']['z_offset'], 0.03)
        self.assertEqual(points['F']['yaw_deg'], 90.0)
        self.assertEqual(points['I']['yaw_deg'], 180.0)
        self.assertEqual(points['I']['mirror'], 'F')

    def test_reads_sample(self):
        path = Path('/tmp/gcs_console_sample.yaml')
        path.write_text(SAMPLE, encoding='utf-8')
        points = core.load_points(path)
        self.assertEqual(points['C']['z_offset'], 0.0)
        self.assertTrue(points['G']['measured'])


class CaptureValuesTest(unittest.TestCase):

    def test_ground_mode_uses_measured_ground_height(self):
        point = dict(xyz=[0.43, 1.33, 0.8])
        values = core.capture_values(point, pose=(1.0, 2.0, 0.033), origin=(0.0, 0.0, 0.0),
                                     mode=core.CAPTURE_GROUND)
        self.assertAlmostEqual(values['x'], 1.0)
        self.assertAlmostEqual(values['y'], 2.0)
        self.assertAlmostEqual(values['z'], 0.033)
        self.assertAlmostEqual(values['z_offset'], 0.033,
                               msg='地面放置时 z_offset 就是该点地面相对起飞原点的高度')

    def test_hover_mode_subtracts_configured_height(self):
        point = dict(xyz=[0.43, 1.33, 0.8])
        values = core.capture_values(point, pose=(1.0, 2.0, 0.87), origin=(0.0, 0.0, 0.02),
                                     mode=core.CAPTURE_HOVER)
        self.assertAlmostEqual(values['z'], 0.85)
        self.assertAlmostEqual(values['z_offset'], 0.05)

    def test_rejects_implausible_offset(self):
        point = dict(xyz=[0.0, 0.0, 0.8])
        with self.assertRaisesRegex(ValueError, 'z_offset'):
            core.capture_values(point, pose=(0.0, 0.0, 1.5), mode=core.CAPTURE_GROUND)
        with self.assertRaisesRegex(ValueError, 'z_offset'):
            core.capture_values(point, pose=(0.0, 0.0, 1.5), mode=core.CAPTURE_HOVER)

    def test_rejects_unknown_mode(self):
        with self.assertRaisesRegex(ValueError, '模式'):
            core.capture_values(dict(xyz=[0, 0, 0.8]), pose=(0, 0, 0), mode='other')

    def test_rejects_nan(self):
        point = dict(xyz=[0.0, 0.0, 0.8])
        with self.assertRaises(ValueError):
            core.capture_values(point, pose=(float('nan'), 0.0, 0.8))


class UpdatePointLineTest(unittest.TestCase):

    def test_updates_xy_and_offset_preserving_comments(self):
        text = core.update_point_line(SAMPLE, 'C', 1.234, 5.678, 0.071, measured=True)
        self.assertIn('# 顶部注释', text)
        self.assertIn('# I 的注释', text)
        self.assertIn('takeoff:', text)
        self.assertIn('C: {xyz: [1.234, 5.678, 0.80]', text)
        self.assertIn('z_offset: 0.071', text)
        self.assertIn('measured: true', text)
        # 其它航点不受影响
        self.assertIn('A: {xyz: [0.0, 0.0, 0.0]', text)
        self.assertIn('G: {xyz: [2.66, 2.66, 0.8]', text)

    def test_appends_missing_offset_and_handles_spaced_colon(self):
        text = core.update_point_line(SAMPLE, 'I', -0.10, -0.20, -0.03)
        self.assertIn('I                 : {xyz: [-0.100, -0.200, 0.8]', text)
        self.assertIn('z_offset: -0.030', text)

    def test_keeps_existing_measured_true(self):
        text = core.update_point_line(SAMPLE, 'G', 2.5, 2.5, 0.01, measured=True)
        self.assertEqual(text.count('measured: true'), 1, 'G 原来已有 measured，不应重复追加')

    def test_missing_point_raises(self):
        with self.assertRaisesRegex(ValueError, '找不到航点'):
            core.update_point_line(SAMPLE, 'B', 0.0, 0.0, 0.0)

    def test_result_parses_back(self):
        text = core.update_point_line(SAMPLE, 'C', 1.0, 2.0, 0.02, measured=True)
        path = Path('/tmp/gcs_console_updated.yaml')
        path.write_text(text, encoding='utf-8')
        points = core.load_points(path)
        self.assertAlmostEqual(points['C']['xyz'][0], 1.0)
        self.assertAlmostEqual(points['C']['z_offset'], 0.02)
        self.assertTrue(points['C']['measured'])


class CommandsTest(unittest.TestCase):

    def test_lio_command_uses_start_script(self):
        command = core.cmd_lio('/home/nvidia/gcs_ws')
        self.assertIn('start_fastlio2_ekf.sh', command)
        self.assertIn('/opt/ros/noetic/setup.bash', command)

    def test_yolo_window_flag(self):
        self.assertIn('show_window:=false', core.cmd_yolo(show_window=False))
        self.assertNotIn('show_window:=false', core.cmd_yolo(show_window=True))

    def test_yolo_backend_flag(self):
        self.assertNotIn('use_vlm:=false', core.cmd_yolo(use_vlm=True))
        self.assertIn('use_vlm:=false', core.cmd_yolo(use_vlm=False))
        both = core.cmd_yolo(show_window=False, use_vlm=False)
        self.assertIn('show_window:=false', both)
        self.assertIn('use_vlm:=false', both)

    def test_vlm_mode_values(self):
        self.assertFalse(core.vlm_enable_value(core.VLM_MODE_DANGER))
        self.assertTrue(core.vlm_enable_value(core.VLM_MODE_PERSON))
        self.assertEqual((core.VLM_MODE_DANGER, core.VLM_MODE_PERSON), (0, 1))
        with self.assertRaises(ValueError):
            core.vlm_enable_value(2)

    def test_vlm_trigger_command(self):
        danger = core.cmd_vlm_trigger(core.VLM_MODE_DANGER)
        portrait = core.cmd_vlm_trigger(core.VLM_MODE_PERSON)
        self.assertIn('/yolo/enable', danger)
        self.assertIn('std_msgs/Bool', danger)
        self.assertIn('data: false', danger)
        self.assertIn('data: true', portrait)

    def test_vlm_service_commands(self):
        start = core.cmd_vlm_service_start()
        self.assertIn('systemctl --user start qwen2.5-vl.service', start)
        self.assertIn('http://127.0.0.1:8080/health', start)
        self.assertIn('systemctl --user stop qwen2.5-vl.service',
                      core.cmd_vlm_service_stop())

    def test_vlm_service_ready_unreachable(self):
        self.assertFalse(core.vlm_service_ready(
            health_url='http://127.0.0.1:9/health', timeout=0.2))

    def test_mission_flags(self):
        argv = core.mission_argv(execute=True, map_verified=True, direct_verified=True,
                                 no_hazard=True, no_rescue=False)
        self.assertIn('--execute', argv)
        self.assertIn('--map-verified', argv)
        self.assertIn('--direct-path-verified', argv)
        self.assertIn('--no-hazard-drop', argv)
        self.assertNotIn('--no-rescue-drop', argv)
        check = core.mission_argv(execute=False)
        self.assertEqual(check[1], '--check')

    def test_debug_bag_topics(self):
        topics = core.debug_bag_topics()
        for topic in ('/Odometry', '/ekf/ekf_odom', '/cloud_registered'):
            self.assertIn(topic, topics)
        self.assertNotIn('/livox/lidar', topics)
        self.assertIn('/livox/lidar', core.debug_bag_topics(include_lidar=True))

    def test_run_bag_raw_lidar_flag(self):
        self.assertIn('--raw-lidar', core.cmd_run_bag(raw_lidar=True))
        self.assertNotIn('--raw-lidar', core.cmd_run_bag(raw_lidar=False))

    def test_camera_command_is_640x480(self):
        command = core.cmd_camera(output='/tmp/test.mp4')
        self.assertIn('--width 640', command.replace("'", ''))
        self.assertIn('--height 480', command.replace("'", ''))


class HealthHelpersTest(unittest.TestCase):

    def test_rate_estimation(self):
        stamps = [100.0, 100.1, 100.2, 100.3]
        self.assertAlmostEqual(core.rate_from_stamps(stamps, now=100.35, window=2.0), 10.0, places=3)
        self.assertEqual(core.rate_from_stamps([], now=100.0), 0.0)
        self.assertEqual(core.rate_from_stamps([90.0], now=100.0), 0.0)

    def test_topic_levels(self):
        self.assertEqual(core.topic_level(10.0, 0.05, expected_hz=10.0), core.LEVEL_OK)
        self.assertEqual(core.topic_level(10.0, 0.5), core.LEVEL_WARN)
        self.assertEqual(core.topic_level(10.0, 2.0), core.LEVEL_BAD)
        self.assertEqual(core.topic_level(2.0, 0.05, expected_hz=10.0), core.LEVEL_BAD)
        self.assertEqual(core.topic_level(7.5, 0.05, expected_hz=10.0), core.LEVEL_WARN)

    def test_clean_motion_has_no_alerts(self):
        alerts = core.motion_anomalies((0.0, 0.0, 0.8), (0.0, 0.0, 0.8), 0.1, 0.1, 0.02)
        self.assertEqual(alerts, [])

    def test_nan_position_is_bad(self):
        alerts = core.motion_anomalies((float('nan'), 0.0, 0.0), None, None, None, None)
        self.assertEqual(alerts[0][0], core.LEVEL_BAD)

    def test_jump_and_speed_and_divergence(self):
        alerts = core.motion_anomalies((1.0, 0.0, 0.8), (0.0, 0.0, 0.8), 0.1, 2.0, 0.5)
        messages = ' '.join(message for _, message in alerts)
        self.assertIn('跳变', messages)
        self.assertIn('速度', messages)
        self.assertIn('偏差', messages)

    def test_ctrl_state_names(self):
        self.assertEqual(core.ctrl_state_name(1), 'MANUAL_CTRL')
        self.assertEqual(core.ctrl_state_name(5), 'AUTO_LAND')
        self.assertIn('UNKNOWN', core.ctrl_state_name(99))


if __name__ == '__main__':
    unittest.main()
