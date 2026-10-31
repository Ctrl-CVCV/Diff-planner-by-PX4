#!/usr/bin/env python3
"""Offline mission tests: no ROS master, vehicles, or real publishers."""

import copy
import importlib.util
import json
import math
from pathlib import Path
import re
import threading
from types import SimpleNamespace
import unittest
from unittest import mock
import xml.etree.ElementTree as ElementTree

import yaml


PACKAGE = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('hybrid_mission', PACKAGE / 'scripts/hybrid_mission.py')
mission = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(mission)


def configuration():
    with (PACKAGE / 'config/hybrid_mission.yaml').open() as stream:
        config = yaml.safe_load(stream)
    config['points'] = dict(A=dict(xyz=[0.0, 0.0, 0.0], yaw_deg=0.0),
                            B=dict(xyz=[0.5, 0.0, 0.8], yaw_deg=0.0),
                            C=dict(xyz=[0.5, 0.5, 0.8], yaw_deg=90.0),
                            D=dict(xyz=[0.0, 0.0, 0.8], yaw_deg=90.0),
                            E=dict(xyz=[0.0, 0.5, 0.8], yaw_deg=90.0),
                            F=dict(xyz=[0.0, 1.0, 0.8], yaw_deg=90.0),
                            G=dict(xyz=[0.0, 1.5, 0.8], yaw_deg=90.0),
                            H=dict(xyz=[0.0, 2.0, 0.8], yaw_deg=90.0),
                            I=dict(xyz=[0.0, 0.0, 0.8], yaw_deg=180.0))
    return config


class Clock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += max(0.001, duration)


class Command:
    TRAJECTORY_STATUS_READY = 1

    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id='world')
        for key in ('position', 'velocity', 'acceleration', 'jerk'):
            setattr(self, key, SimpleNamespace(x=0.0, y=0.0, z=0.0))
        self.yaw = self.yaw_dot = 0.0
        self.trajectory_flag = 1
        self.trajectory_id = 0


class Landing:
    TAKEOFF = 1
    LAND = 2


class ConfigAndGeometry(unittest.TestCase):
    def setUp(self):
        self.config = configuration()

    def test_takeoff_origin_translation_preserves_world_axes_and_source(self):
        original = copy.deepcopy(self.config)
        resolved = mission.resolve_config(self.config, (10, -3, 0.034))
        self.assertEqual(resolved['points']['A']['xyz'], [10, -3, 0.034])
        self.assertEqual(resolved['points']['B']['xyz'][:2], [10.5, -3])
        self.assertEqual(resolved['points']['C']['xyz'][:2], [10.5, -2.5])
        self.assertAlmostEqual(resolved['points']['B']['xyz'][2], 0.834)
        self.assertAlmostEqual(mission.hover_point(resolved)['xyz'][2], 0.834)
        self.assertEqual(resolved['safety'], original['safety'])
        self.assertEqual(resolved['points']['C']['yaw_deg'], 90)
        self.assertEqual(resolved['points']['D']['xyz'][:2], [10, -3])
        self.assertAlmostEqual(resolved['points']['D']['xyz'][2], 0.834)
        self.assertEqual(resolved['frame_id'], 'world')
        self.assertEqual(self.config, original)

    def test_legacy_world_coordinates_do_not_translate(self):
        self.config.pop('coordinate_mode')
        self.config['points']['A']['xyz'][2] = 0.8
        resolved = mission.resolve_config(self.config, (10, -3, 0.034))
        self.assertEqual(resolved, self.config)
        self.assertEqual(mission.hover_point(resolved)['xyz'], [0, 0, 0.8])

    def test_quintic_endpoints_and_limits(self):
        start = self.config['points']['B']['xyz']
        target = self.config['points']['C']
        limits = self.config['direct']
        duration = mission.direct_duration(start, 0, target, limits)
        first = mission.direct_sample(start, 0, target, duration, 0)
        final = mission.direct_sample(start, 0, target, duration, duration + 1)
        self.assertEqual(first['position'], tuple(start))
        self.assertEqual(final['position'], tuple(target['xyz']))
        self.assertEqual(first['velocity'], (0, 0, 0))
        self.assertEqual(final['acceleration'], (0, 0, 0))
        self.assertAlmostEqual(final['yaw'], math.pi / 2)
        for index in range(1001):
            sample = mission.direct_sample(start, 0, target, duration, duration * index / 1000)
            self.assertLessEqual(mission.norm(sample['velocity']), limits['max_speed'] + 1e-8)
            self.assertLessEqual(mission.norm(sample['acceleration']), limits['max_acceleration'] + 1e-8)
            self.assertLessEqual(mission.norm(sample['jerk']), limits['max_jerk'] + 1e-8)
            self.assertLessEqual(abs(sample['yaw_dot']), math.radians(limits['max_yaw_rate_deg']) + 1e-8)

    def test_yaw_uses_shortest_arc(self):
        target = dict(xyz=[0, 0, 0.8], yaw_deg=-179)
        sample = mission.direct_sample([0, 0, 0.8], math.radians(179), target, 2, 1)
        self.assertAlmostEqual(abs(sample['yaw']), math.pi)
        self.assertGreater(sample['yaw_dot'], 0)

    def test_arrival_requires_yaw_and_low_speed(self):
        target = self.config['points']['B']
        self.assertTrue(mission.arrived(target['xyz'], (0, 0, 0), 0, target, self.config['safety']))
        too_fast = self.config['safety']['arrival_speed'] * 2
        self.assertFalse(mission.arrived(target['xyz'], (too_fast, 0, 0), 0, target, self.config['safety']))
        self.assertFalse(mission.arrived(target['xyz'], (0, 0, 0), math.pi / 2, target, self.config['safety']))

    def test_stall_requests_protective_landing(self):
        watchdog = mission.MotionWatchdog(0, (0, 0, 0.8))
        with self.assertRaises(mission.FlightError) as caught:
            watchdog.check(3.01, (0, 0, 0.8), (0.5, 0, 0.8), self.config['safety'])
        self.assertTrue(caught.exception.land_allowed)

    def test_arrival_or_motion_does_not_trigger_stall(self):
        safety = self.config['safety']
        watchdog = mission.MotionWatchdog(0, (0, 0, 0.8))
        watchdog.check(2.5, (0.1, 0, 0.8), (0.5, 0, 0.8), safety)
        watchdog.check(5.0, (0.2, 0, 0.8), (0.5, 0, 0.8), safety)
        watchdog.check(20.0, (0.5, 0, 0.8), (0.5, 0, 0.8), safety)

    def test_launch_isolates_planner_and_preserves_old_default(self):
        base = ElementTree.parse(PACKAGE / 'launch/exp/run_exp_single_lio.launch').getroot()
        argument = base.find("arg[@name='position_cmd_topic']")
        self.assertEqual(argument.attrib['default'], '/setpoints_cmd')
        remap = base.find("node/remap[@from='position_cmd']")
        self.assertEqual(remap.attrib['to'], '$(arg position_cmd_topic)')
        hybrid = ElementTree.parse(PACKAGE / 'launch/exp/run_exp_hybrid_mission.launch').getroot()
        topic = hybrid.find("include/arg[@name='position_cmd_topic']").attrib['value']
        self.assertEqual(topic, self.config['topics']['planner_cmd'])
        self.assertEqual(base.find("arg[@name='hold_at_end']").attrib['default'], 'false')
        self.assertEqual(hybrid.find("include/arg[@name='hold_at_end']").attrib['value'], 'true')

    def test_recorder_captures_both_sides_of_gate_and_status(self):
        spec = importlib.util.spec_from_file_location('mission_recorder', PACKAGE / 'scripts/record_diff_planner_debug.py')
        recorder = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(recorder)
        for key in ('planner_cmd', 'output_cmd', 'status'):
            self.assertTrue(any(re.match(pattern, self.config['topics'][key]) for pattern in recorder.topic_patterns(0)))


class RuntimeRules(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.config = configuration()
        self.runner = mission.Mission.__new__(mission.Mission)
        self.runner.config = self.config
        self.runner.source_config = copy.deepcopy(self.config)
        self.runner.origin = None
        self.runner.read_only = False
        self.runner.safety = self.config['safety']
        self.runner.settings = self.config['mission']
        self.runner.types = dict(command=Command, status=SimpleNamespace, stop=SimpleNamespace, land=Landing)
        self.runner.ros = mock.Mock()
        self.runner.ros.is_shutdown.return_value = False
        self.runner.ros.Time.now.return_value = SimpleNamespace(to_sec=lambda: 1000.0)
        self.runner.ros.get_name.return_value = '/hybrid_mission'
        self.runner.publishers = {key: mock.Mock() for key in ('command', 'goal', 'status', 'stop', 'land')}
        self.runner.stage = 'PREFLIGHT'
        self.runner.active = self.runner.land_sent = self.runner.stopped = False
        self.runner.takeoff_sent = False
        self.runner.lock = threading.Lock()
        self.runner.data = {}

    def simulate_takeoff(self, ctrl_at=None, confirmed_at=0.3, position_at=None, origin=(0, 0, 0)):
        runner, clock = self.runner, self.clock
        start = clock.now
        ground = origin
        runner.config = mission.resolve_config(runner.source_config, origin)
        runner.safety = runner.config['safety']
        runner.origin = origin
        target = tuple(mission.hover_point(runner.config)['xyz'])
        runner.preflight = mock.Mock(return_value=(ground, (0, 0, 0), 0))
        runner.health = mock.Mock(side_effect=lambda **kwargs: (
            position_at(clock.now - start) if position_at else target if clock.now - start >= 1 else ground,
            (0, 0, 0), 0))

        def get(key, *args):
            elapsed = clock.now - start
            if key == 'ctrl':
                return SimpleNamespace(state=ctrl_at(elapsed) if ctrl_at else 2 if elapsed >= 1 else 4 if elapsed >= 0.1 else 1)
            if key == 'fcu':
                return SimpleNamespace(connected=True, armed=elapsed >= confirmed_at,
                                       mode='OFFBOARD' if elapsed >= confirmed_at else 'POSCTL')
            if key == 'extended':
                return SimpleNamespace(landed_state=2 if elapsed >= 1 else 1)
            raise AssertionError('unexpected topic ' + key)

        runner.get = mock.Mock(side_effect=get)

    def test_takeoff_grace_then_stable_hover_without_position_or_goal(self):
        self.simulate_takeoff()
        with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic), mock.patch.object(mission.time, 'sleep', self.clock.sleep):
            result = self.runner.takeoff()
        self.assertEqual(result[0], tuple(mission.hover_point(self.runner.config)['xyz']))
        self.assertGreaterEqual(self.clock.now, 104)
        self.assertEqual(self.runner.stage, 'TAKEOFF_HOVER')
        self.runner.preflight.assert_called_once()
        self.runner.publishers['land'].publish.assert_called_once()
        self.assertEqual(self.runner.publishers['land'].publish.call_args.args[0].takeoff_land_cmd, Landing.TAKEOFF)
        for key in ('command', 'goal', 'stop'):
            self.runner.publishers[key].publish.assert_not_called()
        with self.assertRaisesRegex(mission.FlightError, '禁止重复起飞'):
            self.runner.takeoff()
        self.assertEqual(self.runner.publishers['land'].publish.call_count, 1)

    def test_no_auto_takeoff_ack_does_not_retry(self):
        self.simulate_takeoff(ctrl_at=lambda elapsed: 1, confirmed_at=100, position_at=lambda elapsed: (0, 0, 0))
        with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic), mock.patch.object(mission.time, 'sleep', self.clock.sleep):
            with self.assertRaisesRegex(mission.FlightError, '未确认 AUTO_TAKEOFF') as caught:
                self.runner.takeoff()
        self.assertFalse(caught.exception.land_allowed)
        self.assertEqual(self.runner.publishers['land'].publish.call_count, 1)
        self.runner.publishers['goal'].publish.assert_not_called()

    def test_takeoff_fcu_confirmation_expires(self):
        self.simulate_takeoff(ctrl_at=lambda elapsed: 4, confirmed_at=100, position_at=lambda elapsed: (0, 0, 0))
        with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic), mock.patch.object(mission.time, 'sleep', self.clock.sleep):
            with self.assertRaisesRegex(mission.FlightError, '状态确认超时'):
                self.runner.takeoff()
        self.runner.publishers['command'].publish.assert_not_called()

    def test_takeoff_fcu_may_update_before_controller_state(self):
        self.simulate_takeoff(confirmed_at=0)
        with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic), mock.patch.object(mission.time, 'sleep', self.clock.sleep):
            self.runner.takeoff()
        self.assertEqual(self.runner.stage, 'TAKEOFF_HOVER')

    def simulate_fcu_receipts(self, period=None):
        self.simulate_takeoff(confirmed_at=0)
        original_get = self.runner.get

        def get(key, *args):
            if key != 'fcu':
                return original_get(key, *args)
            entry = self.runner.data.get(key)
            if entry is None or (period is not None and self.clock.now - entry[1] >= period):
                self.runner.receive(original_get(key, *args), key)
            return mission.Mission.get(self.runner, key, *args)

        self.runner.get = get

    def test_one_hz_fcu_jitter_does_not_abort_takeoff(self):
        self.simulate_fcu_receipts(period=1.08)
        with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic), mock.patch.object(mission.time, 'sleep', self.clock.sleep):
            self.runner.takeoff()
        self.assertEqual(self.runner.stage, 'TAKEOFF_HOVER')
        self.assertGreaterEqual(self.clock.now, 104)
        self.runner.publishers['goal'].publish.assert_not_called()

    def test_old_one_second_fcu_timeout_reproduces_takeoff_abort(self):
        self.simulate_fcu_receipts(period=1.08)
        self.runner.safety['fcu_timeout'] = 1.0
        with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic), mock.patch.object(mission.time, 'sleep', self.clock.sleep):
            with self.assertRaisesRegex(mission.FlightError, 'fcu 断更') as caught:
                self.runner.takeoff()
        self.assertIn('阈值=1.000s', str(caught.exception))
        self.assertIn('mode=OFFBOARD', str(caught.exception))
        self.runner.publishers['goal'].publish.assert_not_called()

    def test_fcu_dropout_still_aborts_without_goal_or_retry(self):
        self.simulate_fcu_receipts()
        with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic), mock.patch.object(mission.time, 'sleep', self.clock.sleep):
            with self.assertRaisesRegex(mission.FlightError, 'fcu 断更') as caught:
                self.runner.takeoff()
        self.assertIn('话题=/mavros/state', str(caught.exception))
        self.assertIn('阶段=TAKEOFF_HOVER', str(caught.exception))
        self.assertIn('阈值=3.000s', str(caught.exception))
        self.assertFalse(caught.exception.land_allowed)
        self.assertGreater(self.clock.now, 103)
        self.assertLess(self.clock.now, 103.1)
        self.assertEqual(self.runner.publishers['land'].publish.call_count, 1)
        self.runner.publishers['goal'].publish.assert_not_called()
        self.runner.publishers['command'].publish.assert_not_called()

    def test_takeoff_requires_observed_takeoff_state(self):
        self.simulate_takeoff(ctrl_at=lambda elapsed: 2)
        with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic):
            with self.assertRaisesRegex(mission.FlightError, '未观察到 AUTO_TAKEOFF'):
                self.runner.takeoff()

    def test_takeoff_drift_and_unconfirmed_motion_rejected(self):
        for position in ((0.3, 0, 0), (0, 0, 0.2)):
            with self.subTest(position=position):
                self.setUp()
                self.simulate_takeoff(position_at=lambda elapsed: position)
                with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic):
                    with self.assertRaises(mission.FlightError) as caught:
                        self.runner.takeoff()
                self.assertFalse(caught.exception.land_allowed)
                self.runner.publishers['goal'].publish.assert_not_called()

    def test_takeoff_hover_timeout_and_manual_takeover(self):
        for state in (lambda elapsed: 4, lambda elapsed: 4 if elapsed < 0.5 else 1):
            with self.subTest(state=state):
                self.setUp()
                self.simulate_takeoff(ctrl_at=state)
                with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic), mock.patch.object(mission.time, 'sleep', self.clock.sleep):
                    with self.assertRaises(mission.FlightError) as caught:
                        self.runner.takeoff()
                self.assertFalse(caught.exception.land_allowed)
                self.runner.publishers['goal'].publish.assert_not_called()

    def test_takeoff_confirmed_fcu_loss_stops_mission(self):
        self.simulate_takeoff()
        original_get = self.runner.get

        def get(key, *args):
            message = original_get(key, *args)
            if key == 'fcu' and self.clock.now >= 100.5:
                message.mode = 'POSCTL'
            return message

        self.runner.get = get
        with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic), mock.patch.object(mission.time, 'sleep', self.clock.sleep):
            with self.assertRaisesRegex(mission.FlightError, '退出 armed'):
                self.runner.takeoff()
        self.runner.publishers['goal'].publish.assert_not_called()

    def test_takeoff_hover_dwell_resets_on_drift(self):
        self.simulate_takeoff(position_at=lambda elapsed: (0, 0, 0) if elapsed < 1 else
                              (0.15, 0, 0.8) if 2 <= elapsed < 2.5 else (0, 0, 0.8))
        with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic), mock.patch.object(mission.time, 'sleep', self.clock.sleep):
            self.runner.takeoff()
        self.assertGreaterEqual(self.clock.now, 105.5)

    def test_read_only_cannot_takeoff(self):
        self.runner.read_only = True
        with self.assertRaisesRegex(mission.FlightError, '只读检查'):
            self.runner.takeoff()
        self.runner.publishers['land'].publish.assert_not_called()

    def prepare_preflight(self):
        runner = self.runner
        params = {
            '/use_sim_time': False,
            '/px4ctrl/auto_takeoff_land/enable': True,
            '/px4ctrl/auto_takeoff_land/no_RC': False,
            '/px4ctrl/auto_takeoff_land/enable_auto_arm': True,
            '/px4ctrl/auto_takeoff_land/takeoff_height': 0.8,
            '/drone_0_traj_server/traj_server/hold_at_end': True,
            '/drone_0_diff_planner_node/fsm/flight_type': 1,
            '/drone_0_diff_planner_node/grid_map/enable_virtual_wall': True,
            '/drone_0_diff_planner_node/grid_map/virtual_ground': 0.1,
            '/drone_0_diff_planner_node/grid_map/virtual_ceil': 1.2,
        }
        runner.ros.get_param.side_effect = lambda name, default=None: params.get(name, default)
        runner.graph = mock.Mock()
        runner.health = mock.Mock(return_value=((0, 0, 0), (0, 0, 0), 0))
        telemetry = SimpleNamespace(
            landed_state=1,
            header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 1000.0)),
            pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0, z=0.0))))
        runner.get = mock.Mock(return_value=telemetry)
        runner.safety['preflight_estimator_seconds'] = 0.0
        return params

    def test_preflight_ground_and_height_match(self):
        runner = self.runner
        params = self.prepare_preflight()
        runner.preflight()
        runner.health.assert_called_with(phase='ground')
        params['/px4ctrl/auto_takeoff_land/takeoff_height'] = 0.6
        with self.assertRaisesRegex(mission.FlightError, 'takeoff_height'):
            runner.preflight()
        params['/px4ctrl/auto_takeoff_land/takeoff_height'] = 0.8
        runner.health.return_value = ((0.5, 0, 0), (0, 0, 0), 0)
        runner.preflight()
        self.assertEqual(runner.config['points']['B']['xyz'], [1, 0, 0.8])
        runner.health.return_value = ((0.5, 0, 0), (0, 0, 0), math.pi / 2)
        with self.assertRaisesRegex(mission.FlightError, '不匹配 A'):
            runner.preflight()
        runner.health.return_value = ((0, 0, 0), (0, 0, 0), 0)
        runner.get.return_value.landed_state = 2
        with self.assertRaisesRegex(mission.FlightError, 'ON_GROUND'):
            runner.preflight()
        runner.publishers['land'].publish.assert_not_called()

    def test_preflight_locks_current_origin_without_accumulated_offsets(self):
        self.prepare_preflight()
        runner = self.runner
        runner.health.return_value = ((10, -3, 0.034), (0, 0, 0), 0)
        runner.preflight()
        runner.preflight()
        self.assertEqual(runner.origin, (10, -3, 0.034))
        self.assertEqual(runner.config['points']['B']['xyz'][:2], [10.5, -3])
        self.assertAlmostEqual(runner.config['points']['B']['xyz'][2], 0.834)
        runner.health.return_value = ((12, -4, 0.04), (0, 0, 0), 0)
        runner.preflight()
        self.assertEqual(runner.config['points']['B']['xyz'][:2], [12.5, -4])
        self.assertEqual(runner.source_config['points']['A']['xyz'], [0, 0, 0])
        runner.status('TAKEOFF')
        status = json.loads(runner.publishers['status'].publish.call_args.args[0].data)
        self.assertEqual(status['origin_world'], [12, -4, 0.04])
        self.assertEqual(status['configured_points']['A']['xyz'], [0, 0, 0])
        self.assertAlmostEqual(status['takeoff_hover']['xyz'][2], 0.84)

    def test_relative_height_not_blocked_by_script_virtual_wall_precheck(self):
        params = self.prepare_preflight()
        params['/drone_0_diff_planner_node/grid_map/enable_virtual_wall'] = False
        self.runner.health.return_value = ((10, -3, 0.4), (0, 0, 0), 0)
        self.runner.preflight()
        self.assertAlmostEqual(self.runner.config['points']['D']['xyz'][2], 1.2)
        self.assertFalse(any('/grid_map/' in call.args[0]
                             for call in self.runner.ros.get_param.call_args_list))
        self.runner.publishers['land'].publish.assert_not_called()

    def test_pass_radius_must_be_positive_and_smaller_than_hover_to_b(self):
        self.prepare_preflight()
        self.runner.health.return_value = ((10, -3, 0.034), (0, 0, 0), 0)
        self.runner.settings['pass_radius'] = 0.6   # 夹具中起飞悬停点到 B 只有 0.5m
        with self.assertRaisesRegex(mission.FlightError, 'pass_radius'):
            self.runner.preflight()
        self.runner.settings['pass_radius'] = 0.0
        with self.assertRaisesRegex(mission.FlightError, 'pass_radius'):
            self.runner.preflight()

    def test_preflight_tolerates_stale_lio_stamp(self):
        self.prepare_preflight()
        self.runner.safety['preflight_estimator_seconds'] = 0.5
        self.runner.get.return_value.header = SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 999.27))
        with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic), \
                mock.patch.object(mission.time, 'sleep', self.clock.sleep):
            self.runner.preflight()
        self.runner.publishers['land'].publish.assert_not_called()

    def test_preflight_rejects_diverged_ekf_before_takeoff(self):
        self.prepare_preflight()
        self.runner.safety['preflight_estimator_seconds'] = 0.5
        telemetry = self.runner.get.return_value
        lio = copy.deepcopy(telemetry)
        lio.pose.pose.position.x = 0.6
        self.runner.get.side_effect = lambda key, *args: lio if key == 'lio' else telemetry
        with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic), \
                mock.patch.object(mission.time, 'sleep', self.clock.sleep):
            with self.assertRaisesRegex(mission.FlightError, '起飞前估计器不达标'):
                self.runner.preflight()

    def test_preflight_rejects_waypoints_outside_fence(self):
        self.prepare_preflight()
        self.runner.safety['max_distance_from_origin'] = 1.0
        with self.assertRaisesRegex(mission.FlightError, '围栏'):
            self.runner.preflight()

    def test_unmeasured_point_blocks_execution(self):
        self.config['points']['E']['measured'] = False
        self.assertEqual(mission.unmeasured_points(self.config), ['E'])
        self.config['points']['E']['measured'] = True
        self.assertEqual(mission.unmeasured_points(self.config), [])

    def test_goal_is_published_in_world_not_relative_coordinates(self):
        self.prepare_preflight()
        runner = self.runner
        runner.health.return_value = ((10, -3, 0.034), (0, 0, 0), 0)
        runner.preflight()
        runner.types['goal'] = lambda: SimpleNamespace(header=SimpleNamespace(), pose=SimpleNamespace(
            position=SimpleNamespace(), orientation=SimpleNamespace()))
        runner.send_goal()
        goal = runner.publishers['goal'].publish.call_args.args[0]
        self.assertEqual(goal.header.frame_id, 'world')
        self.assertEqual(mission.vector(goal.pose.position)[:2], (10.5, -3))
        self.assertAlmostEqual(goal.pose.position.z, 0.834)
        runner.send_goal('D')
        goal = runner.publishers['goal'].publish.call_args.args[0]
        self.assertEqual(mission.vector(goal.pose.position)[:2], (10, -3))
        self.assertAlmostEqual(goal.pose.position.z, 0.834)
        self.assertAlmostEqual(goal.pose.orientation.z, math.sin(math.pi / 4))
        self.assertAlmostEqual(goal.pose.orientation.w, math.cos(math.pi / 4))

    def test_preflight_accepts_d_without_script_coordinate_limits(self):
        self.prepare_preflight()
        for coordinates in ([0, 0, 0], [5, -5, 10]):
            with self.subTest(coordinates=coordinates):
                self.runner.source_config['points']['D']['xyz'] = coordinates
                self.runner.preflight()
                self.assertEqual(self.runner.config['points']['D']['xyz'], coordinates)

    def test_planner_reentry_requires_new_id_receipt_and_header(self):
        command = self.runner.command(self.runner.hold((0.5, 0, 0.8), 0))
        command.trajectory_id = 7
        command.header.stamp = SimpleNamespace(to_sec=lambda: 1000.0)
        self.runner.data['planner_cmd'] = (command, self.clock.now)
        self.runner.send_goal = mock.Mock()
        with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic):
            sent, previous_id, stamp = self.runner.start_planner('D')
        self.assertEqual(previous_id, 7)
        self.assertEqual(self.runner.stage, 'WAIT_PLANNER_D')
        self.runner.send_goal.assert_called_once_with('D')
        self.runner.publishers['stop'].publish.assert_not_called()
        for trajectory_id, received, header_stamp, expected in (
                (7, sent + 0.1, 1000.1, False),
                (6, sent + 0.1, 1000.1, False),
                (8, sent - 0.1, 1000.1, False),
                (8, sent + 0.1, 999.9, False),
                (8, sent + 0.1, 1000.1, True)):
            with self.subTest(trajectory_id=trajectory_id, received=received, header_stamp=header_stamp):
                command.trajectory_id = trajectory_id
                command.header.stamp = SimpleNamespace(to_sec=lambda: header_stamp)
                self.runner.data['planner_cmd'] = (command, received)
                self.assertEqual(self.runner.planner_ready(sent, previous_id, stamp), expected)

    def test_mandatory_stop_is_not_reset_to_resume_planner(self):
        self.runner.stopped = True
        self.runner.send_goal = mock.Mock()
        with self.assertRaisesRegex(mission.FlightError, 'mandatory stop'):
            self.runner.start_planner('D')
        self.runner.send_goal.assert_not_called()

    def test_ground_hover_state_rejected_even_if_not_armed(self):
        self.runner.get = mock.Mock(side_effect=lambda key, *args: {
            'fcu': SimpleNamespace(connected=True, armed=False, mode='POSCTL'),
            'ctrl': SimpleNamespace(state=2), 'rc': SimpleNamespace(channels=[1500] * 4 + [2000] * 4),
        }[key])
        with self.assertRaisesRegex(mission.FlightError, 'MANUAL_CTRL'):
            self.runner.health(phase='ground')

    def test_stale_original_header_rejected_even_if_just_received(self):
        message = Command()
        message.header.stamp = SimpleNamespace(to_sec=lambda: 990)
        self.runner.data['planner_cmd'] = (message, self.clock.now)
        with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic):
            with self.assertRaises(mission.FlightError):
                self.runner.get('planner_cmd', 0.25, True)

    def test_missing_fcu_reports_topic_stage_and_timeout(self):
        with self.assertRaisesRegex(mission.FlightError, 'fcu 未收到') as caught:
            self.runner.get('fcu', self.runner.safety['fcu_timeout'])
        self.assertIn('话题=/mavros/state', str(caught.exception))
        self.assertIn('阶段=PREFLIGHT', str(caught.exception))
        self.assertIn('阈值=3.000s', str(caught.exception))

    def test_fresh_fcu_disconnect_mode_exit_or_disarm_is_not_graced(self):
        for connected, armed, mode, reason in ((False, True, 'OFFBOARD', '连接丢失'),
                                               (True, True, 'POSCTL', 'OFFBOARD'),
                                               (True, False, 'OFFBOARD', '已解锁')):
            with self.subTest(connected=connected, armed=armed, mode=mode):
                header = SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 1000))
                entries = dict(fcu=SimpleNamespace(connected=connected, armed=armed, mode=mode),
                               ctrl=SimpleNamespace(state=3, header=header),
                               rc=SimpleNamespace(channels=[1500] * 4 + [2000] * 4))
                self.runner.data = {key: (message, self.clock.now) for key, message in entries.items()}
                with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic):
                    with self.assertRaisesRegex(mission.FlightError, reason) as caught:
                        self.runner.health()
                self.assertFalse(caught.exception.land_allowed)

    def healthy_telemetry(self, phase='flight', landing=False):
        def header():
            return SimpleNamespace(frame_id='world', stamp=SimpleNamespace(to_sec=lambda: 1000.0))

        position = SimpleNamespace(x=0.0, y=0.0, z=0.8)
        lio_position = SimpleNamespace(x=0.0, y=0.0, z=0.8)
        velocity = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        entries = dict(
            fcu=SimpleNamespace(connected=True, armed=phase != 'ground', mode='OFFBOARD'),
            ctrl=SimpleNamespace(state=1 if phase == 'ground' else 4 if phase == 'takeoff' else 5 if landing else 3,
                                 header=header()),
            rc=SimpleNamespace(channels=[1500] * 4 + [2000] * 4),
            odom=SimpleNamespace(header=header(), pose=SimpleNamespace(pose=SimpleNamespace(
                position=position, orientation=orientation)), twist=SimpleNamespace(twist=SimpleNamespace(linear=velocity))),
            lio=SimpleNamespace(header=header(), pose=SimpleNamespace(pose=SimpleNamespace(
                position=lio_position, orientation=orientation))))
        self.runner.data = {key: (value, self.clock.now) for key, value in entries.items()}
        return entries

    def health_ticks(self, entries, ticks=30, step=0.05):
        with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic):
            for _ in range(ticks):
                self.runner.data = {key: (value, self.clock.now) for key, value in entries.items()}
                self.runner.health()
                self.clock.now += step

    def test_lio_stamp_latency_never_aborts(self):
        # 2026-10-29 起飞阶段因 0.73s 的 header 延迟误判中止；该判定已删除。
        entries = self.healthy_telemetry()
        for stamp in (999.27, 990.0, 0.0, 1002.0):
            with self.subTest(stamp=stamp):
                entries['lio'].header.stamp = SimpleNamespace(to_sec=lambda: stamp)
                self.health_ticks(entries, ticks=60)

    def test_lio_receive_timeout_tolerates_ten_hz_jitter(self):
        entries = self.healthy_telemetry()
        with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic):
            for age in (0.312, 0.346, 0.6):
                self.runner.data = {key: (value, self.clock.now) for key, value in entries.items()}
                self.runner.data['lio'] = (entries['lio'], self.clock.now - age)
                self.runner.health()
                self.clock.now += 0.05

    def test_lio_receive_timeout_aborts_on_real_dropout(self):
        entries = self.healthy_telemetry()
        with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic):
            self.runner.data = {key: (value, self.clock.now) for key, value in entries.items()}
            self.runner.data['lio'] = (entries['lio'], self.clock.now - 1.2)
            with self.assertRaisesRegex(mission.FlightError, 'lio 断更'):
                self.runner.health()

    def test_ekf_lio_divergence_aborts_after_grace(self):
        entries = self.healthy_telemetry()
        entries['lio'].pose.pose.position.x = 0.5
        with self.assertRaisesRegex(mission.FlightError, '水平偏差'):
            self.health_ticks(entries)

    def test_fence_aborts_outside_radius_only_when_enabled(self):
        entries = self.healthy_telemetry()
        for key in ('odom', 'lio'):
            entries[key].pose.pose.position.x = 2.0
        self.runner.origin = (0.0, 0.0, 0.8)
        self.health_ticks(entries, ticks=5)
        self.runner.safety['max_distance_from_origin'] = 1.0
        with self.assertRaisesRegex(mission.FlightError, '围栏'):
            self.health_ticks(entries, ticks=5)

    def test_zero_divergence_limit_disables_check(self):
        entries = self.healthy_telemetry()
        entries['lio'].pose.pose.position.x = 0.9
        self.runner.safety['max_ekf_lio_divergence'] = 0.0
        self.health_ticks(entries, ticks=40)

    def test_ekf_invalid_header_rejected(self):
        for stamp in (0, 990, 1001):
            with self.subTest(stamp=stamp):
                entries = self.healthy_telemetry()
                entries['odom'].header.stamp = SimpleNamespace(to_sec=lambda: stamp)
                with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic):
                    with self.assertRaisesRegex(mission.FlightError, 'odom 时间戳'):
                        self.runner.health()

    def test_rc_takeover_does_not_force_landing(self):
        self.runner.active = True
        self.runner.request_land = mock.Mock()
        self.runner.abort(mission.FlightError('飞手接管'))
        self.runner.request_land.assert_not_called()

    def test_planner_failure_can_request_landing_once(self):
        self.runner.health = mock.Mock()
        self.runner.graph = mock.Mock()
        self.runner.request_land()
        self.runner.request_land()
        self.assertEqual(self.runner.publishers['land'].publish.call_count, 1)
        self.assertEqual(self.runner.publishers['stop'].publish.call_count, 1)
        self.assertTrue(self.runner.land_sent)

    def test_invalid_command_never_published(self):
        for coordinate in (float('nan'), 10):
            message = self.runner.command(self.runner.hold((coordinate, 0, 0.8), 0))
            with self.assertRaises(mission.FlightError):
                self.runner.publish(message, (0, 0, 0.8))
        self.runner.publishers['command'].publish.assert_not_called()

    def test_graph_rejects_second_output_publisher(self):
        own = '/hybrid_mission'
        topics = self.config['topics']
        publishers = [[topics[key], [own]] for key in ('goal', 'land', 'stop')]
        publishers += [[topics['output_cmd'], [own, '/old_traj_server']]]
        self.runner.ros.get_master.return_value.getSystemState.return_value = (1, '', [publishers, [], []])
        with self.assertRaisesRegex(mission.FlightError, '争抢'):
            self.runner.graph()

    def test_landing_requires_ground_and_disarmed(self):
        self.runner.active = True
        self.runner.get = mock.Mock(side_effect=lambda key, *args: {
            'fcu': SimpleNamespace(connected=True, armed=False),
            'extended': SimpleNamespace(landed_state=1),
        }[key])
        self.runner.wait_landing()
        self.assertEqual(self.runner.stage, 'DONE')
        self.assertFalse(self.runner.active)
        self.runner.publishers['command'].publish.assert_not_called()

    def landing_stub(self, ctrl_state, height, origin=(10.0, -3.0, 0.034)):
        """wait_landing 的假数据源：FCU 已解锁、PX4 一直报 IN_AIR、实测离地 height。"""
        self.runner.active = True
        self.runner.origin = origin
        position = SimpleNamespace(x=origin[0], y=origin[1], z=origin[2] + height)
        self.runner.get = mock.Mock(side_effect=lambda key, *args: {
            'fcu': SimpleNamespace(connected=True, armed=True),
            'extended': SimpleNamespace(landed_state=2),
            'ctrl': SimpleNamespace(state=ctrl_state),
            'odom': SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=position))),
        }[key])
        self.runner.health = mock.Mock()

    def run_landing(self):
        with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic), \
                mock.patch.object(mission.time, 'sleep', self.clock.sleep):
            self.runner.wait_landing()

    def test_landing_handover_on_ground_finishes_without_error(self):
        """PX4Ctrl 交还飞手（PX4 一直不报 ON_GROUND）但实测已落地：收尾 DONE，提示杀桨。"""
        self.landing_stub(ctrl_state=1, height=0.03)
        self.run_landing()
        self.assertEqual(self.runner.stage, 'DONE')
        self.assertFalse(self.runner.active)
        self.runner.publishers['command'].publish.assert_not_called()
        self.runner.publishers['land'].publish.assert_not_called()
        self.assertIn('杀桨', self.runner.ros.logwarn.call_args.args[0])

    def test_landing_handover_in_air_still_requires_takeover(self):
        """飞手空中接管也会进 MANUAL_CTRL；离地还很高时必须报错要求接管。"""
        self.landing_stub(ctrl_state=1, height=0.50)
        with self.assertRaisesRegex(mission.FlightError, '交还飞手'):
            self.run_landing()
        self.assertNotEqual(self.runner.stage, 'DONE')

    def test_landing_timeout_on_ground_finishes_instead_of_error(self):
        """超时时实测已在地面：不再报"降落超时"，收尾并提示杀桨。"""
        self.landing_stub(ctrl_state=5, height=0.03)
        self.run_landing()
        self.assertEqual(self.runner.stage, 'DONE')
        self.assertLess(self.clock.now - 100.0, self.config['mission']['landing_timeout'] + 1.0)
        self.runner.health.assert_called()

    def test_landing_timeout_in_air_still_errors(self):
        self.landing_stub(ctrl_state=5, height=0.50)
        with self.assertRaisesRegex(mission.FlightError, '降落超时'):
            self.run_landing()

    def test_read_only_graph_requires_no_flight_publishers(self):
        self.runner.read_only = True
        topics = self.config['topics']
        publishers = [[topics['planner_cmd'], [self.config['traj_server_node']]]]
        subscribers = [[topics[key], [node]] for key, node in (
            ('output_cmd', self.config['controller_node']), ('land', self.config['controller_node']),
            ('goal', self.config['planner_node']), ('stop', self.config['planner_node']))]
        self.runner.ros.get_master.return_value.getSystemState.return_value = (1, '', [publishers, subscribers, []])
        self.runner.graph()

    def test_unacknowledged_land_is_not_retried(self):
        self.runner.land_sent = True
        self.runner.health = mock.Mock()
        self.runner.get = mock.Mock(side_effect=lambda key, *args: {
            'fcu': SimpleNamespace(connected=True, armed=True),
            'extended': SimpleNamespace(landed_state=2),
            'ctrl': SimpleNamespace(state=3),
        }[key])
        with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic), mock.patch.object(mission.time, 'sleep', self.clock.sleep):
            with self.assertRaisesRegex(mission.FlightError, '未确认 AUTO_LAND'):
                self.runner.wait_landing()
        self.runner.publishers['land'].publish.assert_not_called()
        self.runner.publishers['command'].publish.assert_not_called()

    def simulate_full_mission(self, respond_c=True, unfinished_c=False, hazard=None):
        runner, clock = self.runner, self.clock
        self.simulate_takeoff(origin=(10, -3, 0.034))
        takeoff_health, takeoff_get = runner.health, runner.get
        runner.config['points']['B']['yaw_deg'] = 45.0
        runner.config['points']['D']['yaw_deg'] = -90.0
        state = dict(position=tuple(mission.hover_point(runner.config)['xyz']), yaw=0.0)
        trace = []
        goals = []
        runner.graph = mock.Mock()
        runner.wait_landing = mock.Mock()
        planner_command = [None]
        next_id = [0]
        unfinished = [False]
        # 模拟 Planner 轨迹：从发目标时的实际位置出发，匀速飞向目标；旧轨迹在新目标
        # 到达前继续推进，这样桥接阶段不会瞬移，跟踪误差也不会凭空超限。
        trajectory = dict(start=None, target=None, started=0.0, duration=0.0)

        def planner_position(now):
            if trajectory['target'] is None:
                return None
            progress = min(1.0, max(0.0, (now - trajectory['started']) / trajectory['duration'])) \
                if trajectory['duration'] > 0 else 1.0
            return tuple(begin + (end - begin) * progress
                         for begin, end in zip(trajectory['start'], trajectory['target']))

        def refresh_planner_command(now):
            position = planner_position(now)
            if position is None:
                return
            planner_command[0] = runner.command(runner.hold(position, 0.0))
            planner_command[0].trajectory_id = next_id[0]
            if unfinished[0]:
                planner_command[0].jerk.x = 0.01

        def health(**kwargs):
            if kwargs.get('phase') == 'takeoff':
                return takeoff_health(**kwargs)
            refresh_planner_command(clock.now)
            if planner_command[0] is not None:
                planner_command[0].header.stamp = SimpleNamespace(to_sec=lambda: 1000.0)
                runner.data['planner_cmd'] = (planner_command[0], clock.now)
            if hazard:
                # 只在悬停于该航点上方时喂检测结果，模拟相机始终能看到下方的危险品。
                near = math.dist(state['position'], tuple(runner.config['points'][hazard]['xyz'])) <= 0.1
                if near:
                    runner.receive(SimpleNamespace(data=json.dumps(dict(
                        count=1, detections=[dict(label='危险品标识', confidence=0.9)]))), 'detections')
            return state['position'], (0, 0, 0), state['yaw']

        def send_goal(label):
            self.assertGreaterEqual(clock.now, 104)
            runner.publishers['stop'].publish.assert_not_called()
            goals.append(label)
            if label == 'C' and not respond_c:
                return
            next_id[0] += 1
            target = tuple(runner.config['points'][label]['xyz'])
            trajectory.update(start=tuple(state['position']), target=target, started=clock.now,
                              duration=max(1e-6, math.dist(state['position'], target) / 0.2))
            unfinished[0] = bool(label == 'C' and unfinished_c)
            refresh_planner_command(clock.now)

        def get_command(key, *args):
            if key != 'planner_cmd':
                return takeoff_get(key, *args)
            self.assertEqual(key, 'planner_cmd')
            self.assertIn(runner.stage, ('PLANNER_B', 'PLANNER_C', 'PLANNER_E', 'PLANNER_F', 'PLANNER_I'),
                          'Planner 数据绝不能在直控段再次读取')
            return planner_command[0]

        def receive_command(command):
            self.assertNotIn(runner.stage, ('TAKEOFF', 'TAKEOFF_HOVER'))
            runner.publishers['stop'].publish.assert_not_called()
            state['position'], state['yaw'] = mission.vector(command.position), command.yaw
            trace.append((clock.now, runner.stage, copy.deepcopy(command)))

        runner.send_goal = send_goal
        runner.health = mock.Mock(side_effect=health)
        runner.get = get_command
        runner.publishers['command'].publish.side_effect = receive_command
        return state, trace, goals

    def test_full_sequence_closes_gate_reenters_planner_and_hovers_three_seconds(self):
        runner, clock = self.runner, self.clock
        state, trace, goals = self.simulate_full_mission()
        with mock.patch.object(mission.time, 'monotonic', clock.monotonic), mock.patch.object(mission.time, 'sleep', clock.sleep):
            runner.run()
        phases = [entry[1] for entry in trace]
        self.assertIn('PLANNER_B', phases)
        self.assertIn('PLANNER_C', phases)
        self.assertIn('DIRECT', phases)
        self.assertIn('DWELL', phases)
        self.assertIn('PLANNER_E', phases)
        self.assertIn('PLANNER_F', phases)
        self.assertIn('PLANNER_I', phases)
        self.assertIn('HOLD', phases)
        self.assertNotIn('DESCEND', phases, '默认 landing/approach_height=0：悬停到位后直接 LAND')
        self.assertEqual(goals, ['B', 'C', 'E', 'F', 'I'])
        f_index = phases.index('PLANNER_F')
        f_start = trace[f_index][0]
        g_start = next(entry[0] for entry in trace[f_index:] if entry[1] == 'DIRECT')
        self.assertGreaterEqual(g_start - f_start, self.config['mission']['arrival_dwell'],
                                'F 到点后应连续稳定悬停 arrival_dwell 再切直控')
        self.assertEqual(runner.stage, 'LANDING')
        self.assertEqual(runner.publishers['stop'].publish.call_count, 1)
        self.assertEqual([entry.args[0].takeoff_land_cmd for entry in runner.publishers['land'].publish.call_args_list],
                         [Landing.TAKEOFF, Landing.LAND])
        runner.preflight.assert_called_once()
        runner.wait_landing.assert_called_once()
        b_start = next(entry[0] for entry in trace if entry[1] == 'PLANNER_B')
        c_start = next(entry[0] for entry in trace if entry[1] == 'WAIT_PLANNER_C')
        self.assertGreaterEqual(c_start - b_start, self.config['mission']['arrival_dwell'],
                                'B 到点后应连续稳定悬停 arrival_dwell 秒，再从静止状态规划 C')
        hover_start = next(entry[0] for entry in trace if entry[1] == 'HOLD')
        self.assertGreaterEqual(clock.now - hover_start, self.config['mission']['hover_seconds'],
                                'I 点必须先连续稳定悬停 hover_seconds 才允许降落')
        self.assertEqual(state['position'], tuple(runner.config['points']['I']['xyz']),
                         '默认不进近：发 LAND 时飞机就在 I 点，PX4Ctrl 以该位姿为落点参考')
        self.assertAlmostEqual(state['yaw'], math.pi)
        tolerance = math.radians(self.config['safety']['yaw_tolerance_deg'])
        planner_outputs = [entry[2] for entry in trace if entry[1] == 'PLANNER_B']
        # B 是经过点：一旦进入 pass_radius 就切到 C，仿真里只经过一个控制周期，
        # 偏航还来不及转到 B 的 45°。这里只要求指令朝目标偏航方向推进且不超速率限制。
        self.assertTrue(all(0.0 <= mission.wrap(command.yaw) <= math.pi / 4 + 1e-9 for command in planner_outputs))
        self.assertLessEqual(max(abs(command.yaw_dot) for command in planner_outputs),
                             math.radians(self.config['direct']['max_yaw_rate_deg']) + 1e-8)
        c_outputs = [entry[2] for entry in trace if entry[1] == 'PLANNER_C']
        self.assertLessEqual(math.dist(mission.vector(c_outputs[-1].position),
                                       tuple(runner.config['points']['C']['xyz'])),
                             self.config['safety']['position_tolerance'])
        self.assertLessEqual(abs(mission.wrap(c_outputs[-1].yaw - math.pi / 2)), tolerance)
        direct_outputs = [entry[2] for entry in trace if entry[1] == 'DIRECT']
        self.assertLessEqual(max(abs(command.yaw_dot) for command in direct_outputs),
                             math.radians(self.config['direct']['max_yaw_rate_deg']) + 1e-8)
        # C→D、F→G 直接到点悬停；G→H 之后 H 只是经过点，不再进入 DWELL。
        visited = {mission.vector(entry[2].position) for entry in trace if entry[1] == 'DWELL'}
        self.assertEqual(visited, {tuple(runner.config['points'][label]['xyz']) for label in ('D', 'G')})
        d_outputs = [entry[2] for entry in trace if entry[1] == 'DWELL']
        self.assertLessEqual(abs(mission.wrap(d_outputs[0].yaw + math.pi / 2)), tolerance,
                             'D 的直控段结束时应保持模拟配置的 -90°')
        # H 是经过点：进入 pass_radius 就发 I 目标，且切换那一刻还没飞到 H。
        i_start = next(entry[0] for entry in trace if entry[1] == 'WAIT_PLANNER_I')
        i_position = next(entry[2] for entry in trace if entry[1] == 'WAIT_PLANNER_I')
        h_xyz = tuple(runner.config['points']['H']['xyz'])
        self.assertLessEqual(math.dist(mission.vector(i_position.position), h_xyz),
                             self.config['mission']['pass_radius'] + 1e-6)
        self.assertNotIn('DWELL', [entry[1] for entry in trace if entry[0] >= i_start])
        for label in ('E', 'F', 'I'):
            outputs = [entry[2] for entry in trace if entry[1] == 'PLANNER_' + label]
            self.assertLessEqual(math.dist(mission.vector(outputs[-1].position),
                                           tuple(runner.config['points'][label]['xyz'])),
                                 self.config['safety']['position_tolerance'],
                                 '%s 段最后一个 Planner 指令应在目标点附近' % label)

    def test_landing_approach_is_off_by_default(self):
        self.assertEqual(self.config['landing']['approach_height'], 0.0,
                         '交接结论：落点精度取决于发 LAND 时的实测位姿，不取决于触发高度')
        runner, clock = self.runner, self.clock
        state, trace, goals = self.simulate_full_mission()
        with mock.patch.object(mission.time, 'monotonic', clock.monotonic), mock.patch.object(mission.time, 'sleep', clock.sleep):
            runner.run()
        self.assertNotIn('DESCEND', [entry[1] for entry in trace], 'approach_height=0 时不应有进近段')
        self.assertEqual(runner.stage, 'LANDING')
        runner.wait_landing.assert_called_once()

    def test_landing_waits_for_tight_reference_then_lands_anyway(self):
        runner, clock = self.runner, self.clock
        state, trace, goals = self.simulate_full_mission()
        landing = runner.config['landing']
        # 恒定 6cm 水平残差：满足 safety/position_tolerance(0.10m)，但不满足落点参考判据(0.03m)。
        original = runner.publishers['command'].publish.side_effect

        def publish(command):
            original(command)
            if runner.stage == 'HOLD':
                state['position'] = (state['position'][0] + 0.06, state['position'][1], state['position'][2])
        runner.publishers['command'].publish.side_effect = publish
        with mock.patch.object(mission.time, 'monotonic', clock.monotonic), mock.patch.object(mission.time, 'sleep', clock.sleep):
            runner.run()
        hover_start = next(entry[0] for entry in trace if entry[1] == 'HOLD')
        self.assertGreaterEqual(clock.now - hover_start,
                                self.config['mission']['hover_seconds'] + landing['target_timeout'] - 1e-6,
                                '落点参考没收敛时应等满 target_timeout 再降落，而不是立刻降落')
        self.assertEqual(runner.stage, 'LANDING', '收不紧也必须降落，不能停在空中等')
        self.assertEqual(runner.publishers['land'].publish.call_args.args[0].takeoff_land_cmd, Landing.LAND)
        runner.wait_landing.assert_called_once()

    def test_landing_reference_converged_lands_right_after_hover(self):
        runner, clock = self.runner, self.clock
        state, trace, goals = self.simulate_full_mission()
        landing = runner.config['landing']
        with mock.patch.object(mission.time, 'monotonic', clock.monotonic), mock.patch.object(mission.time, 'sleep', clock.sleep):
            runner.run()
        hover_start = next(entry[0] for entry in trace if entry[1] == 'HOLD')
        self.assertLess(clock.now - hover_start,
                        self.config['mission']['hover_seconds'] + landing['target_timeout'],
                        '落点参考已经收敛时不应空等 target_timeout')

    def test_landing_approach_stops_when_ground_is_higher_than_expected(self):
        runner, clock = self.runner, self.clock
        state, trace, goals = self.simulate_full_mission()
        runner.config['landing']['approach_height'] = 0.10
        # 地面比 EKF 记录的起飞原点高 0.12m：实测高度提前低于目标，必须就地交给 PX4Ctrl，不能继续下压。
        real_ground = [runner.origin[2] + 0.12]
        original = runner.publishers['command'].publish.side_effect

        def publish(command):
            original(command)
            if runner.stage == 'DESCEND':
                state['position'] = (state['position'][0], state['position'][1],
                                     max(state['position'][2], real_ground[0]))
        runner.publishers['command'].publish.side_effect = publish
        with mock.patch.object(mission.time, 'monotonic', clock.monotonic), mock.patch.object(mission.time, 'sleep', clock.sleep):
            runner.run()
        self.assertEqual(runner.stage, 'LANDING')
        self.assertGreaterEqual(state['position'][2], real_ground[0] - 1e-9,
                                '进近不得把飞机压到实际地面以下')
        runner.wait_landing.assert_called_once()

    def test_old_b_hold_does_not_acknowledge_c_and_times_out_safely(self):
        runner, clock = self.runner, self.clock
        state, trace, goals = self.simulate_full_mission(respond_c=False)
        with mock.patch.object(mission.time, 'monotonic', clock.monotonic), mock.patch.object(mission.time, 'sleep', clock.sleep):
            with self.assertRaisesRegex(mission.FlightError, '发送 C 点后未收到新轨迹 ID') as caught:
                runner.run()
        self.assertTrue(caught.exception.land_allowed)
        self.assertEqual(goals, ['B', 'C'])
        self.assertEqual(runner.stage, 'WAIT_PLANNER_C')
        self.assertEqual(state['position'], tuple(runner.config['points']['B']['xyz']))
        self.assertNotIn('PLANNER_C', [entry[1] for entry in trace])
        runner.publishers['stop'].publish.assert_not_called()
        with mock.patch.object(mission.time, 'monotonic', clock.monotonic):
            runner.abort(caught.exception)
        self.assertEqual(runner.stage, 'LANDING')
        self.assertEqual(runner.publishers['stop'].publish.call_count, 1)
        self.assertEqual(runner.publishers['land'].publish.call_args.args[0].takeoff_land_cmd, Landing.LAND)

    def test_direct_handoff_requires_planner_c_endpoint_hold(self):
        runner, clock = self.runner, self.clock
        _, trace, goals = self.simulate_full_mission(unfinished_c=True)
        # B 段包含 arrival_dwell(3s) 悬停，5s 会在 B 就先超时，测不到 C；留足 B 的时间。
        runner.settings['planner_timeout'] = 10.0
        with mock.patch.object(mission.time, 'monotonic', clock.monotonic), mock.patch.object(mission.time, 'sleep', clock.sleep):
            with self.assertRaisesRegex(mission.FlightError, '到达 C 超时'):
                runner.run()
        self.assertEqual(goals, ['B', 'C'])
        self.assertNotIn('DIRECT', [entry[1] for entry in trace])
        runner.publishers['stop'].publish.assert_not_called()


class HazardDrop(unittest.TestCase):
    """危险品投放：实机配置默认开启（可用 --no-hazard-drop 关）；只在 C/D 稳定悬停且稳定检测到时下降并推舵机。"""

    setUp = RuntimeRules.setUp
    simulate_takeoff = RuntimeRules.simulate_takeoff
    simulate_full_mission = RuntimeRules.simulate_full_mission

    def run_mission(self):
        runner, clock = self.runner, self.clock
        with mock.patch.object(mission.time, 'monotonic', clock.monotonic), \
                mock.patch.object(mission.time, 'sleep', clock.sleep):
            runner.run()

    def drop_config(self):
        return self.runner.config['hazard_drop']

    def enable_drop(self, **changes):
        # simulate_takeoff 会用 resolve_config(source_config) 重建 runner.config，
        # 所以三份配置都要改，否则开关会在起飞时被 deepcopy 覆盖回 false。
        for config in (self.config, self.runner.config, self.runner.source_config):
            config['hazard_drop']['enabled'] = True
            config['hazard_drop'].update(changes)

    def test_shipped_config_enables_drop(self):
        self.assertTrue(self.config['hazard_drop']['enabled'],
                        '2026-10-31 起实机任务默认启用投放；临时关闭用 --no-hazard-drop')

    def test_disabled_ignores_detections(self):
        runner, clock = self.runner, self.clock
        # 显式改配置关闭（等价于 --no-hazard-drop），而不是依赖出厂默认值。
        for config in (self.config, self.runner.config, self.runner.source_config):
            config['hazard_drop']['enabled'] = False
        runner.begin_push = mock.Mock(return_value=dict(ok=True, message='2500 us'))
        state, trace, _ = self.simulate_full_mission(hazard='C')
        self.run_mission()
        phases = [entry[1] for entry in trace]
        self.assertNotIn('DROP_DESCEND', phases)
        self.assertNotIn('DROP_PUSH', phases)
        runner.begin_push.assert_not_called()
        self.assertEqual(runner.stage, 'LANDING')

    def test_detection_at_c_descends_pushes_and_climbs_back(self):
        runner, clock = self.runner, self.clock
        self.enable_drop()
        runner.begin_push = mock.Mock(return_value=dict(ok=True, message='2500 us'))
        state, trace, goals = self.simulate_full_mission(hazard='C')
        self.run_mission()
        phases = [entry[1] for entry in trace]
        self.assertIn('DROP_DESCEND', phases)
        self.assertIn('DROP_PUSH', phases)
        self.assertIn('DROP_ASCEND', phases, '投放后必须爬回航点高度再继续任务')
        runner.begin_push.assert_called_once()
        # 下降目标 = 起飞原点 + hazard_drop/height，不压到地面以下。
        floor = runner.origin[2] + self.drop_config()['height']
        lowest = min(mission.vector(entry[2].position)[2] for entry in trace
                     if entry[1] in ('DROP_DESCEND', 'DROP_PUSH'))
        self.assertAlmostEqual(lowest, floor, places=2)
        # 爬回后 C→D 直控段从 C 的高度开始，而不是从投放低空开始。
        ascend_end = max(index for index, entry in enumerate(trace) if entry[1] == 'DROP_ASCEND')
        resumed = next(entry for entry in trace[ascend_end:] if entry[1] == 'DIRECT')
        self.assertAlmostEqual(mission.vector(resumed[2].position)[2],
                               self.config['points']['C']['xyz'][2] + runner.origin[2], places=2)
        # 投放仍然完成整条任务并降落。
        self.assertIn('PLANNER_E', phases)
        self.assertIn('PLANNER_I', phases)
        self.assertEqual(runner.stage, 'LANDING')
        self.assertEqual(goals, ['B', 'C', 'E', 'F', 'I'])
        # 必须先连续稳定悬停 hover_seconds 才允许下降。
        arrival = next(entry[0] for entry in trace
                       if entry[1] == 'PLANNER_C'
                       and math.dist(mission.vector(entry[2].position),
                                     tuple(runner.config['points']['C']['xyz'])) <= 0.1)
        drop_start = next(entry[0] for entry in trace if entry[1] == 'DROP_DESCEND')
        self.assertGreaterEqual(drop_start - arrival,
                                self.drop_config()['hover_seconds'] - 1e-6,
                                '悬停不足 hover_seconds 就下降了')

    def test_detection_at_d_drops_from_direct_dwell(self):
        runner, clock = self.runner, self.clock
        self.enable_drop()
        runner.begin_push = mock.Mock(return_value=dict(ok=True, message='2500 us'))
        state, trace, goals = self.simulate_full_mission(hazard='D')
        self.run_mission()
        phases = [entry[1] for entry in trace]
        self.assertIn('DWELL', phases)
        self.assertIn('DROP_DESCEND', phases, 'D 是直控段到点后的 DWELL，也必须支持投放')
        runner.begin_push.assert_called_once()
        self.assertEqual(goals, ['B', 'C', 'E', 'F', 'I'])
        self.assertEqual(runner.stage, 'LANDING')

    def test_no_detection_waits_then_continues_without_dropping(self):
        runner, clock = self.runner, self.clock
        self.enable_drop()
        runner.begin_push = mock.Mock(return_value=dict(ok=True, message='2500 us'))
        state, trace, goals = self.simulate_full_mission()
        self.run_mission()
        phases = [entry[1] for entry in trace]
        self.assertNotIn('DROP_DESCEND', phases)
        runner.begin_push.assert_not_called()
        self.assertEqual(goals, ['B', 'C', 'E', 'F', 'I'], '检测不到时必须继续整条任务')
        self.assertEqual(runner.stage, 'LANDING')
        # C 点按 wait_timeout 等满后继续，而不是立刻离开或永久悬停。
        first_c = next(entry[0] for entry in trace if entry[1] == 'PLANNER_C')
        after_c = next(entry[0] for entry in trace if entry[0] > first_c and entry[1] == 'DIRECT')
        self.assertGreaterEqual(after_c - first_c, self.drop_config()['wait_timeout'])

    def feed(self, detections, width=640, height=480):
        payload = dict(image_width=width, image_height=height,
                       count=len(detections), detections=detections)
        self.runner.receive(SimpleNamespace(data=json.dumps(payload)), 'detections')

    def last_vote(self):
        return list(self.runner.hazard_votes)[-1][1]

    def test_center_target_and_frame_vote_decide_hazard(self):
        runner = self.runner
        values = runner.drop_settings()
        centered = dict(label='危险品标识', confidence=0.9, cx=330.0, cy=230.0,
                        x1=300.0, y1=200.0, x2=360.0, y2=260.0)
        edge = dict(label='危险品标识', confidence=0.9, cx=40.0, cy=40.0,
                    x1=10.0, y1=10.0, x2=70.0, y2=70.0)
        person = dict(label='救助人员', confidence=0.95, cx=320.0, cy=240.0,
                      x1=290.0, y1=210.0, x2=350.0, y2=270.0)
        with mock.patch.object(mission.time, 'monotonic', self.clock.monotonic):
            # 两个框时只认离图像中心最近的那个：中心是救助人员 → 不算危险品帧。
            self.feed([person, edge])
            self.assertFalse(self.last_vote(), '边缘的危险品框不应压过中心目标')
            # 危险品在中心、别的框在边缘 → 算命中。
            self.feed([centered, dict(person, cx=40.0, cy=40.0)])
            self.assertTrue(self.last_vote(), '中心目标才是投放对象')
            # 中心危险品但置信度不足 → 不命中。
            self.feed([dict(centered, confidence=0.2)])
            self.assertFalse(self.last_vote(), '置信度不足不算命中')
            # 坏消息按未命中参与投票，不抛异常。
            runner.receive(SimpleNamespace(data='not json'), 'detections')
            self.assertFalse(self.last_vote(), '坏消息按未检测处理')

            # 投票：帧数不足不确认；比例不足不确认；比例达标才确认。
            frames = values['vote_frames']
            runner.hazard_votes.clear()
            for _ in range(frames - 1):
                self.feed([centered])
            self.assertFalse(runner.hazard_stable(self.clock.now, values), '帧数不足还不能确认')
            for _ in range(int(frames * (1.0 - values['vote_ratio'])) + 1):
                self.feed([person])
            self.assertFalse(runner.hazard_stable(self.clock.now, values), '命中比例不足不能确认')
            for _ in range(frames):
                self.feed([centered])
            self.assertTrue(runner.hazard_stable(self.clock.now, values), '命中比例达标后确认')
            # 检测断更（超过 max_age 没有新帧）→ 不能确认。
            self.assertFalse(runner.hazard_stable(self.clock.now + values['max_age'] * 2.0, values))

    def test_once_drops_at_most_one_point(self):
        runner = self.runner
        self.enable_drop()
        frames = runner.drop_settings()['vote_frames']
        runner.hazard_votes = [(104.0, True)] * frames
        runner.hazard_last = 104.9
        self.assertEqual(runner.drop_decision(105.0, 'C', 99.0), 'drop')
        runner.drop_done = True
        self.assertEqual(runner.drop_decision(105.0, 'D', 99.0), 'continue',
                         'once=true 时第二次到点不再投放')
        self.drop_config()['once'] = False
        self.assertEqual(runner.drop_decision(105.0, 'D', 99.0), 'drop')

    def test_drop_settings_reject_unsafe_values(self):
        runner = self.runner
        self.drop_config()['points'] = ['C', 'E']
        with self.assertRaisesRegex(mission.FlightError, '只允许 C/D'):
            runner.drop_settings()
        self.drop_config()['points'] = ['C', 'D']
        self.drop_config()['height'] = self.config['takeoff']['height']
        with self.assertRaisesRegex(mission.FlightError, 'height'):
            runner.drop_settings()
        self.drop_config()['height'] = 0.15
        self.drop_config()['speed'] = self.config['direct']['max_speed'] + 0.1
        with self.assertRaisesRegex(mission.FlightError, 'speed'):
            runner.drop_settings()

    def test_servo_push_reports_failure_without_raising(self):
        try:
            from std_srvs.srv import Trigger  # noqa: F401
        except ImportError:
            self.skipTest('std_srvs 不可用')
        runner = self.runner
        runner.ros.wait_for_service.side_effect = RuntimeError('service down')
        ok, message = runner.servo_push()
        self.assertFalse(ok)
        self.assertIn('service down', message)
        runner.ros.wait_for_service.side_effect = None
        runner.ros.ServiceProxy.return_value.return_value = SimpleNamespace(success=False, message='PX4 未就绪')
        ok, message = runner.servo_push()
        self.assertFalse(ok, '服务拒绝时返回失败而不是抛异常')
        self.assertEqual(message, 'PX4 未就绪')

    def test_begin_push_returns_immediately_and_backfills_outcome(self):
        import time as wall
        runner = self.runner
        runner.servo_push = mock.Mock(return_value=(True, '2500 us'))
        started = wall.monotonic()
        outcome = runner.begin_push()
        self.assertLess(wall.monotonic() - started, 0.5, '推舵机不能同步阻塞位置指令循环')
        deadline = wall.monotonic() + 2.0
        while outcome['ok'] is None and wall.monotonic() < deadline:
            wall.sleep(0.01)
        self.assertTrue(outcome['ok'], '后台线程应把服务结果回填到 outcome')


if __name__ == '__main__':
    unittest.main()
