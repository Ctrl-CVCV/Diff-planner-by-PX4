#!/usr/bin/env python3
"""Offline tests only. No ROS master, subscriptions, arming or flight commands."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

path = Path(__file__).resolve().parents[1] / 'takeoff_then_diff_planner.py'
spec = importlib.util.spec_from_file_location('sequence', path)
s = importlib.util.module_from_spec(spec)
spec.loader.exec_module(s)


def odom(z=0, speed=0):
    return NS(pose=NS(pose=NS(position=NS(x=0., y=0., z=z),
                             orientation=NS(x=0., y=0., z=0., w=1.))),
              twist=NS(twist=NS(linear=NS(x=speed, y=0., z=0.))),
              header=NS(stamp=NS(to_sec=lambda: 100.)))


class Rules(unittest.TestCase):
    def test_rc_centered(self):
        self.assertEqual(s.rc_error([1500]*4+[2000, 2000, 1500, 1000]), '')

    def test_low_throttle_rejected(self):
        self.assertIn('居中', s.rc_error([1500, 1500, 1000, 1500, 2000, 2000, 1500, 1000]))

    def test_bad_switch_rejected(self):
        self.assertIn('CH5/CH6', s.rc_error([1500]*4+[1750, 2000, 1500, 1000]))

    def test_rc_missing_channels(self):
        self.assertTrue(s.rc_error([1500]*4))

    def test_goal_is_world_offset_and_keeps_height(self):
        self.assertEqual(s.goal_for_hover((4., 5., .8), .5, -.2), (4.5, 4.8, .8))

    def test_vertical_nominal(self):
        s.check_vertical(.8, .1, 1.2)

    def test_vertical_at_ceiling_rejected(self):
        with self.assertRaises(s.SafetyError):
            s.check_vertical(1.2, .1, 1.2)

    def test_vertical_margin_rejected(self):
        with self.assertRaises(s.SafetyError):
            s.check_vertical(1.1, .1, 1.2)

    def test_vertical_origin_offset(self):
        s.check_vertical(.55, -.15, .95)

    def test_nan_rejected(self):
        with self.assertRaises(s.SafetyError):
            s.check_vertical(float('nan'), .1, 1.2)

    def test_hover_stable(self):
        self.assertTrue(s.hover_ok((.05, .03, .8), (0, 0, .01), (0, 0, 0), .8))

    def test_hover_drift(self):
        self.assertFalse(s.hover_ok((.3, 0, .8), (0, 0, 0), (0, 0, 0), .8))

    def test_hover_moving(self):
        self.assertFalse(s.hover_ok((0, 0, .8), (.2, 0, 0), (0, 0, 0), .8))

    def test_hover_wrong_height(self):
        self.assertFalse(s.hover_ok((0, 0, .5), (0, 0, 0), (0, 0, 0), .8))


class Sequence(unittest.TestCase):
    def make(self):
        obj = s.Sequencer.__new__(s.Sequencer)
        obj.args = s.parser().parse_args([])
        obj.ros = Mock()
        obj.mon = Mock()
        obj.sleep = Mock()
        obj.height = .8
        obj.preflight = Mock(return_value=(0, 0, 0))
        obj.flight_requested = False
        obj.goal_sent = False
        obj.landing_requested = False
        return obj

    def test_default_is_check_only_no_publishers(self):
        obj = self.make()
        obj.run()
        obj.preflight.assert_called_once()
        obj.ros.Publisher.assert_not_called()
        self.assertFalse(obj.flight_requested)

    def test_execution_requires_verified_map(self):
        obj = self.make()
        obj.args.execute = True
        with self.assertRaisesRegex(s.SafetyError, '地图未确认'):
            obj.run()
        obj.ros.Publisher.assert_not_called()

    def test_cancelled_confirmation_never_publishes(self):
        obj = self.make()
        obj.args.execute = obj.args.map_verified = True
        with patch('builtins.input', return_value='no'):
            with self.assertRaisesRegex(s.SafetyError, '取消'):
                obj.run()
        obj.ros.Publisher.assert_not_called()

    def test_unexpected_command_aborts_hover_wait(self):
        obj = self.make()
        obj.live = Mock(return_value=(NS(armed=True, mode='OFFBOARD'), odom(.8), NS(state=s.HOVER)))
        obj.mon.quiet.return_value = False
        sent = s.time.monotonic()
        with self.assertRaisesRegex(s.SafetyError, '提前收到'):
            obj.wait_hover((0, 0, 0), sent)

    def test_lost_offboard_aborts_hover_wait(self):
        obj = self.make()
        obj.live = Mock(return_value=(NS(armed=True, mode='POSCTL'), odom(.8), NS(state=s.HOVER)))
        obj.mon.quiet.return_value = True
        with self.assertRaisesRegex(s.SafetyError, 'OFFBOARD'):
            obj.wait_hover((0, 0, 0), s.time.monotonic())

    def test_initial_state_feedback_lag_waits_then_confirms_hover(self):
        obj = self.make()
        obj.live = Mock()
        obj.mon.quiet.return_value = True
        obj.mon.get.return_value = NS(landed_state=2)
        clock = [10.]
        obj.sleep.side_effect = lambda duration: clock.__setitem__(0, clock[0] + .25)
        obj.live.side_effect = lambda: (
            NS(armed=clock[0] >= 10.75, mode='OFFBOARD'),
            odom(.8 if clock[0] >= 11. else 0.),
            NS(state=s.HOVER if clock[0] >= 11. else s.TAKEOFF))
        with patch.object(s.time, 'monotonic', side_effect=lambda: clock[0]):
            result = obj.wait_hover((0, 0, 0), 10.)
        self.assertGreaterEqual(clock[0], 14.)
        self.assertEqual(result.pose.pose.position.z, .8)
        obj.ros.logwarn.assert_called_once()
        obj.ros.Publisher.assert_not_called()

    def test_initial_mode_feedback_lag_waits_then_confirms_hover(self):
        obj = self.make()
        obj.live = Mock()
        obj.mon.quiet.return_value = True
        obj.mon.get.return_value = NS(landed_state=1)
        clock = [10.]
        obj.sleep.side_effect = lambda duration: clock.__setitem__(0, clock[0] + .25)
        obj.live.side_effect = lambda: (
            NS(armed=clock[0] >= 10.5, mode='OFFBOARD' if clock[0] >= 10.75 else 'POSCTL'),
            odom(.8 if clock[0] >= 11. else 0.),
            NS(state=s.HOVER if clock[0] >= 11. else s.TAKEOFF))
        with patch.object(s.time, 'monotonic', side_effect=lambda: clock[0]):
            obj.wait_hover((0, 0, 0), 10.)
        self.assertGreaterEqual(clock[0], 14.)

    def test_initial_state_confirmation_is_bounded(self):
        obj = self.make()
        obj.mon.quiet.return_value = True
        obj.mon.get.return_value = NS(landed_state=1)
        obj.live = Mock(return_value=(NS(armed=False, mode='OFFBOARD'), odom(), NS(state=s.TAKEOFF)))
        clock = [10.]
        obj.sleep.side_effect = lambda duration: clock.__setitem__(0, clock[0] + .25)
        with patch.object(s.time, 'monotonic', side_effect=lambda: clock[0]):
            with self.assertRaisesRegex(s.SafetyError, '确认超时'):
                obj.wait_hover((0, 0, 0), 10.)
        self.assertEqual(clock[0], 10. + s.FCU_CONFIRM_TIMEOUT)
        obj.ros.Publisher.assert_not_called()
        self.assertFalse(obj.goal_sent)

    def test_disarming_after_confirmation_aborts_immediately(self):
        obj = self.make()
        obj.mon.quiet.return_value = True
        obj.live = Mock(side_effect=[
            (NS(armed=True, mode='OFFBOARD'), odom(), NS(state=s.TAKEOFF)),
            (NS(armed=False, mode='OFFBOARD'), odom(), NS(state=s.TAKEOFF)),
        ])
        clock = [10.]
        obj.sleep.side_effect = lambda duration: clock.__setitem__(0, clock[0] + .25)
        with patch.object(s.time, 'monotonic', side_effect=lambda: clock[0]):
            with self.assertRaisesRegex(s.SafetyError, '未保持armed'):
                obj.wait_hover((0, 0, 0), 10.)
        self.assertEqual(clock[0], 10.25)
        obj.ros.logwarn.assert_not_called()

    def test_mode_loss_after_confirmation_aborts_immediately(self):
        obj = self.make()
        obj.mon.quiet.return_value = True
        obj.live = Mock(side_effect=[
            (NS(armed=True, mode='OFFBOARD'), odom(), NS(state=s.TAKEOFF)),
            (NS(armed=True, mode='POSCTL'), odom(), NS(state=s.TAKEOFF)),
        ])
        clock = [10.]
        obj.sleep.side_effect = lambda duration: clock.__setitem__(0, clock[0] + .25)
        with patch.object(s.time, 'monotonic', side_effect=lambda: clock[0]):
            with self.assertRaisesRegex(s.SafetyError, 'OFFBOARD'):
                obj.wait_hover((0, 0, 0), 10.)
        self.assertEqual(clock[0], 10.25)

    def test_unconfirmed_state_while_moving_aborts(self):
        obj = self.make()
        obj.mon.quiet.return_value = True
        obj.mon.get.return_value = NS(landed_state=1)
        obj.live = Mock(return_value=(NS(armed=False, mode='OFFBOARD'), odom(speed=.2), NS(state=s.TAKEOFF)))
        with self.assertRaisesRegex(s.SafetyError, '已离地/移动'):
            obj.wait_hover((0, 0, 0), s.time.monotonic())
        obj.sleep.assert_not_called()

    def test_unconfirmed_state_with_altitude_change_aborts(self):
        obj = self.make()
        obj.mon.quiet.return_value = True
        obj.live = Mock(return_value=(NS(armed=False, mode='OFFBOARD'), odom(.2), NS(state=s.TAKEOFF)))
        with self.assertRaisesRegex(s.SafetyError, '已离地/移动'):
            obj.wait_hover((0, 0, 0), s.time.monotonic())
        obj.sleep.assert_not_called()

    def test_unconfirmed_state_with_position_drift_aborts(self):
        obj = self.make()
        obj.mon.quiet.return_value = True
        obj.mon.get.return_value = NS(landed_state=1)
        drifting = odom()
        drifting.pose.pose.position.x = .11
        obj.live = Mock(return_value=(NS(armed=False, mode='OFFBOARD'), drifting, NS(state=s.TAKEOFF)))
        with self.assertRaisesRegex(s.SafetyError, '已离地/移动'):
            obj.wait_hover((0, 0, 0), s.time.monotonic())
        obj.sleep.assert_not_called()

    def test_hover_without_armed_state_aborts_without_grace(self):
        obj = self.make()
        obj.mon.quiet.return_value = True
        obj.live = Mock(return_value=(NS(armed=False, mode='OFFBOARD'), odom(.8), NS(state=s.HOVER)))
        with self.assertRaisesRegex(s.SafetyError, '未保持armed'):
            obj.wait_hover((0, 0, 0), s.time.monotonic())
        obj.sleep.assert_not_called()

    def test_hover_hold_restarts_after_motion(self):
        obj = self.make()
        obj.mon.quiet.return_value = True
        clock = [10.]
        obj.sleep.side_effect = lambda duration: clock.__setitem__(0, clock[0] + .25)
        obj.live = Mock(side_effect=lambda: (
            NS(armed=True, mode='OFFBOARD'),
            odom(.8, speed=.2 if clock[0] == 12. else 0.), NS(state=s.HOVER)))
        with patch.object(s.time, 'monotonic', side_effect=lambda: clock[0]):
            obj.wait_hover((0, 0, 0), 10.)
        self.assertGreaterEqual(clock[0], 15.25)

    def test_stale_topic_during_initial_confirmation_still_aborts(self):
        obj = self.make()
        obj.mon.quiet.return_value = True
        obj.mon.get.return_value = NS(landed_state=1)
        obj.live = Mock(side_effect=[
            (NS(armed=False, mode='OFFBOARD'), odom(), NS(state=s.TAKEOFF)),
            s.SafetyError('话题缺失/超时：/mavros/state'),
        ])
        with self.assertRaisesRegex(s.SafetyError, '话题缺失/超时'):
            obj.wait_hover((0, 0, 0), s.time.monotonic())
        obj.ros.Publisher.assert_not_called()

    def test_rejected_takeoff_not_retried(self):
        obj = self.make()
        obj.live = Mock(return_value=(NS(armed=False, mode='POSCTL'), odom(), NS(state=s.MANUAL)))
        obj.mon.quiet.return_value = True
        with self.assertRaisesRegex(s.SafetyError, '不自动重试'):
            obj.wait_hover((0, 0, 0), s.time.monotonic()-6)
        obj.ros.Publisher.assert_not_called()

    def test_stable_hover_requires_continuous_hold(self):
        obj = self.make()
        obj.live = Mock(return_value=(NS(armed=True, mode='OFFBOARD'), odom(.8), NS(state=s.HOVER)))
        obj.mon.quiet.return_value = True
        clock = [10.]
        obj.sleep.side_effect = lambda dt: clock.__setitem__(0, clock[0]+1)
        with patch.object(s.time, 'monotonic', side_effect=lambda: clock[0]):
            result = obj.wait_hover((0, 0, 0), 10.)
        self.assertGreaterEqual(clock[0], 13.)
        self.assertEqual(result.pose.pose.position.z, .8)

    def test_graph_rejects_other_goal_source(self):
        obj = self.make()
        obj.ros.get_name.return_value = '/sequence'
        pubs = [(s.TOPICS['cmd'], [s.TRAJ_SERVER]), ('/goal', ['/multipointplan'])]
        subs = [('/goal', [s.PLANNER]), ('/px4ctrl/takeoff_land', ['/px4ctrl']),
                (s.TOPICS['cmd'], ['/px4ctrl'])]
        obj.ros.get_master.return_value.getSystemState.return_value = (1, '', [pubs, subs, []])
        with self.assertRaisesRegex(s.SafetyError, '其他发布者'):
            obj.graph_ready()

    def test_graph_accepts_expected_chain(self):
        obj = self.make()
        obj.ros.get_name.return_value = '/sequence'
        pubs = [(s.TOPICS['cmd'], [s.TRAJ_SERVER]), ('/goal', ['/sequence'])]
        subs = [('/goal', [s.PLANNER]), ('/px4ctrl/takeoff_land', ['/px4ctrl']),
                (s.TOPICS['cmd'], ['/px4ctrl'])]
        obj.ros.get_master.return_value.getSystemState.return_value = (1, '', [pubs, subs, []])
        obj.graph_ready()


class GoalMotionSafety(unittest.TestCase):
    def setUp(self):
        self.obj = Sequence().make()
        self.clock = [10.]
        self.obj.sleep.side_effect = lambda duration: self.clock.__setitem__(0, self.clock[0] + duration)
        self.obj.live = Mock(return_value=(NS(armed=True, mode='OFFBOARD'), odom(.8), NS(state=s.COMMAND)))
        self.obj.mon.command_after.return_value = True
        self.obj.request_landing = Mock()
        self.publisher = Mock()

    def monitor(self):
        with patch.object(s.time, 'monotonic', side_effect=lambda: self.clock[0]):
            self.obj.monitor_goal((0, 0, .8), 10., self.publisher)

    def test_stationary_command_requests_landing_after_three_seconds(self):
        self.monitor()
        self.assertGreaterEqual(self.clock[0], 13.)
        self.assertLess(self.clock[0], 13.1)
        self.obj.request_landing.assert_called_once_with(self.publisher)

    def test_no_command_but_hover_still_requests_landing(self):
        self.obj.live.return_value = (NS(armed=True, mode='OFFBOARD'), odom(.8), NS(state=s.HOVER))
        self.obj.mon.command_after.return_value = False
        self.monitor()
        self.obj.request_landing.assert_called_once()

    def test_real_horizontal_motion_passes_but_waits_for_full_window(self):
        moving = odom(.8)
        moving.pose.pose.position.x = .08
        self.obj.live.return_value = (NS(armed=True, mode='OFFBOARD'), moving, NS(state=s.COMMAND))
        self.monitor()
        self.assertGreaterEqual(self.clock[0], 13.)
        self.obj.request_landing.assert_not_called()

    def test_vertical_motion_does_not_count_as_waypoint_motion(self):
        self.obj.live.return_value = (NS(armed=True, mode='OFFBOARD'), odom(1.), NS(state=s.COMMAND))
        self.monitor()
        self.obj.request_landing.assert_called_once()

    def test_small_horizontal_noise_does_not_pass(self):
        noisy = odom(.8)
        noisy.pose.pose.position.x = .02
        self.obj.live.return_value = (NS(armed=True, mode='OFFBOARD'), noisy, NS(state=s.COMMAND))
        self.monitor()
        self.obj.request_landing.assert_called_once()

    def test_motion_without_handoff_is_not_reported_as_success(self):
        moving = odom(.8)
        moving.pose.pose.position.y = .1
        self.obj.live.return_value = (NS(armed=True, mode='OFFBOARD'), moving, NS(state=s.HOVER))
        self.obj.mon.command_after.return_value = False
        with self.assertRaisesRegex(s.SafetyError, '10秒'):
            self.monitor()
        self.obj.request_landing.assert_not_called()

    def test_manual_takeover_never_requests_landing(self):
        self.obj.live.return_value = (NS(armed=True, mode='POSCTL'), odom(.8), NS(state=s.HOVER))
        with self.assertRaisesRegex(s.SafetyError, '不抢占飞手'):
            self.monitor()
        self.obj.request_landing.assert_not_called()

    def test_stale_localization_never_requests_landing(self):
        self.obj.live.side_effect = s.SafetyError('话题缺失/超时：EKF')
        with self.assertRaisesRegex(s.SafetyError, '超时'):
            self.monitor()
        self.obj.request_landing.assert_not_called()

    def test_rc_takeover_never_requests_landing(self):
        self.obj.live.side_effect = s.SafetyError('CH5/CH6需均高于1750')
        with self.assertRaisesRegex(s.SafetyError, 'CH5/CH6'):
            self.monitor()
        self.obj.request_landing.assert_not_called()


class LandingSafety(unittest.TestCase):
    def setUp(self):
        self.obj = Sequence().make()
        self.publisher = Mock()
        self.publisher.get_num_connections.return_value = 1
        self.clock = [10.]
        self.obj.sleep.side_effect = lambda duration: self.clock.__setitem__(0, self.clock[0] + duration)
        self.active = (NS(armed=True, mode='OFFBOARD'), odom(.8), NS(state=s.COMMAND))
        self.landing = (NS(armed=True, mode='OFFBOARD'), odom(.8), NS(state=s.LANDING))

    def request(self):
        with patch.object(s.time, 'monotonic', side_effect=lambda: self.clock[0]):
            self.obj.request_landing(self.publisher)

    def test_landing_is_published_once_and_auto_land_is_confirmed(self):
        from quadrotor_msgs.msg import TakeoffLand
        self.obj.live = Mock(side_effect=[self.active, self.active, self.landing])
        self.request()
        self.assertTrue(self.obj.landing_requested)
        self.publisher.publish.assert_called_once()
        self.assertEqual(self.publisher.publish.call_args[0][0].takeoff_land_cmd, TakeoffLand.LAND)
        self.obj.ros.ServiceProxy.assert_not_called()

    def test_legacy_controller_rejection_is_bounded_no_retry(self):
        self.obj.live = Mock(return_value=self.active)
        with self.assertRaisesRegex(s.SafetyError, '未确认AUTO_LAND'):
            self.request()
        self.publisher.publish.assert_called_once()
        self.assertLess(self.clock[0], 12.1)

    def test_takeover_just_before_landing_blocks_publication(self):
        self.obj.live = Mock(return_value=(NS(armed=True, mode='POSCTL'), odom(.8), NS(state=s.HOVER)))
        with self.assertRaisesRegex(s.SafetyError, '不抢占飞手'):
            self.request()
        self.publisher.publish.assert_not_called()

    def test_missing_connection_blocks_landing(self):
        self.obj.live = Mock(return_value=self.active)
        self.publisher.get_num_connections.return_value = 0
        with self.assertRaisesRegex(s.SafetyError, '连接丢失'):
            self.request()
        self.publisher.publish.assert_not_called()

    def test_duplicate_landing_is_never_retried(self):
        self.obj.live = Mock(return_value=self.active)
        self.obj.landing_requested = True
        with self.assertRaisesRegex(s.SafetyError, '不自动重试'):
            self.request()
        self.publisher.publish.assert_not_called()

    def test_fcu_takeover_after_landing_stops_confirmation_without_retry(self):
        manual = (NS(armed=True, mode='POSCTL'), odom(.8), NS(state=s.MANUAL))
        self.obj.live = Mock(side_effect=[self.active, manual])
        with self.assertRaisesRegex(s.SafetyError, '退出armed'):
            self.request()
        self.publisher.publish.assert_called_once()


if __name__ == '__main__':
    unittest.main()
