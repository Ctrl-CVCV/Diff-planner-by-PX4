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


if __name__ == '__main__':
    unittest.main()
