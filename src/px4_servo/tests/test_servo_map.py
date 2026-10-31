#!/usr/bin/env python3
"""servo_map 纯函数测试：不需要 ROS，直接 python3 -m unittest 即可跑。"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))

import servo_map  # noqa: E402


class LimitsTest(unittest.TestCase):

    def test_accepts_ordered_limits(self):
        self.assertEqual(servo_map.validate_limits(1000, 2200), (1000.0, 2200.0))

    def test_rejects_unordered_limits(self):
        for values in ((1500, 1500), (2000, 1000)):
            with self.assertRaises(servo_map.ServoConfigError):
                servo_map.validate_limits(*values)

    def test_rejects_non_finite_and_bool(self):
        for value in (float('nan'), float('inf'), True, '1500'):
            with self.assertRaises(servo_map.ServoConfigError):
                servo_map.validate_limits(1000, value)

    def test_center_is_midpoint(self):
        self.assertEqual(servo_map.center_us(500, 2500), 1500.0)
        self.assertEqual(servo_map.half_span_us(500, 2500), 1000.0)

    def test_asymmetric_range_center_is_still_midpoint(self):
        # PX4 的换算只看 min/max 中点，不看 1500；这里确认节点没有偷偷假设 1500
        self.assertEqual(servo_map.center_us(1000, 2000), 1500.0)
        self.assertEqual(servo_map.center_us(600, 1400), 1000.0)


class PulseToNormalizedTest(unittest.TestCase):

    def test_center_maps_to_zero(self):
        self.assertAlmostEqual(servo_map.pwm_to_normalized(1500, 1000, 2000), 0.0)

    def test_positive_half(self):
        self.assertAlmostEqual(servo_map.pwm_to_normalized(1750, 1000, 2000), 0.5)

    def test_negative_half(self):
        self.assertAlmostEqual(servo_map.pwm_to_normalized(1250, 1000, 2000), -0.5)

    def test_full_scale_2200_needs_widened_max(self):
        self.assertAlmostEqual(servo_map.pwm_to_normalized(2200, 1000, 2200), 1.0)
        # 默认 max 2000 时 2200 根本发不出去，必须报错而不是静默钳位
        with self.assertRaises(servo_map.ServoConfigError) as caught:
            servo_map.pwm_to_normalized(2200, 1000, 2000, channel=6)
        self.assertIn('PWM_AUX_MAX6', str(caught.exception))

    def test_out_of_range(self):
        for pulse in (999, 2001):
            with self.assertRaises(servo_map.ServoConfigError):
                servo_map.pwm_to_normalized(pulse, 1000, 2000)

    def test_round_trip(self):
        for pulse in (1000, 1200, 1500, 1900, 2200):
            normalized = servo_map.pwm_to_normalized(pulse, 1000, 2200)
            self.assertAlmostEqual(
                servo_map.normalized_to_pwm(normalized, 1000, 2200), pulse, places=6)


class ActuatorSetIndexTest(unittest.TestCase):

    def test_offboard_actuator_sets(self):
        self.assertEqual(servo_map.actuator_set_index(301), 1)
        self.assertEqual(servo_map.actuator_set_index(306), 6)
        self.assertEqual(servo_map.actuator_set_index(306.0), 6)

    def test_other_functions_are_rejected(self):
        for value in (0, 101, 300, 307, 400, 401, 430, float('nan'), float('inf'), True, None):
            self.assertIsNone(servo_map.actuator_set_index(value), value)


class CommandParamsTest(unittest.TestCase):

    def test_only_target_slot_is_filled(self):
        params = servo_map.command_params(6, 1.0)
        self.assertEqual(len(params), 6)
        self.assertEqual(params[5], 1.0)
        for slot in params[:5]:
            self.assertTrue(math.isnan(slot))

    def test_rejects_bad_index(self):
        for index in (0, 7):
            with self.assertRaises(servo_map.ServoConfigError):
                servo_map.command_params(index, 0.5)

    def test_rejects_out_of_range_value(self):
        with self.assertRaises(servo_map.ServoConfigError):
            servo_map.command_params(1, 1.5)

    def test_clamps_rounding_noise(self):
        params = servo_map.command_params(1, 1.0 + 1e-12)
        self.assertEqual(params[0], 1.0)


class DescribeFunctionTest(unittest.TestCase):

    def test_configured_channel(self):
        text = servo_map.describe_output_function(306, 6)
        self.assertIn('Offboard Actuator Set 6', text)

    def test_disabled_channel_points_at_fix(self):
        text = servo_map.describe_output_function(0, 6)
        self.assertIn('PWM_AUX_FUNC6', text)
        self.assertIn('301..306', text)

    def test_parachute_hint(self):
        self.assertIn('Parachute', servo_map.describe_output_function(401, 6))


class FieldConfigurationTest(unittest.TestCase):
    """本机 AUX6 参数 dump：MIN6=500 / MAX6=2500（无 TRIM6），收起 500，推出 1400（归一化 -0.1）。"""

    MIN_US = 500.0
    MAX_US = 2500.0

    def test_retract_maps_to_minus_one(self):
        self.assertAlmostEqual(servo_map.pwm_to_normalized(500, self.MIN_US, self.MAX_US), -1.0)

    def test_center_maps_to_zero(self):
        self.assertAlmostEqual(servo_map.pwm_to_normalized(1500, self.MIN_US, self.MAX_US), 0.0)

    def test_2200_maps_to_0_7(self):
        self.assertAlmostEqual(servo_map.pwm_to_normalized(2200, self.MIN_US, self.MAX_US), 0.7)

    def test_max_maps_to_one(self):
        self.assertAlmostEqual(servo_map.pwm_to_normalized(2500, self.MIN_US, self.MAX_US), 1.0)

    def test_push_at_max_sends_exactly_one(self):
        # 区间上限 2500 µs 必须被当作合法值接受（推出位 1400 在区间内，更不会有问题），param6 = +1.0
        normalized = servo_map.pwm_to_normalized(2500, self.MIN_US, self.MAX_US)
        params = servo_map.command_params(6, normalized)
        self.assertEqual(params[5], 1.0)
        for slot in params[:5]:
            self.assertTrue(math.isnan(slot))

    def test_round_trip_over_full_range(self):
        for pulse in (500, 900, 1200, 1400, 1500, 2200, 2500):
            normalized = servo_map.pwm_to_normalized(pulse, self.MIN_US, self.MAX_US)
            self.assertAlmostEqual(
                servo_map.normalized_to_pwm(normalized, self.MIN_US, self.MAX_US), pulse, places=6)

    def test_narrow_range_rejects_push(self):
        # 若 MAX6 还是默认的 2000，2200 推不出去，节点应该在启动自检时就报出来
        with self.assertRaises(servo_map.ServoConfigError) as caught:
            servo_map.pwm_to_normalized(2200, 1000, 2000, channel=6)
        self.assertIn('PWM_AUX_MAX6', str(caught.exception))

    def test_outside_configured_range_is_rejected(self):
        for pulse in (499, 2501):
            with self.assertRaises(servo_map.ServoConfigError):
                servo_map.pwm_to_normalized(pulse, self.MIN_US, self.MAX_US, channel=6)


if __name__ == '__main__':
    unittest.main()
