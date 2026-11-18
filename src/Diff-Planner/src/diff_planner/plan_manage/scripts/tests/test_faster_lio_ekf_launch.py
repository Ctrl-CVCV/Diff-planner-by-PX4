#!/usr/bin/env python3

import os
import shlex
import unittest

import roslaunch


LAUNCH_PATH = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'launch', 'faster_lio_ekf_bridge.launch'))


def load_config(*arguments, launch_path=LAUNCH_PATH):
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(
        launch_path, config, argv=list(arguments), verbose=False)
    return config


class FasterLioEkfLaunchTest(unittest.TestCase):

    def test_default_starts_only_external_ekf(self):
        config = load_config()
        self.assertEqual([(node.package, node.type) for node in config.nodes],
                         [('ekf', 'ekf')])
        remaps = dict(config.nodes[0].remap_args)
        self.assertEqual(remaps['~bodyodometry'], '/Odometry')
        self.assertEqual(remaps['~imu'], '/mavros/imu/data')
        self.assertEqual(remaps['~ekf_odom'], '/ekf/ekf_odom')

    def test_keeps_identity_extrinsics_and_zero_offsets(self):
        config = load_config()
        self.assertEqual(config.params['/ekf/Rr_i'].value, [1, 0, 0, 0])
        self.assertEqual(config.params['/ekf/tr_i'].value, [0.0, 0.0, 0.0])
        for parameter in ('offset_px', 'offset_py', 'offset_pz',
                          'imu_trans_x', 'imu_trans_y', 'imu_trans_z'):
            self.assertEqual(config.params['/ekf/' + parameter].value, 0.0)

    def test_remapped_lio_and_ekf_output(self):
        config = load_config('lio_odom_topic:=/Odometry',
                             'imu_topic:=/test/imu',
                             'ekf_odom_topic:=/test/ekf')
        remaps = dict(config.nodes[0].remap_args)
        self.assertEqual(remaps['~bodyodometry'], '/Odometry')
        self.assertEqual(remaps['~imu'], '/test/imu')
        self.assertEqual(remaps['~ekf_odom'], '/test/ekf')

    def test_bridge_requires_explicit_enable_and_uses_ekf_pose(self):
        config = load_config('publish_vision:=true')
        self.assertEqual(len(config.nodes), 2)
        bridge = config.nodes[1]
        self.assertEqual(bridge.type, 'ekf_vision_bridge.py')
        arguments = shlex.split(bridge.args)
        self.assertEqual(arguments[arguments.index('--odom') + 1], '/ekf/ekf_odom')
        self.assertEqual(arguments[arguments.index('--topic') + 1],
                         '/mavros/vision_pose/pose')
        self.assertNotIn('--frame', arguments)
        self.assertNotIn('--stamp-offset', arguments)

    def test_existing_ekf_can_be_reused_with_preview_output(self):
        config = load_config('start_ekf:=false', 'publish_vision:=true',
                             'vision_topic:=/debug/vision_pose',
                             'ekf_odom_topic:=/test/ekf')
        self.assertEqual(len(config.nodes), 1)
        self.assertEqual(config.nodes[0].type, 'ekf_vision_bridge.py')
        arguments = shlex.split(config.nodes[0].args)
        self.assertEqual(arguments[arguments.index('--topic') + 1], '/debug/vision_pose')
        self.assertEqual(arguments[arguments.index('--odom') + 1], '/test/ekf')


class FasterLioWorkspaceLaunchTest(unittest.TestCase):

    def load_mapping(self, *arguments):
        mapping_launch = os.path.abspath(os.path.join(
            os.path.dirname(__file__), '..', '..', '..', '..',
            'realflight_modules', 'faster-lio', 'faster-lio',
            'launch', 'mapping_mid360.launch'))
        return load_config(*arguments, launch_path=mapping_launch)

    def test_default_mapping_uses_verified_config_without_starting_driver(self):
        config = self.load_mapping()
        self.assertEqual([(node.package, node.type) for node in config.nodes],
                         [('faster_lio', 'run_mapping_online')])
        self.assertAlmostEqual(config.params['/filter_size_surf'].value, 0.20)
        self.assertAlmostEqual(config.params['/filter_size_map'].value, 0.40)
        self.assertEqual(config.params['/point_filter_num'].value, 3)
        self.assertFalse(config.params['/mapping/extrinsic_est_en'].value)
        self.assertAlmostEqual(
            config.params['/common/time_offset_lidar_to_imu'].value, -0.07625571)
        self.assertEqual(config.params['/common/imu_topic'].value,
                         '/mavros/imu/data_raw')
        self.assertNotIn('/ivox_max_points_per_grid', config.params)
        remaps = dict(config.nodes[0].remap_args)
        self.assertEqual(remaps['/laserMapping/odometry'], '/Odometry')
        self.assertEqual(remaps['/laserMapping/cloud_registered'], '/cloud_registered')
        self.assertEqual(remaps['/laserMapping/cloud_registered_body'], '/cloud_registered_body')
        self.assertEqual(remaps['/laserMapping/path'], '/path')

    def test_one_click_arguments_and_custom_outputs_are_accepted(self):
        config = self.load_mapping(
            'start_lidar:=false', 'mapping_node_name:=laserMapping_faster',
            'odom_topic:=/test/odom', 'cloud_topic:=/test/cloud',
            'body_cloud_topic:=/test/body_cloud', 'path_topic:=/test/path',
            'rviz:=false')
        self.assertEqual(len(config.nodes), 1)
        self.assertEqual(config.nodes[0].name, 'laserMapping_faster')
        remaps = dict(config.nodes[0].remap_args)
        self.assertEqual(remaps['/laserMapping/odometry'], '/test/odom')
        self.assertEqual(remaps['/laserMapping/cloud_registered'], '/test/cloud')
        self.assertEqual(remaps['/laserMapping/cloud_registered_body'], '/test/body_cloud')
        self.assertEqual(remaps['/laserMapping/path'], '/test/path')

    def test_rviz_is_optional(self):
        config = self.load_mapping('rviz:=true')
        self.assertEqual([(node.package, node.type) for node in config.nodes],
                         [('faster_lio', 'run_mapping_online'), ('rviz', 'rviz')])


if __name__ == '__main__':
    unittest.main()
