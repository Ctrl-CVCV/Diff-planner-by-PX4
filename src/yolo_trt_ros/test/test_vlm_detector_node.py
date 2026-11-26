"""Offline tests for semantic publication without a running camera or ROS master."""
import importlib.util
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest import mock

from vlm_recognition import RecognitionHold


class VlmResultPublicationTests(unittest.TestCase):
    def setUp(self):
        self.ros = mock.Mock()
        self.ros.Time.now.return_value = SimpleNamespace(to_sec=lambda: 1000.0)
        messages = SimpleNamespace(Bool=SimpleNamespace, String=SimpleNamespace)
        sensor_messages = SimpleNamespace(Image=SimpleNamespace)
        modules = {'rospy': self.ros,
                   'cv_bridge': SimpleNamespace(CvBridge=mock.Mock()),
                   'sensor_msgs': SimpleNamespace(msg=sensor_messages),
                   'sensor_msgs.msg': sensor_messages,
                   'std_msgs': SimpleNamespace(msg=messages),
                   'std_msgs.msg': messages}
        path = Path(__file__).resolve().parents[1] / 'scripts/vlm_detector_node.py'
        spec = importlib.util.spec_from_file_location('offline_vlm_detector', path)
        self.module = importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules, modules):
            spec.loader.exec_module(self.module)
        self.node = self.module.VlmDetectorNode.__new__(self.module.VlmDetectorNode)
        self.node.lock = threading.Lock()
        self.node._generation = 7
        self.node._track_prior = (10, 20, 50, 50)
        self.node._active = True
        self.node.busy = True
        self.node.hold = RecognitionHold()
        self.node.recognition_pub = mock.Mock()
        self.node.endpoint = 'offline'
        self.node.model = 'offline'
        self.node.timeout = 60.0
        self.node.max_image_side = 640

    def test_positive_result_publishes_before_any_cv_frame(self):
        for mode, answer, class_id in (('sign', '危险标志', 0),
                                       ('portrait', '人物头像', 2)):
            with self.subTest(mode=mode), mock.patch.object(
                    self.module, 'classify_frame', return_value=(answer, 1200.0)):
                self.node.recognition_pub.reset_mock()
                self.node._infer(None, mode, 7)
                self.node.recognition_pub.publish.assert_called_once()
                message = self.node.recognition_pub.publish.call_args.args[0]
                result = json.loads(message.data)
                self.assertEqual(result['class_id'], class_id)
                self.assertEqual(result['stamp'], 1000.0)
                self.assertIsNone(self.node.hold.bbox)
                self.assertEqual(self.node.hold.seconds, 3.0)

    def test_old_inference_cannot_publish_after_a_new_trigger(self):
        with mock.patch.object(self.module, 'classify_frame',
                               return_value=('危险标志', 1200.0)):
            self.node._infer(None, 'sign', 6)
        self.node.recognition_pub.publish.assert_not_called()
        self.assertIsNone(self.node.hold.until)

    def test_failed_request_does_not_publish_or_start_tracking(self):
        with mock.patch.object(self.module, 'classify_frame',
                               side_effect=self.module.VisionError('offline failure')):
            self.node._infer(None, 'sign', 7)
        self.node.recognition_pub.publish.assert_not_called()
        self.assertIsNone(self.node.hold.until)


if __name__ == '__main__':
    unittest.main()
