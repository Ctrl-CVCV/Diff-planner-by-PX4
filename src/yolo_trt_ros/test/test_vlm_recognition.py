"""The VLM path must emit the same detection fields the mission already reads."""
import unittest

from vlm_recognition import (
    RecognitionHold, disabled_payload, enabled_payload, make_detection,
    should_call_vlm, yolo_class,
)


class VlmRecognitionTests(unittest.TestCase):
    def test_labels_match_the_yolo_names_the_mission_searches(self):
        self.assertEqual(yolo_class('sign', '危险标志'), (0, '危险品标识'))
        self.assertIsNone(yolo_class('sign', '非危险标志'))
        self.assertEqual(yolo_class('portrait', '人物头像'), (2, '救助人员'))
        self.assertIsNone(yolo_class('portrait', '不是人物头像'))
        self.assertIsNone(yolo_class('sign', '人物头像'))
        self.assertIsNone(yolo_class('portrait', '危险标志'))

    def test_detection_and_payload_keep_the_yolo_fields(self):
        detection = make_detection(0, '危险品标识', (100, 80, 200, 180), 640, 480)
        self.assertEqual(detection['label'], '危险品标识')
        self.assertEqual(detection['confidence'], 1.0)
        self.assertEqual(detection['cx'], 200.0)
        self.assertEqual(detection['cy'], 170.0)
        self.assertIn('危险品', detection['label'])
        payload = enabled_payload(
            10.0, 'camera', 7, 640, 480, [detection], 2500.0, 30.0, 10.0)
        self.assertTrue(payload['enabled'])
        self.assertEqual(payload['count'], 1)
        self.assertEqual(payload['image_width'], 640)
        self.assertEqual(payload['detections'][0]['class_id'], 0)
        empty = disabled_payload(11.0, 30.0)
        self.assertFalse(empty['enabled'])
        self.assertEqual(empty['detections'], [])
        self.assertEqual(empty['count'], 0)

    def test_each_enable_calls_the_vlm_once(self):
        self.assertTrue(should_call_vlm('sign', False, False))
        self.assertTrue(should_call_vlm('portrait', False, False))
        self.assertFalse(should_call_vlm(None, False, False))
        self.assertFalse(should_call_vlm('sign', True, False))
        self.assertFalse(should_call_vlm('portrait', False, True))

    def test_positive_result_stays_marked_for_four_seconds(self):
        hold = RecognitionHold(4.0)
        hold.start(100.0, 'red', 0, '危险品标识', 1800.0)
        self.assertTrue(hold.running(103.9))
        self.assertFalse(hold.running(104.0))
        hold.clear()
        self.assertFalse(hold.running(100.1))
        hold.start(100.0, 'red', 0, '危险品标识', 1800.0)
        hold.note_box((10, 10, 40, 40))
        self.assertFalse(hold.missed(100.2))
        self.assertTrue(hold.missed(100.6))


if __name__ == '__main__':
    unittest.main()
