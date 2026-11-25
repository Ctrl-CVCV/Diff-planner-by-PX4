#!/usr/bin/env python3
"""Replace the TensorRT detector with Qwen2.5-VL, keeping the YOLO topics.

/yolo/enable (std_msgs/Bool) is a one-shot trigger, not a level switch.
false runs one danger-sign recognition. true runs one portrait recognition.
That message alone chooses the VLM question. Border color is not consulted.
The crop sent to the VLM is the outer rectangular frame. The published sign box
is the rectangle tangent to the triangle's three corners. A portrait keeps its
outermost frame. Either box is followed for about 4 seconds.
非危险标志 and 不是人物头像 are published as class 1, 其它标识. The JSON on
/yolo/detections uses the original YOLO fields.
"""
import json
import os
import sys
import threading
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vlm_client import VisionError, classify_frame
from vlm_track import follow_target, recognition_crop
from vlm_recognition import (
    RecognitionHold, disabled_payload, enabled_payload, make_detection,
    should_call_vlm, yolo_class,
)

_CLASS_COLORS = ((0, 220, 0), (0, 96, 255), (255, 200, 0))
_PUBLISH_INTERVAL = 0.1


class VlmDetectorNode(object):
    def __init__(self):
        self.image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')
        self.preview_topic = rospy.get_param('~preview_topic', '/yolo/preview/image')
        self.detections_topic = rospy.get_param('~detections_topic', '/yolo/detections')
        self.enable_topic = str(rospy.get_param('~enable_topic', '/yolo/enable'))
        self.endpoint = rospy.get_param(
            '~endpoint', 'http://127.0.0.1:8080/v1/chat/completions')
        self.model = rospy.get_param('~model', 'qwen2.5-vl-7b')
        self.timeout = float(rospy.get_param('~timeout', 60.0))
        self.max_image_side = int(rospy.get_param('~max_image_side', 640))
        self.display = bool(rospy.get_param('~show_window', True))
        self.window_name = str(rospy.get_param('~window_name', 'VLM Recognition'))
        self.hold = RecognitionHold(float(rospy.get_param('~hold_seconds', 4.0)))
        if self.display and not os.environ.get('DISPLAY'):
            rospy.logwarn('show_window is set but DISPLAY is empty; preview window disabled')
            self.display = False

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest = None
        self.sequence = 0
        self.camera_fps = 0.0
        self._last_frame_time = None
        self.busy = False
        self._pending = None
        self._active = False
        self._generation = 0
        self._last_publish = 0.0
        self._published = 0
        self._publish_mark = time.monotonic()
        self.processed_fps = 0.0
        self.preview_pub = rospy.Publisher(self.preview_topic, Image, queue_size=1)
        self.detections_pub = rospy.Publisher(self.detections_topic, String, queue_size=1)
        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self._on_image, queue_size=1,
            buff_size=16 * 1024 * 1024)
        self.enable_sub = rospy.Subscriber(
            self.enable_topic, Bool, self._on_enable, queue_size=1)
        self.disabled_timer = rospy.Timer(rospy.Duration(_PUBLISH_INTERVAL),
                                          self._publish_disabled)
        rospy.loginfo('VLM detector waits for one %s message: false=危险品, true=人像 | hold %.1fs',
                      self.enable_topic, self.hold.seconds)

    def _on_enable(self, message):
        mode = 'portrait' if bool(message.data) else 'sign'
        with self.lock:
            self._generation += 1
            self._pending = mode
            self._active = True
            self.hold.clear()
        rospy.logwarn('单次识别触发：%s（%s=%s）',
                      '人像' if mode == 'portrait' else '危险品',
                      self.enable_topic, bool(message.data))

    def _publish_disabled(self, _event):
        with self.lock:
            if self._active or self.hold.running(time.monotonic()):
                return
            camera_fps = self.camera_fps
        self._publish_json(disabled_payload(rospy.Time.now().to_sec(), camera_fps))

    def _on_image(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as exc:
            rospy.logwarn_throttle(5.0, 'image conversion failed: %s', exc)
            return
        now = time.monotonic()
        with self.lock:
            self._note_camera_fps(now)
            self.sequence += 1
            sequence = self.sequence
            self.latest = (sequence, message.header, frame)
            hold = self.hold
            pending = self._pending
            busy = self.busy
            camera_fps = self.camera_fps
            processed_fps = self._processed_fps(now)
        holding = hold.running(now)
        detection = None
        if holding:
            detection = self._track(frame, hold, now)
        elif should_call_vlm(pending, busy, holding):
            shape = 'triangle' if pending == 'sign' else 'square'
            try:
                crop = recognition_crop(frame, shape)
            except (ValueError, cv2.error) as exc:
                rospy.logwarn_throttle(5.0, 'recognition ROI failed: %s', exc)
                crop = None
            if crop is None:
                rospy.loginfo_throttle(2.0, '等待画面中的外层矩形边框后再请求 VLM')
            else:
                self._start_inference(crop['image'], pending)
        with self.lock:
            if not self.hold.running(now) and not self.busy and self._pending is None:
                self._active = False
            active = self._active
        preview = self._draw(frame, detection, hold if detection else None, now)
        self._publish_preview(preview, message.header)
        if not active:
            return
        if now - self._last_publish < _PUBLISH_INTERVAL:
            return
        payload = enabled_payload(
            message.header.stamp.to_sec(), message.header.frame_id, sequence,
            frame.shape[1], frame.shape[0],
            [] if detection is None else [detection],
            0.0 if detection is None else hold.inference_ms,
            camera_fps, processed_fps)
        self._publish_json(payload)

    def _start_inference(self, frame, mode):
        with self.lock:
            if not should_call_vlm(
                    self._pending, self.busy, self.hold.running(time.monotonic())):
                return
            if self._pending != mode:
                return
            self.busy = True
            self._pending = None
            generation = self._generation
        snapshot = frame.copy()
        worker = threading.Thread(
            target=self._infer, name='vlm_infer',
            args=(snapshot, mode, generation), daemon=True)
        worker.start()

    def _track(self, frame, hold, now):
        """Move the published box to the triangle or square in this frame."""
        tracked = None
        if hold.shape in ('triangle', 'square'):
            try:
                tracked = follow_target(frame, hold.shape, hold.bbox)
            except (ValueError, cv2.error) as exc:
                rospy.logwarn_throttle(5.0, 'target tracking failed: %s', exc)
        if tracked is not None:
            hold.note_box(tracked['bbox'], tracked.get('polygon'))
            return make_detection(
                hold.class_id, hold.label, hold.bbox, frame.shape[1], frame.shape[0])
        if hold.bbox is not None and not hold.missed(now):
            return make_detection(
                hold.class_id, hold.label, hold.bbox, frame.shape[1], frame.shape[0])
        if hold.bbox is not None:
            with self.lock:
                hold.clear()
        return None

    def _infer(self, frame, mode, generation):
        try:
            text, elapsed_ms = classify_frame(
                frame, mode, self.endpoint, self.model,
                self.timeout, self.max_image_side)
            mapped = yolo_class(mode, text)
            rospy.loginfo('VLM %s -> %s (%.0f ms)', mode, text, elapsed_ms)
        except VisionError as exc:
            rospy.logwarn('VLM request failed: %s', exc)
            mapped = None
            elapsed_ms = 0.0
        with self.lock:
            self.busy = False
            if generation != self._generation:
                return
            if mapped is None:
                self._active = False
                rospy.loginfo('本次触发未确认目标，不再重复识别')
                return
            class_id, label = mapped
            self.hold.start(time.monotonic(), mode, class_id, label, elapsed_ms)
            rospy.loginfo('已确认%s，之后 %.1f 秒跟随%s', label, self.hold.seconds,
                          '与三角形相切的矩形' if self.hold.shape == 'triangle' else '最外层边框')

    def _draw(self, frame, detection, hold, now):
        canvas = frame.copy()
        if detection is not None:
            color = _CLASS_COLORS[detection['class_id'] % len(_CLASS_COLORS)]
            anchor = (int(round(detection['x1'])), int(round(detection['y1'])))
            cv2.rectangle(
                canvas, anchor,
                (int(round(detection['x2'])), int(round(detection['y2']))),
                color, 2)
            remaining = max(0.0, hold.until - now) if hold and hold.until else 0.0
            cv2.putText(canvas, '%s %.1fs' % (detection['label'], remaining),
                        (anchor[0], max(24, anchor[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, color, 2, cv2.LINE_AA)
        if self.display:
            try:
                cv2.imshow(self.window_name, canvas)
                cv2.waitKey(1)
            except cv2.error as exc:
                rospy.logwarn('preview window disabled: %s', exc)
                self.display = False
        return canvas

    def _publish_preview(self, canvas, header):
        preview = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
        preview.header = header
        self.preview_pub.publish(preview)

    def _publish_json(self, payload):
        self.detections_pub.publish(String(data=json.dumps(
            payload, ensure_ascii=False, separators=(',', ':'))))
        now = time.monotonic()
        self._last_publish = now
        self._published += 1

    def _note_camera_fps(self, now):
        if self._last_frame_time is not None:
            delta = now - self._last_frame_time
            if delta > 0:
                instant = 1.0 / delta
                self.camera_fps = (instant if self.camera_fps == 0.0
                                   else 0.9 * self.camera_fps + 0.1 * instant)
        self._last_frame_time = now

    def _processed_fps(self, now):
        elapsed = now - self._publish_mark
        if elapsed >= 1.0:
            self.processed_fps = self._published / elapsed
            self._published = 0
            self._publish_mark = now
        return self.processed_fps


def main():
    rospy.init_node('yolo_trt_detector')
    try:
        VlmDetectorNode()
    except (ValueError, TypeError) as exc:
        rospy.logfatal('Invalid VLM detector configuration: %s', exc)
        return 1
    rospy.spin()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
