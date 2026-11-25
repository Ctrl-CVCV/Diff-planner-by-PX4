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
import numpy as np
import rospy
from cv_bridge import CvBridge
from PIL import Image as PilImage, ImageDraw, ImageFont
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vlm_client import VisionError, classify_frame
from vlm_track import follow_target, recognition_crop
from vlm_recognition import (
    RecognitionHold, disabled_payload, enabled_payload, make_detection,
    should_call_vlm, yolo_class,
)

# 方框和文字颜色（BGR）：危险品红、救助人员蓝、其它标识橙。
_CLASS_COLORS = {0: (0, 0, 255), 1: (0, 165, 255), 2: (255, 0, 0)}
_CLASS_LABELS = {0: '危险品', 1: '其它标识', 2: '救助人员'}
_PUBLISH_INTERVAL = 0.1
_LABEL_FONT_SIZE = 20
_LABEL_FONT_CANDIDATES = (
    ('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc', 2),
    ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 2),
    ('/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc', 0),
)


def _load_label_font():
    for path, index in _LABEL_FONT_CANDIDATES:
        if os.path.exists(path):
            try:
                return ImageFont.truetype(path, _LABEL_FONT_SIZE, index=index)
            except OSError:
                continue
    return None


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
        self._label_font = _load_label_font()
        if self._label_font is None:
            rospy.logwarn('找不到中文字体（fonts-noto-cjk），预览文字将退化为英文')
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
        if detection is not None:
            status = ''
        elif busy:
            status = 'VLM 识别中…'
        elif pending in ('sign', 'portrait'):
            status = '等待外层矩形边框…'
        elif holding:
            status = '目标跟丢'
        else:
            status = ''
        preview = self._draw(frame, detection, hold if detection else None, now, status)
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

    def _draw(self, frame, detection, hold, now, status=''):
        canvas = frame.copy()
        if detection is not None:
            class_id = int(detection['class_id'])
            color = _CLASS_COLORS.get(class_id, _CLASS_COLORS[1])
            x1, y1 = int(round(detection['x1'])), int(round(detection['y1']))
            x2, y2 = int(round(detection['x2'])), int(round(detection['y2']))
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
            remaining = max(0.0, hold.until - now) if hold and hold.until else 0.0
            label = _CLASS_LABELS.get(class_id, detection['label'])
            self._draw_text(canvas, '%s %.1fs' % (label, remaining),
                            x1, y1, color, above=True)
        elif status:
            self._draw_text(canvas, status, 10, 10, (0, 200, 255), above=False)
        if self.display:
            try:
                cv2.imshow(self.window_name, canvas)
                cv2.waitKey(1)
            except cv2.error as exc:
                rospy.logwarn('preview window disabled: %s', exc)
                self.display = False
        return canvas

    def _draw_text(self, canvas, text, x, y, color, above=True):
        """用 PIL 画中文标签（cv2.putText 不能渲染中文）。"""
        if self._label_font is None:
            cv2.putText(canvas, text.encode('ascii', 'replace').decode(),
                        (max(0, int(x)), max(16, int(y))), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, color, 2, cv2.LINE_AA)
            return
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        image = PilImage.fromarray(rgb)
        draw = ImageDraw.Draw(image)
        if hasattr(draw, 'textbbox'):        # Pillow >= 8
            left, top, right, bottom = draw.textbbox((0, 0), text, font=self._label_font)
            width, height = right - left, bottom - top
        else:                                # Jetson 上的 Pillow 7.0
            width, height = draw.textsize(text, font=self._label_font)
            left = top = 0
        tx = max(0, min(int(x), canvas.shape[1] - width - 10))
        ty = int(y) - height - 8 if above else int(y)
        if ty < 0:
            ty = max(0, int(y) + 2)
        draw.rectangle([tx, ty, tx + width + 6, ty + height + 4], fill=(32, 32, 32))
        draw.text((tx + 3 - left, ty + 2 - top), text, font=self._label_font,
                  fill=(color[2], color[1], color[0]))
        canvas[:] = cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)

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
