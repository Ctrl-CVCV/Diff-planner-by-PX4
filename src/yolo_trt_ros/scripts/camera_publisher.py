#!/usr/bin/env python3
"""Publish one local V4L2 camera as bounded-queue sensor_msgs/Image messages."""
import math
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class CameraPublisher:
    def __init__(self):
        self.device = rospy.get_param('~device', '/dev/video0')
        self.width = int(rospy.get_param('~width', 640))
        self.height = int(rospy.get_param('~height', 480))
        self.fps = float(rospy.get_param('~fps', 30.0))
        self.fourcc = str(rospy.get_param('~fourcc', 'MJPG'))
        self.buffer_size = int(rospy.get_param('~buffer_size', 2))
        self.frame_id = rospy.get_param('~frame_id', 'camera_optical_frame')
        self.topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')
        self.open_attempts = int(rospy.get_param('~open_attempts', 3))
        self.max_failures = int(rospy.get_param('~max_read_failures', 5))
        self.retry_delay = float(rospy.get_param('~retry_delay', 0.2))
        if self.width < 1 or self.height < 1 or not math.isfinite(self.fps) or not 0 < self.fps <= 60:
            raise ValueError('width/height must be positive and fps in (0, 60]')
        if self.fourcc and len(self.fourcc) != 4:
            raise ValueError('fourcc must be empty or contain exactly four characters')
        if not 1 <= self.buffer_size <= 8:
            raise ValueError('buffer_size must be in [1, 8]')
        if not 1 <= self.open_attempts <= 20 or not 1 <= self.max_failures <= 100:
            raise ValueError('open_attempts in [1, 20], max_read_failures in [1, 100] required')
        if not math.isfinite(self.retry_delay) or not 0 <= self.retry_delay <= 5:
            raise ValueError('retry_delay must be in [0, 5] seconds')
        self.capture = None
        self.sequence = 0
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher(self.topic, Image, queue_size=1)
        self.actual = {}

    def _release(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None

    def _open(self):
        for attempt in range(1, self.open_attempts + 1):
            if rospy.is_shutdown():
                return False
            self.capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            if self.capture.isOpened():
                if self.fourcc:
                    self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
                self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.capture.set(cv2.CAP_PROP_FPS, self.fps)
                self.capture.set(cv2.CAP_PROP_BUFFERSIZE, self.buffer_size)
                self.actual = {
                    'width': self.capture.get(cv2.CAP_PROP_FRAME_WIDTH),
                    'height': self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT),
                    'fps': self.capture.get(cv2.CAP_PROP_FPS),
                    'fourcc': int(self.capture.get(cv2.CAP_PROP_FOURCC)),
                    'buffer_size': self.capture.get(cv2.CAP_PROP_BUFFERSIZE),
                }
                rospy.loginfo('V4L2 camera %s -> %s; requested %dx%d %.2f fps fourcc=%s buffer=%d; actual=%s',
                              self.device, self.topic, self.width, self.height,
                              self.fps, self.fourcc or 'driver default', self.buffer_size, self.actual)
                return True
            self._release()
            rospy.logwarn('Cannot open V4L2 camera %s, attempt %d/%d',
                          self.device, attempt, self.open_attempts)
            if attempt < self.open_attempts:
                time.sleep(self.retry_delay)
        return False

    def run(self):
        try:
            if not self._open():
                if rospy.is_shutdown():
                    return
                raise RuntimeError('could not open camera {}'.format(self.device))
            rate = rospy.Rate(self.fps)
            failures = 0
            while not rospy.is_shutdown():
                success, frame = self.capture.read()
                if not success or frame is None or frame.size == 0:
                    failures += 1
                    rospy.logwarn('Camera %s read failed (%d/%d)',
                                  self.device, failures, self.max_failures)
                    if failures >= self.max_failures:
                        raise RuntimeError('camera stopped delivering valid frames')
                    time.sleep(self.retry_delay)
                    continue
                failures = 0
                # Hardware may negotiate a different size. Publish the actual frame,
                # preserving its aspect ratio; CvBridge derives width/height/step.
                message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                message.header.seq = self.sequence
                message.header.stamp = rospy.Time.now()
                message.header.frame_id = self.frame_id
                self.publisher.publish(message)
                self.sequence = (self.sequence + 1) % (2 ** 32)
                rate.sleep()
        finally:
            self._release()


def main():
    rospy.init_node('usb_camera')
    try:
        CameraPublisher().run()
    except rospy.ROSInterruptException:
        return 0
    except Exception as exc:
        rospy.logfatal('Camera publisher stopped: %s', exc)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
