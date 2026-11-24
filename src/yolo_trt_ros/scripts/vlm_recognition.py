"""Map a VLM label onto the YOLO detection JSON the mission already consumes."""
import math

# Same class ids and Chinese labels as the TensorRT engine metadata mapping.
YOLO_CLASSES = {
    '危险标志': (0, '危险品标识'),
    '非危险标志': (1, '其它标识'),
    '人物头像': (2, '救助人员'),
}
HOLD_SECONDS = 4.0
# The grammar has no probability. A forced label is published as a full score
# so the mission confidence gate (default 0.70) treats it like a YOLO hit.
VLM_CONFIDENCE = 1.0

_ENABLED_KEYS = (
    'stamp', 'enabled', 'frame_id', 'sequence', 'image_width', 'image_height',
    'batch_size', 'batch_inference_ms', 'inference_ms', 'processed_fps',
    'camera_fps', 'count', 'detections',
)
_DETECTION_KEYS = (
    'class_id', 'label', 'confidence', 'x1', 'y1', 'x2', 'y2', 'cx', 'cy',
    'width', 'height', 'norm_cx', 'norm_cy',
)
_DISABLED_KEYS = (
    'stamp', 'enabled', 'image_width', 'image_height', 'count', 'detections',
    'camera_fps',
)


class RecognitionHold(object):
    """Keep one accepted target marked for longer than the required 3 seconds."""

    def __init__(self, seconds=HOLD_SECONDS):
        if not math.isfinite(seconds) or seconds < 3.0:
            raise ValueError('hold must be at least 3 seconds')
        self.seconds = float(seconds)
        self.until = None
        self.color = None
        self.class_id = None
        self.label = None
        self.inference_ms = 0.0
        self.bbox = None
        self.polygon = None
        self.shape = None
        self.miss_since = None

    def start(self, now, mode, class_id, label, inference_ms):
        self.until = float(now) + self.seconds
        self.color = mode
        self.shape = 'triangle' if mode == 'sign' else 'square' if mode == 'portrait' else None
        self.class_id = int(class_id)
        self.label = label
        self.inference_ms = float(inference_ms)
        self.bbox = None
        self.polygon = None
        self.miss_since = None

    def note_box(self, bbox, polygon=None):
        self.bbox = tuple(bbox)
        if polygon is None:
            self.polygon = None
        else:
            self.polygon = tuple((int(round(x)), int(round(y))) for x, y in polygon)
        self.miss_since = None

    def missed(self, now, grace=0.3):
        """Allow a brief tracking gap so one dropped frame does not end the mark."""
        if self.miss_since is None:
            self.miss_since = float(now)
        return float(now) - self.miss_since >= grace

    def running(self, now):
        return self.until is not None and float(now) < self.until

    def clear(self):
        self.until = None
        self.color = None
        self.class_id = None
        self.label = None
        self.inference_ms = 0.0
        self.bbox = None
        self.polygon = None
        self.shape = None
        self.miss_since = None


def should_call_vlm(pending, busy, holding):
    """A trigger may start one request. Repeating it requires another message."""
    return pending in ('sign', 'portrait') and not busy and not holding


def yolo_class(mode, text):
    """Return (class_id, label) or None. The trigger selects the question.

    Both negative answers use the original YOLO class 1, 其它标识.
    """
    if mode == 'sign' and text in ('危险标志', '非危险标志'):
        return YOLO_CLASSES[text]
    if mode == 'portrait' and text == '人物头像':
        return YOLO_CLASSES[text]
    if mode == 'portrait' and text == '不是人物头像':
        return YOLO_CLASSES['非危险标志']
    return None


def make_detection(class_id, label, bbox, image_width, image_height,
                   confidence=VLM_CONFIDENCE):
    """One detection in the original YOLO field set, in full-image pixels."""
    image_width = int(image_width)
    image_height = int(image_height)
    if image_width < 1 or image_height < 1:
        raise ValueError('image size must be positive')
    x, y, width, height = [float(value) for value in bbox]
    x1 = min(max(x, 0.0), image_width - 1.0)
    y1 = min(max(y, 0.0), image_height - 1.0)
    x2 = min(max(x + width, 0.0), image_width - 1.0)
    y2 = min(max(y + height, 0.0), image_height - 1.0)
    if x2 <= x1 or y2 <= y1:
        raise ValueError('detection box is empty')
    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0
    return {
        'class_id': int(class_id),
        'label': label,
        'confidence': round(float(confidence), 3),
        'x1': round(x1, 1),
        'y1': round(y1, 1),
        'x2': round(x2, 1),
        'y2': round(y2, 1),
        'cx': round(center_x, 1),
        'cy': round(center_y, 1),
        'width': round(x2 - x1, 1),
        'height': round(y2 - y1, 1),
        'norm_cx': round(center_x / image_width, 4),
        'norm_cy': round(center_y / image_height, 4),
    }


def enabled_payload(stamp, frame_id, sequence, image_width, image_height,
                    detections, inference_ms, camera_fps, processed_fps):
    payload = {
        'stamp': float(stamp),
        'enabled': True,
        'frame_id': frame_id,
        'sequence': int(sequence),
        'image_width': int(image_width),
        'image_height': int(image_height),
        'batch_size': 1,
        'batch_inference_ms': round(float(inference_ms), 2),
        'inference_ms': round(float(inference_ms), 2),
        'processed_fps': round(float(processed_fps), 2),
        'camera_fps': round(float(camera_fps), 2),
        'count': len(detections),
        'detections': detections,
    }
    if tuple(payload) != _ENABLED_KEYS:
        raise RuntimeError('enabled payload keys drifted from the YOLO message')
    for detection in detections:
        if tuple(detection) != _DETECTION_KEYS:
            raise RuntimeError('detection keys drifted from the YOLO message')
    return payload


def disabled_payload(stamp, camera_fps):
    payload = {
        'stamp': float(stamp),
        'enabled': False,
        'image_width': 0,
        'image_height': 0,
        'count': 0,
        'detections': [],
        'camera_fps': round(float(camera_fps), 2),
    }
    if tuple(payload) != _DISABLED_KEYS:
        raise RuntimeError('disabled payload keys drifted from the YOLO message')
    return payload
