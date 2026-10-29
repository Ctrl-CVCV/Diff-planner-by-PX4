#!/usr/bin/env python3
"""Run a YOLO TensorRT engine on a ROS image topic with a batching pipeline."""
import copy
import ctypes
import json
import math
import os
import struct
import threading
import time
from collections import deque
from queue import Empty, Queue

# System OpenCV must be imported before cv_bridge on the Jetson Noetic stack.
import cv2
import numpy as np
import rospy
import tensorrt as trt
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

_CUDART = ctypes.CDLL('libcudart.so')
_CUDART.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
_CUDART.cudaMalloc.restype = ctypes.c_int
_CUDART.cudaFree.argtypes = [ctypes.c_void_p]
_CUDART.cudaFree.restype = ctypes.c_int
_CUDART.cudaHostAlloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t,
                                  ctypes.c_uint]
_CUDART.cudaHostAlloc.restype = ctypes.c_int
_CUDART.cudaFreeHost.argtypes = [ctypes.c_void_p]
_CUDART.cudaFreeHost.restype = ctypes.c_int
_CUDART.cudaMemcpyAsync.argtypes = [ctypes.c_void_p, ctypes.c_void_p,
                                    ctypes.c_size_t, ctypes.c_int, ctypes.c_void_p]
_CUDART.cudaMemcpyAsync.restype = ctypes.c_int
_CUDART.cudaStreamCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
_CUDART.cudaStreamCreate.restype = ctypes.c_int
_CUDART.cudaStreamDestroy.argtypes = [ctypes.c_void_p]
_CUDART.cudaStreamDestroy.restype = ctypes.c_int
_CUDART.cudaStreamWaitEvent.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint]
_CUDART.cudaStreamWaitEvent.restype = ctypes.c_int
_CUDART.cudaDeviceSynchronize.restype = ctypes.c_int
_CUDART.cudaEventCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
_CUDART.cudaEventCreate.restype = ctypes.c_int
_CUDART.cudaEventRecord.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
_CUDART.cudaEventRecord.restype = ctypes.c_int
_CUDART.cudaEventSynchronize.argtypes = [ctypes.c_void_p]
_CUDART.cudaEventSynchronize.restype = ctypes.c_int
_CUDART.cudaEventElapsedTime.argtypes = [ctypes.POINTER(ctypes.c_float),
                                         ctypes.c_void_p, ctypes.c_void_p]
_CUDART.cudaEventElapsedTime.restype = ctypes.c_int
_CUDART.cudaEventDestroy.argtypes = [ctypes.c_void_p]
_CUDART.cudaEventDestroy.restype = ctypes.c_int

_HOST_TO_DEVICE = 1
_DEVICE_TO_HOST = 2
_PAD_VALUE = 114.0 / 255.0
_SLOT_COUNT = 6
_CLASS_COLORS = [
    (0, 220, 0),
    (0, 96, 255),
    (255, 200, 0),
    (255, 0, 255),
    (0, 165, 255),
    (255, 255, 0),
]


def _cuda_check(status, action):
    if status != 0:
        raise RuntimeError('%s failed with CUDA error %d' % (action, status))


def _read_engine_file(path):
    """Split Ultralytics' optional length-prefixed JSON metadata off an engine file."""
    with open(path, 'rb') as handle:
        data = handle.read()
    if len(data) > 4 and data[4:5] == b'{':
        size = struct.unpack('<I', data[:4])[0]
        if 4 + size < len(data):
            try:
                metadata = json.loads(data[4:4 + size].decode('utf-8'))
                return data[4 + size:], metadata
            except (ValueError, UnicodeDecodeError):
                pass
    return data, {}


def _geometry(frame_shape, input_width, input_height):
    height, width = frame_shape[:2]
    ratio = min(input_width / float(width), input_height / float(height))
    resized_width = int(round(width * ratio))
    resized_height = int(round(height * ratio))
    pad_width = (input_width - resized_width) / 2.0
    pad_height = (input_height - resized_height) / 2.0
    left = int(round(pad_width - 0.1))
    top = int(round(pad_height - 0.1))
    return ratio, left, top, resized_width, resized_height


def _suppress_overlaps(detections, threshold):
    kept = []
    for det in detections:
        box_area = (det['x2'] - det['x1']) * (det['y2'] - det['y1'])
        keep = True
        for other in kept:
            inter_w = min(det['x2'], other['x2']) - max(det['x1'], other['x1'])
            inter_h = min(det['y2'], other['y2']) - max(det['y1'], other['y1'])
            if inter_w <= 0.0 or inter_h <= 0.0:
                continue
            inter = inter_w * inter_h
            other_area = (other['x2'] - other['x1']) * (other['y2'] - other['y1'])
            union = box_area + other_area - inter
            smaller = min(box_area, other_area)
            iou = inter / union if union > 0.0 else 1.0
            # IoM catches boxes nested inside a larger one, where plain IoU stays low.
            iom = inter / smaller if smaller > 0.0 else 1.0
            if max(iou, iom) >= threshold:
                keep = False
                break
        if keep:
            kept.append(det)
    return kept


class _Slot(object):
    def __init__(self, engine):
        in_shape = engine.input_shape
        self.pin_in_ptr = self._host_buffer(int(np.prod(in_shape)) * 4)
        self.pin_in = np.ctypeslib.as_array(
            ctypes.cast(self.pin_in_ptr, ctypes.POINTER(ctypes.c_float)),
            shape=(int(np.prod(in_shape)),)).reshape(in_shape)
        out_shape = engine.output_shape
        self.pin_out_ptr = self._host_buffer(int(np.prod(out_shape)) * 4)
        self.pin_out = np.ctypeslib.as_array(
            ctypes.cast(self.pin_out_ptr, ctypes.POINTER(ctypes.c_float)),
            shape=(int(np.prod(out_shape)),)).reshape(out_shape)
        self.dev_in = ctypes.c_void_p()
        _cuda_check(_CUDART.cudaMalloc(ctypes.byref(self.dev_in),
                                       self.pin_in.nbytes), 'cudaMalloc input')
        self.dev_out = ctypes.c_void_p()
        _cuda_check(_CUDART.cudaMalloc(ctypes.byref(self.dev_out),
                                       self.pin_out.nbytes), 'cudaMalloc output')
        self.ready = self._event()
        self.start = self._event()
        self.done = self._event()
        self.pad_keys = [None] * engine.batch
        self.clear()

    @staticmethod
    def _host_buffer(size):
        pointer = ctypes.c_void_p()
        _cuda_check(_CUDART.cudaHostAlloc(ctypes.byref(pointer), size, 0),
                    'cudaHostAlloc')
        if not pointer.value:
            raise RuntimeError('cudaHostAlloc returned a null pointer')
        return pointer

    @staticmethod
    def _event():
        event = ctypes.c_void_p()
        _cuda_check(_CUDART.cudaEventCreate(ctypes.byref(event)),
                    'cudaEventCreate')
        return event

    def clear(self):
        self.real_count = 0
        self.frames = []
        self.messages = []
        self.geometries = []
        self.sequences = []

    def release(self):
        for event_name in ('ready', 'start', 'done'):
            event = getattr(self, event_name, None)
            if event:
                _CUDART.cudaEventDestroy(event)
                setattr(self, event_name, None)
        for name in ('dev_in', 'dev_out'):
            pointer = getattr(self, name, None)
            if pointer:
                _CUDART.cudaFree(pointer)
                setattr(self, name, None)
        for name in ('pin_in_ptr', 'pin_out_ptr'):
            pointer = getattr(self, name, None)
            if pointer:
                _CUDART.cudaFreeHost(pointer)
                setattr(self, name, None)
        self.pin_in = None
        self.pin_out = None


class YoloEngine(object):
    """TensorRT YOLO engine with one execution context and pinned buffers per slot."""

    def __init__(self, engine_path, slot_count=_SLOT_COUNT, context_count=3):
        engine_bytes, self.metadata = _read_engine_file(engine_path)
        trt.init_libnvinfer_plugins(None, '')
        logger = trt.Logger(trt.Logger.WARNING)
        self.engine = trt.Runtime(logger).deserialize_cuda_engine(engine_bytes)
        if self.engine is None:
            raise RuntimeError('TensorRT could not deserialize %s; '
                               'engine and runtime versions must match' % engine_path)
        if not hasattr(self.engine, 'num_bindings'):
            raise RuntimeError('this node requires the TensorRT 8 binding API, found %s'
                               % trt.__version__)

        input_name = None
        output_name = None
        input_shape = None
        output_shape = None
        input_dtype = None
        output_dtype = None
        for index in range(self.engine.num_bindings):
            name = self.engine.get_tensor_name(index)
            is_input = self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT
            shape = tuple(self.engine.get_tensor_shape(name))
            dtype = np.dtype(trt.nptype(self.engine.get_tensor_dtype(name)))
            if is_input:
                if input_name is not None:
                    raise RuntimeError('engine must have exactly one input')
                input_name, input_shape, input_dtype = name, shape, dtype
            else:
                if output_name is not None:
                    raise RuntimeError('engine must have exactly one output')
                output_name, output_shape, output_dtype = name, shape, dtype
        if input_name is None or output_name is None:
            raise RuntimeError('engine is missing an input or output binding')
        if len(input_shape) != 4 or any(dim <= 0 for dim in input_shape):
            raise RuntimeError('engine input must be a static NCHW tensor, got %s'
                               % (input_shape,))
        if input_shape[1] != 3:
            raise RuntimeError('engine input must have three channels, got %s'
                               % (input_shape,))
        if len(output_shape) != 3 or output_shape[2] != 6:
            raise RuntimeError('engine output must be (batch, max_det, 6), got %s'
                               % (output_shape,))
        if output_shape[0] != input_shape[0]:
            raise RuntimeError('engine input/output batch sizes differ')
        if input_dtype != np.float32 or output_dtype != np.float32:
            raise RuntimeError('this node expects float32 engine bindings')

        self.input_name = input_name
        self.output_name = output_name
        self.batch = input_shape[0]
        self.input_shape = input_shape
        self.output_shape = output_shape
        self.input_height = input_shape[2]
        self.input_width = input_shape[3]

        self.compute_stream = ctypes.c_void_p()
        _cuda_check(_CUDART.cudaStreamCreate(ctypes.byref(self.compute_stream)),
                    'cudaStreamCreate compute')
        self.copy_stream = ctypes.c_void_p()
        _cuda_check(_CUDART.cudaStreamCreate(ctypes.byref(self.copy_stream)),
                    'cudaStreamCreate copy')

        self.slots = [_Slot(self) for _ in range(slot_count)]
        self.contexts = []
        for _ in range(context_count):
            context = self.engine.create_execution_context()
            if context is None:
                raise RuntimeError('failed to create TensorRT execution context')
            self.contexts.append(context)

        names = self.metadata.get('names') or {}
        self.labels = {int(key): str(value) for key, value in names.items()}

    def preprocess(self, slot, index, frame):
        ratio, left, top, resized_width, resized_height = _geometry(
            frame.shape, self.input_width, self.input_height)
        key = (left, top, resized_width, resized_height)
        if slot.pad_keys[index] != key:
            slot.pin_in[index].fill(_PAD_VALUE)
            slot.pad_keys[index] = key
        if (resized_width, resized_height) != (frame.shape[1], frame.shape[0]):
            resized = cv2.resize(frame, (resized_width, resized_height),
                                 interpolation=cv2.INTER_LINEAR)
        else:
            resized = frame
        blob = cv2.dnn.blobFromImage(
            resized, 1.0 / 255.0, (resized_width, resized_height),
            (0, 0, 0), swapRB=True, crop=False)
        slot.pin_in[index:index + 1, :, top:top + resized_height,
                    left:left + resized_width] = blob
        return ratio, left, top, frame.shape[1], frame.shape[0]

    def pad_slot(self, slot):
        if slot.real_count < self.batch:
            for index in range(slot.real_count, self.batch):
                slot.pin_in[index] = slot.pin_in[slot.real_count - 1]

    def launch(self, slot, context):
        """Enqueue one batch. Note: this engine's NMS plugin blocks until it finishes."""
        if not context.set_tensor_address(self.input_name, int(slot.dev_in.value)):
            raise RuntimeError('failed to bind TensorRT input %s' % self.input_name)
        if not context.set_tensor_address(self.output_name, int(slot.dev_out.value)):
            raise RuntimeError('failed to bind TensorRT output %s' % self.output_name)
        _cuda_check(_CUDART.cudaMemcpyAsync(
            slot.dev_in, slot.pin_in_ptr, slot.pin_in.nbytes,
            _HOST_TO_DEVICE, self.copy_stream), 'cudaMemcpyAsync input')
        _cuda_check(_CUDART.cudaEventRecord(slot.ready, self.copy_stream),
                    'cudaEventRecord ready')
        _cuda_check(_CUDART.cudaStreamWaitEvent(self.compute_stream,
                                                slot.ready, 0),
                    'cudaStreamWaitEvent')
        _cuda_check(_CUDART.cudaEventRecord(slot.start, self.compute_stream),
                    'cudaEventRecord start')
        if not context.execute_async_v3(int(self.compute_stream.value)):
            raise RuntimeError('TensorRT execute_async_v3 failed')
        _cuda_check(_CUDART.cudaMemcpyAsync(
            slot.pin_out_ptr, slot.dev_out, slot.pin_out.nbytes,
            _DEVICE_TO_HOST, self.compute_stream), 'cudaMemcpyAsync output')
        _cuda_check(_CUDART.cudaEventRecord(slot.done, self.compute_stream),
                    'cudaEventRecord done')

    def finish(self, slot, conf_thres, nms_iou):
        _cuda_check(_CUDART.cudaEventSynchronize(slot.done),
                    'cudaEventSynchronize done')
        elapsed = ctypes.c_float()
        _cuda_check(_CUDART.cudaEventElapsedTime(
            ctypes.byref(elapsed), slot.start, slot.done),
            'cudaEventElapsedTime')
        results = []
        for index in range(slot.real_count):
            ratio, left, top, width, height = slot.geometries[index]
            results.append(self._decode_frame(
                slot.pin_out[index], ratio, left, top, width, height,
                conf_thres, nms_iou))
        return results, elapsed.value

    def _decode_frame(self, rows, ratio, left, top, width, height, conf_thres,
                      nms_iou):
        detections = []
        for row in rows:
            score = float(row[4])
            if not math.isfinite(score) or score < conf_thres:
                continue
            x1 = (float(row[0]) - left) / ratio
            y1 = (float(row[1]) - top) / ratio
            x2 = (float(row[2]) - left) / ratio
            y2 = (float(row[3]) - top) / ratio
            x1 = min(max(x1, 0.0), width - 1.0)
            y1 = min(max(y1, 0.0), height - 1.0)
            x2 = min(max(x2, 0.0), width - 1.0)
            y2 = min(max(y2, 0.0), height - 1.0)
            if x2 <= x1 or y2 <= y1:
                continue
            class_id = int(round(float(row[5])))
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            detections.append({
                'class_id': class_id,
                'label': self.labels.get(class_id, 'class_%d' % class_id),
                'confidence': round(score, 3),
                'x1': round(x1, 1),
                'y1': round(y1, 1),
                'x2': round(x2, 1),
                'y2': round(y2, 1),
                'cx': round(center_x, 1),
                'cy': round(center_y, 1),
                'width': round(x2 - x1, 1),
                'height': round(y2 - y1, 1),
                'norm_cx': round(center_x / width, 4),
                'norm_cy': round(center_y / height, 4),
            })
        detections.sort(key=lambda det: det['confidence'], reverse=True)
        if nms_iou < 1.0:
            detections = _suppress_overlaps(detections, nms_iou)
        return detections

    def warmup(self, iterations=2):
        slot = self.slots[0]
        slot.clear()
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        for index in range(self.batch):
            self.preprocess(slot, index, dummy)
        slot.real_count = 0
        for _ in range(iterations):
            self.launch(slot, self.contexts[0])
            _cuda_check(_CUDART.cudaEventSynchronize(slot.done),
                        'cudaEventSynchronize warmup')
        slot.clear()

    def close(self):
        try:
            _CUDART.cudaDeviceSynchronize()
        except Exception:
            pass
        for slot in getattr(self, 'slots', []):
            slot.release()
        self.slots = []
        self.contexts = []
        for name in ('compute_stream', 'copy_stream'):
            stream = getattr(self, name, None)
            if stream:
                _CUDART.cudaStreamDestroy(stream)
                setattr(self, name, None)
        self.engine = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


class YoloTrtNode(object):
    def __init__(self):
        self.engine_path = rospy.get_param(
            '~engine_path', '/home/nvidia/yolo/best.engine')
        self.image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')
        self.preview_topic = rospy.get_param('~preview_topic', '/yolo/preview/image')
        self.detections_topic = rospy.get_param('~detections_topic', '/yolo/detections')
        self.conf_thres = float(rospy.get_param('~conf_thres', 0.60))
        if not math.isfinite(self.conf_thres) or not 0.0 <= self.conf_thres <= 1.0:
            raise ValueError('conf_thres must be in [0, 1]')
        self.nms_iou = float(rospy.get_param('~nms_iou', 0.60))
        if not math.isfinite(self.nms_iou) or not 0.05 <= self.nms_iou <= 1.0:
            raise ValueError('nms_iou must be in [0.05, 1] (1.0 disables overlap '
                             'suppression outside the engine)')
        self.batch_timeout = float(rospy.get_param('~batch_timeout', 0.1))
        if not math.isfinite(self.batch_timeout) or not 0.01 <= self.batch_timeout <= 1.0:
            raise ValueError('batch_timeout must be in [0.01, 1] seconds')
        self.infer_threads = int(rospy.get_param('~infer_threads', 3))
        if not 1 <= self.infer_threads <= 4:
            raise ValueError('infer_threads must be in [1, 4]')
        self.log_period = float(rospy.get_param('~log_period', 1.0))
        if not math.isfinite(self.log_period) or self.log_period < 0:
            raise ValueError('log_period must be non-negative')
        self.display = bool(rospy.get_param('~show_window', True))
        self.display_fps_target = float(rospy.get_param('~display_fps', 10.0))
        if (not math.isfinite(self.display_fps_target)
                or not 1.0 <= self.display_fps_target <= 120.0):
            raise ValueError('display_fps must be in [1, 120]')
        self.window_name = str(rospy.get_param('~window_name', 'YOLO TensorRT'))
        self.window_width = int(rospy.get_param('~window_width', 640))
        self.window_height = int(rospy.get_param('~window_height', 480))
        if self.window_width < 160 or self.window_height < 120:
            raise ValueError('window size is too small')

        self.detector = YoloEngine(self.engine_path,
                                   context_count=self.infer_threads)
        if self.conf_thres < 0.25:
            rospy.logwarn('conf_thres %.2f is below the 0.25 threshold baked into '
                          'the exported engine; lower scores cannot be recovered',
                          self.conf_thres)
        self.bridge_in = CvBridge()
        self.bridge_out = CvBridge()
        try:
            self.bridge_out.cv2_to_imgmsg(np.zeros((2, 2, 3), dtype=np.uint8),
                                          encoding='bgr8')
        except Exception as exc:
            raise RuntimeError(
                'cv_bridge is incompatible with OpenCV %s (%s); run the node with '
                'PYTHONNOUSERSITE=1 to use the system OpenCV 4.2'
                % (cv2.__version__, exc))

        self.stop = threading.Event()
        self.frame_cond = threading.Condition()
        self.frame_queue = deque(maxlen=8)
        self.frame_sequence = 0
        self.last_frame_time = None
        self.camera_fps = 0.0
        self.ready_slots = Queue()
        self.free_slots = Queue()
        self.output_queue = Queue()
        for slot in self.detector.slots:
            self.free_slots.put(slot)
        self.display_cond = threading.Condition()
        self.display_queue = deque(maxlen=8)
        self.display_fps = 0.0
        self.engine_fps = 0.0
        self.processed_fps = 0.0
        self.prep_thread = None
        self.infer_workers = []
        self.output_thread = None
        self.closed = False
        self._stats_lock = threading.Lock()
        self._stats_time = time.monotonic()
        self._stats_engine_frames = 0
        self._stats_real_frames = 0
        self._stats_batches = 0
        self._stats_prep_ms = 0.0
        self._stats_emit_ms = 0.0
        self._last_batch_ms = 0.0
        self._last_detections = []

        self.preview_pub = rospy.Publisher(self.preview_topic, Image, queue_size=1)
        self.detections_pub = rospy.Publisher(self.detections_topic, String,
                                              queue_size=1)
        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self._on_image, queue_size=1,
            buff_size=16 * 1024 * 1024)
        if self.display and not os.environ.get('DISPLAY'):
            rospy.logwarn('show_window is set but DISPLAY is empty; '
                          'preview window disabled')
            self.display = False
        rospy.loginfo('YOLO TensorRT: %s | input %dx%d batch %d | output %dx6 | '
                      '%d classes | conf >= %.2f | overlap suppress >= %.2f',
                      self.engine_path, self.detector.input_width,
                      self.detector.input_height, self.detector.batch,
                      self.detector.output_shape[1], len(self.detector.labels),
                      self.conf_thres, self.nms_iou)
        rospy.loginfo('pipeline: %d slots, %d infer workers, batch timeout %.0f ms, '
                      'display %.0f FPS | %s -> detections %s, preview %s',
                      len(self.detector.slots), self.infer_threads,
                      self.batch_timeout * 1000.0, self.display_fps_target,
                      self.image_topic, self.detections_topic, self.preview_topic)

    def _on_image(self, message):
        now = time.monotonic()
        with self.frame_cond:
            self.frame_sequence += 1
            self.frame_queue.append((self.frame_sequence, now, message))
            if self.last_frame_time is not None:
                delta = now - self.last_frame_time
                if delta > 0:
                    instant = 1.0 / delta
                    self.camera_fps = (instant if self.camera_fps == 0.0
                                       else 0.9 * self.camera_fps + 0.1 * instant)
            self.last_frame_time = now
            self.frame_cond.notify()

    def _take_frame(self, timeout):
        with self.frame_cond:
            if not self.frame_queue and not self.stop.is_set():
                self.frame_cond.wait(timeout)
            if self.stop.is_set() or not self.frame_queue:
                return None
            return self.frame_queue.popleft()

    def _prep_loop(self):
        batch = self.detector.batch
        while not self.stop.is_set():
            try:
                slot = self.free_slots.get(timeout=0.1)
            except Empty:
                continue
            slot.clear()
            count = 0
            batch_start = None
            while not self.stop.is_set() and count < batch:
                if batch_start is None:
                    item = self._take_frame(0.2)
                else:
                    remaining = self.batch_timeout - (time.monotonic() - batch_start)
                    item = self._take_frame(max(0.0, min(remaining, 0.2)))
                if item is None:
                    if batch_start is not None:
                        break
                    continue
                sequence, arrival, message = item
                if batch_start is None:
                    batch_start = arrival
                try:
                    started = time.perf_counter()
                    frame = self.bridge_in.imgmsg_to_cv2(
                        message, desired_encoding='bgr8')
                    geometry = self.detector.preprocess(slot, count, frame)
                    prep_ms = (time.perf_counter() - started) * 1000.0
                    with self._stats_lock:
                        self._stats_prep_ms += prep_ms
                except Exception as exc:
                    rospy.logwarn_throttle(5.0, 'preprocess failed: %s', exc)
                    continue
                slot.frames.append(frame)
                slot.messages.append(message)
                slot.geometries.append(geometry)
                slot.sequences.append(sequence)
                count += 1
            if self.stop.is_set():
                self.free_slots.put(slot)
                return
            if count == 0:
                self.free_slots.put(slot)
                continue
            slot.real_count = count
            self.detector.pad_slot(slot)
            self.ready_slots.put(slot)

    def _infer_worker(self, index):
        context = self.detector.contexts[index]
        while not self.stop.is_set():
            try:
                slot = self.ready_slots.get(timeout=0.1)
            except Empty:
                continue
            try:
                self.detector.launch(slot, context)
                detections, batch_ms = self.detector.finish(
                    slot, self.conf_thres, self.nms_iou)
                self.output_queue.put((slot, detections, batch_ms))
            except Exception as exc:
                rospy.logerr('TensorRT pipeline failed: %s', exc)
                try:
                    _CUDART.cudaDeviceSynchronize()
                except Exception:
                    pass
                self.stop.set()
                with self.frame_cond:
                    self.frame_cond.notify_all()
                with self.display_cond:
                    self.display_cond.notify_all()
                self.free_slots.put(slot)
                return

    def _output_loop(self):
        while not self.stop.is_set():
            try:
                slot, detections, batch_ms = self.output_queue.get(timeout=0.1)
            except Empty:
                continue
            try:
                self._emit(slot, detections, batch_ms)
            except Exception as exc:
                rospy.logwarn_throttle(5.0, 'output failed: %s', exc)
            finally:
                self.free_slots.put(slot)

    def _emit(self, slot, detections_per_frame, batch_ms):
        started = time.perf_counter()
        for index in range(slot.real_count):
            detections = detections_per_frame[index]
            annotated = self._annotate(slot.frames[index], detections, batch_ms)
            self._publish(slot, index, annotated, detections, batch_ms)
            if self.display:
                with self.display_cond:
                    self.display_queue.append(annotated)
                    self.display_cond.notify()
        emit_ms = (time.perf_counter() - started) * 1000.0
        with self._stats_lock:
            self._stats_engine_frames += self.detector.batch
            self._stats_real_frames += slot.real_count
            self._stats_batches += 1
            self._stats_emit_ms += emit_ms
            self._last_batch_ms = batch_ms
            self._last_detections = (detections_per_frame[-1]
                                     if detections_per_frame else [])
        self._maybe_log()

    def _publish(self, slot, index, annotated, detections, batch_ms):
        message = slot.messages[index]
        preview = self.bridge_out.cv2_to_imgmsg(annotated, encoding='bgr8')
        preview.header = copy.deepcopy(message.header)
        self.preview_pub.publish(preview)
        height, width = annotated.shape[:2]
        payload = {
            'stamp': message.header.stamp.to_sec(),
            'frame_id': message.header.frame_id,
            'sequence': slot.sequences[index],
            'image_width': int(width),
            'image_height': int(height),
            'batch_size': self.detector.batch,
            'batch_inference_ms': round(batch_ms, 2),
            'inference_ms': round(batch_ms / self.detector.batch, 2),
            'processed_fps': round(self.processed_fps, 2),
            'camera_fps': round(self.camera_fps, 2),
            'count': len(detections),
            'detections': detections,
        }
        self.detections_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False,
                                   separators=(',', ':'))))

    def _maybe_log(self):
        with self._stats_lock:
            now = time.monotonic()
            if now - self._stats_time < self.log_period:
                return
            elapsed = now - self._stats_time
            engine_fps = self._stats_engine_frames / elapsed
            processed_fps = self._stats_real_frames / elapsed
            prep_ms = self._stats_prep_ms / max(self._stats_real_frames, 1)
            emit_ms = self._stats_emit_ms / max(self._stats_batches, 1)
            self.engine_fps = engine_fps
            self.processed_fps = processed_fps
            detections = self._last_detections
            batch_ms = self._last_batch_ms
            self._stats_time = now
            self._stats_engine_frames = 0
            self._stats_real_frames = 0
            self._stats_batches = 0
            self._stats_prep_ms = 0.0
            self._stats_emit_ms = 0.0
        summary = []
        for det in detections[:8]:
            summary.append('%s %.2f bbox=(%.0f,%.0f,%.0f,%.0f) center=(%.0f,%.0f)'
                           % (det['label'], det['confidence'],
                              det['x1'], det['y1'], det['x2'], det['y2'],
                              det['cx'], det['cy']))
        rospy.loginfo('engine %.1f FPS | processed %.1f FPS | camera %.1f FPS | '
                      'display %.1f FPS | batch %.1f ms | prep %.1f ms/frame | '
                      'emit %.1f ms/batch | dets %d%s',
                      engine_fps, processed_fps, self.camera_fps,
                      self.display_fps, batch_ms, prep_ms, emit_ms,
                      len(detections),
                      (' | ' + ' | '.join(summary)) if summary else '')

    def _annotate(self, frame, detections, batch_ms):
        canvas = frame.copy()
        height, width = canvas.shape[:2]
        for det in detections:
            color = _CLASS_COLORS[det['class_id'] % len(_CLASS_COLORS)]
            p1 = (int(round(det['x1'])), int(round(det['y1'])))
            p2 = (int(round(det['x2'])), int(round(det['y2'])))
            cv2.rectangle(canvas, p1, p2, color, 2)
            cv2.circle(canvas, (int(round(det['cx'])), int(round(det['cy']))),
                       3, color, -1)
            text = '%s %.0f%%' % (det['label'], det['confidence'] * 100.0)
            (text_w, text_h), baseline = cv2.getTextSize(
                text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            top = max(p1[1] - text_h - baseline - 4, 0)
            cv2.rectangle(canvas, (p1[0], top),
                          (p1[0] + text_w + 6, top + text_h + baseline + 4),
                          color, -1)
            luminance = 0.114 * color[0] + 0.587 * color[1] + 0.299 * color[2]
            text_color = (0, 0, 0) if luminance > 150 else (255, 255, 255)
            cv2.putText(canvas, text, (p1[0] + 3, top + text_h + 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1, cv2.LINE_AA)

        lines = ['TensorRT %.1f FPS  batch %.1f ms'
                 % (self.engine_fps, batch_ms),
                 'camera %.1f  display %.1f  dets %d'
                 % (self.camera_fps, self.display_fps, len(detections))]
        for det in detections[:5]:
            lines.append('%s %.2f  [%d,%d,%d,%d]  c=(%d,%d)'
                         % (det['label'], det['confidence'],
                            int(round(det['x1'])), int(round(det['y1'])),
                            int(round(det['x2'])), int(round(det['y2'])),
                            int(round(det['cx'])), int(round(det['cy']))))
        line_height = 17
        panel_width = 20 + max(
            cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)[0][0]
            for line in lines)
        panel_width = min(width, panel_width)
        panel_height = 8 + line_height * len(lines)
        cv2.rectangle(canvas, (0, 0), (panel_width, panel_height), (24, 24, 24), -1)
        for index, line in enumerate(lines):
            cv2.putText(canvas, line, (8, 20 + line_height * index),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1,
                        cv2.LINE_AA)
        return canvas

    def _display_loop(self):
        period = 1.0 / self.display_fps_target
        next_tick = time.monotonic()
        shown = 0
        stats_time = next_tick
        while not rospy.is_shutdown() and not self.stop.is_set():
            now = time.monotonic()
            if now < next_tick:
                time.sleep(min(next_tick - now, 0.005))
                continue
            next_tick += period
            if next_tick < now:
                next_tick = now + period
            frame = None
            with self.display_cond:
                if self.display_queue:
                    frame = self.display_queue.popleft()
            if self.display and frame is not None:
                try:
                    cv2.imshow(self.window_name, frame)
                    cv2.waitKey(1)
                    shown += 1
                except cv2.error as exc:
                    rospy.logwarn('preview window disabled: %s', exc)
                    self.display = False
            elif self.display:
                try:
                    cv2.waitKey(1)
                except cv2.error as exc:
                    rospy.logwarn('preview window disabled: %s', exc)
                    self.display = False
            now = time.monotonic()
            if now - stats_time >= 1.0:
                self.display_fps = shown / (now - stats_time)
                shown = 0
                stats_time = now

    def run(self):
        self.detector.warmup()
        rospy.loginfo('TensorRT warm-up finished; starting pipeline threads')
        if self.display:
            try:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, self.window_width,
                                 self.window_height)
            except cv2.error as exc:
                rospy.logwarn('cannot create preview window: %s', exc)
                self.display = False
        self.prep_thread = threading.Thread(
            target=self._prep_loop, name='yolo_prep', daemon=True)
        self.prep_thread.start()
        self.output_thread = threading.Thread(
            target=self._output_loop, name='yolo_output', daemon=True)
        self.output_thread.start()
        for index in range(self.infer_threads):
            worker = threading.Thread(
                target=self._infer_worker, args=(index,),
                name='yolo_infer_%d' % index, daemon=True)
            worker.start()
            self.infer_workers.append(worker)
        if self.display:
            self._display_loop()
        else:
            while not rospy.is_shutdown() and not self.stop.is_set():
                time.sleep(0.1)

    def shutdown(self):
        if self.closed:
            return
        self.closed = True
        self.stop.set()
        with self.frame_cond:
            self.frame_cond.notify_all()
        with self.display_cond:
            self.display_cond.notify_all()
        threads = [self.prep_thread, self.output_thread] + self.infer_workers
        for thread in threads:
            if thread is not None and thread.is_alive():
                thread.join(timeout=3.0)
                if thread.is_alive():
                    rospy.logwarn('thread %s did not stop in time', thread.name)
        self.detector.close()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass


def main():
    rospy.init_node('yolo_trt_detector')
    node = YoloTrtNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
