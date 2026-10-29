#!/usr/bin/env python3
"""Offline camera and image-writing checks without a camera, GUI, or ROS master."""

import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest import mock

import cv2
import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / 'camera_yolo_capture.py'
SPEC = importlib.util.spec_from_file_location('camera_yolo_capture', SCRIPT)
camera = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(camera)


class Clock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now


class Capture:
    def __init__(self, clock, fps=120, interval=None, width=1280, height=720):
        self.clock = clock
        self.interval = interval or 1.0 / fps
        self.frame = np.zeros((height, width, 3), dtype=np.uint8)
        self.properties = {cv2.CAP_PROP_FRAME_WIDTH: width, cv2.CAP_PROP_FRAME_HEIGHT: height,
                           cv2.CAP_PROP_FPS: fps, cv2.CAP_PROP_FOURCC: cv2.VideoWriter_fourcc(*'MJPG'),
                           cv2.CAP_PROP_BUFFERSIZE: 2}
        self.released = False

    def isOpened(self):
        return True

    def set(self, prop, value):
        return True

    def get(self, prop):
        return self.properties[prop]

    def read(self):
        self.grab()
        return True, self.frame

    def grab(self):
        self.clock.now += self.interval
        return True

    def release(self):
        self.released = True


class Collector:
    def __init__(self):
        self.frames = []

    def check(self):
        pass

    def submit(self, frame, metadata):
        self.frames.append((frame, metadata))
        return True


class CameraTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.capture = Capture(self.clock)

    def test_defaults_request_120_and_capture_at_most_ten_seconds(self):
        args = camera.arguments([])
        self.assertEqual((args.fps, args.duration, args.mode), (120, 10, 'preview'))
        self.assertEqual((args.width, args.height), (1280, 720))
        self.assertEqual(args.save_fps, 0)
        self.assertEqual(args.buffer_size, 2)

    def test_bad_arguments_rejected(self):
        for argv in (['--fps', 'nan'], ['--fps', '0'], ['--duration', '11'], ['--duration', '-1'],
                     ['--save-fps', '121'], ['--warmup', 'inf'], ['--height-m', '-1'],
                     ['--queue-size', '0'], ['--width', '0'], ['--fourcc', 'BAD'],
                     ['--grab-only'], ['--mode', 'capture', '--grab-only'],
                     ['--mode', 'benchmark', '--preview']):
            with self.subTest(argv=argv), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    camera.arguments(argv)

    def test_open_explicitly_requests_120_fps_and_double_buffer(self):
        args = camera.arguments([])
        with mock.patch.object(cv2, 'VideoCapture', return_value=self.capture), mock.patch.object(self.capture, 'set', wraps=self.capture.set) as setter:
            capture, info = camera.open_camera(cv2, args)
        self.assertIs(capture, self.capture)
        self.assertEqual(info['actual']['fps'], 120)
        self.assertEqual(info['actual']['fourcc'], 'MJPG')
        setter.assert_any_call(cv2.CAP_PROP_FPS, 120)
        setter.assert_any_call(cv2.CAP_PROP_BUFFERSIZE, 2)

    def test_explicit_1080p_request_keeps_native_frame_size(self):
        args = camera.arguments(['--mode', 'capture', '--width', '1920', '--height', '1080'])
        device = Capture(self.clock, width=1920, height=1080)
        writer = Collector()
        with mock.patch.object(cv2, 'VideoCapture', return_value=device), mock.patch.object(device, 'set', wraps=device.set) as setter:
            capture, info = camera.open_camera(cv2, args)
        setter.assert_any_call(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        setter.assert_any_call(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.assertEqual(info['actual']['fps'], 120)
        with mock.patch.object(camera.time, 'monotonic', self.clock.monotonic):
            camera.stream_camera(cv2, capture, args, writer)
        self.assertTrue(writer.frames)
        self.assertEqual(writer.frames[0][0].shape, (1080, 1920, 3))

    def test_negotiated_fallback_rejected_and_camera_released(self):
        for reported in (30, 0):
            with self.subTest(reported=reported):
                self.capture.properties[cv2.CAP_PROP_FPS] = reported
                with mock.patch.object(cv2, 'VideoCapture', return_value=self.capture):
                    with self.assertRaisesRegex(RuntimeError, '帧率协商未达到'):
                        camera.open_camera(cv2, camera.arguments([]))
                self.assertTrue(self.capture.released)

    def test_fps_fallback_requires_explicit_opt_in(self):
        self.capture.properties[cv2.CAP_PROP_FPS] = 30
        with mock.patch.object(cv2, 'VideoCapture', return_value=self.capture):
            _, info = camera.open_camera(cv2, camera.arguments(['--allow-fps-fallback']))
        self.assertEqual(info['requested']['fps'], 120)
        self.assertEqual(info['actual']['fps'], 30)

    def test_wrong_resolution_rejected(self):
        self.capture.properties[cv2.CAP_PROP_FRAME_WIDTH] = 320
        with mock.patch.object(cv2, 'VideoCapture', return_value=self.capture):
            with self.assertRaisesRegex(RuntimeError, '分辨率协商不匹配'):
                camera.open_camera(cv2, camera.arguments([]))
        self.assertTrue(self.capture.released)

    def test_ten_second_window_saves_each_received_frame_once(self):
        writer = Collector()
        args = camera.arguments(['--mode', 'capture', '--height-m', '0.8'])
        with mock.patch.object(camera.time, 'monotonic', self.clock.monotonic), contextlib.redirect_stdout(io.StringIO()):
            stats = camera.stream_camera(cv2, self.capture, args, writer)
        self.assertAlmostEqual(stats['window_elapsed_s'], 10)
        self.assertAlmostEqual(stats['read_fps'], 120)
        self.assertEqual(stats['received'], len(writer.frames))
        self.assertGreaterEqual(stats['received'], 1199)
        self.assertLessEqual(stats['received'], 1200)
        self.assertEqual(len({metadata['sequence'] for _, metadata in writer.frames}), stats['received'])
        for _, metadata in writer.frames:
            self.assertLess(metadata['elapsed_s'], 10)
            self.assertEqual(metadata['tagged_height_m'], 0.8)

    def test_slow_camera_is_not_upsampled(self):
        capture = Capture(self.clock, fps=30)
        writer = Collector()
        with mock.patch.object(camera.time, 'monotonic', self.clock.monotonic), contextlib.redirect_stdout(io.StringIO()):
            stats = camera.stream_camera(cv2, capture, camera.arguments(['--mode', 'capture']), writer)
        self.assertAlmostEqual(stats['read_fps'], 30)
        self.assertLessEqual(len(writer.frames), 300)

    def test_no_image_returned_after_deadline_is_saved(self):
        self.capture.interval = 11
        writer = Collector()
        with mock.patch.object(camera.time, 'monotonic', self.clock.monotonic):
            stats = camera.stream_camera(cv2, self.capture, camera.arguments(['--mode', 'capture']), writer)
        self.assertEqual(stats['received'], 0)
        self.assertEqual(stats['window_elapsed_s'], 10)
        self.assertEqual(writer.frames, [])

    def test_subsampling_and_queue_drops_are_counted_separately(self):
        writer = Collector()
        args = camera.arguments(['--mode', 'capture', '--duration', '1', '--save-fps', '10'])
        with mock.patch.object(camera.time, 'monotonic', self.clock.monotonic):
            stats = camera.stream_camera(cv2, self.capture, args, writer)
        self.assertLessEqual(len(writer.frames), 10)
        self.assertGreater(stats['rate_skipped'], 0)
        self.assertEqual(stats['queue_dropped'], 0)
        self.clock.now = 100
        writer.submit = mock.Mock(return_value=False)
        args.save_fps = 0
        with mock.patch.object(camera.time, 'monotonic', self.clock.monotonic):
            stats = camera.stream_camera(cv2, self.capture, args, writer)
        self.assertEqual(stats['queue_dropped'], stats['received'])
        self.assertEqual(stats['submitted'], 0)

    def test_preview_overlay_never_changes_saved_pixels(self):
        self.capture.interval = 0.04
        writer = Collector()
        args = camera.arguments(['--mode', 'capture', '--preview'])
        with mock.patch.object(camera.time, 'monotonic', self.clock.monotonic), mock.patch.object(cv2, 'imshow'), mock.patch.object(cv2, 'waitKey', return_value=ord('q')):
            stats = camera.stream_camera(cv2, self.capture, args, writer)
        self.assertEqual(stats['stop_reason'], 'user')
        self.assertEqual(len(writer.frames), 1)
        self.assertFalse(writer.frames[0][0].any())
        self.assertFalse(self.capture.frame.any())

    def test_five_read_failures_stop_capture(self):
        self.capture.read = mock.Mock(return_value=(False, None))
        with mock.patch.object(camera.time, 'monotonic', self.clock.monotonic), mock.patch.object(camera.time, 'sleep'):
            with self.assertRaisesRegex(RuntimeError, '连续五次读取失败'):
                camera.stream_camera(cv2, self.capture, camera.arguments(['--mode', 'capture']))

    def test_real_jpeg_and_metadata_are_saved_without_labels(self):
        with tempfile.TemporaryDirectory() as directory:
            session = camera.session_directory(Path(directory))
            writer = camera.ImageWriter(cv2, session, 95, 2)
            self.assertTrue(writer.submit(self.capture.frame, dict(elapsed_s=0.1)))
            writer.close()
            self.assertEqual(writer.saved, 1)
            image = cv2.imread(str(session / 'images/frame_000000.jpg'))
            self.assertEqual(image.shape, (720, 1280, 3))
            rows = (session / 'frames.jsonl').read_text().splitlines()
            self.assertEqual(len(rows), 1)
            self.assertEqual(json.loads(rows[0])['file'], 'images/frame_000000.jpg')
            self.assertFalse((session / 'labels').exists())
            self.assertNotEqual(camera.session_directory(Path(directory)), session)

    def test_bounded_writer_queue_drops_instead_of_blocking(self):
        entered, release = threading.Event(), threading.Event()

        def imwrite(*args):
            entered.set()
            return release.wait(2)

        with tempfile.TemporaryDirectory() as directory:
            session = camera.session_directory(Path(directory))
            writer_cv = SimpleNamespace(imwrite=imwrite, IMWRITE_JPEG_QUALITY=cv2.IMWRITE_JPEG_QUALITY)
            writer = camera.ImageWriter(writer_cv, session, 95, 1)
            try:
                self.assertTrue(writer.submit(self.capture.frame, {}))
                self.assertTrue(entered.wait(2))
                self.assertTrue(writer.submit(self.capture.frame, {}))
                self.assertFalse(writer.submit(self.capture.frame, {}))
            finally:
                release.set()
                writer.close()
            self.assertEqual(writer.saved, 2)

    def test_write_failure_is_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            session = camera.session_directory(Path(directory))
            writer_cv = SimpleNamespace(imwrite=lambda *args: False, IMWRITE_JPEG_QUALITY=cv2.IMWRITE_JPEG_QUALITY)
            writer = camera.ImageWriter(writer_cv, session, 95, 1)
            writer.submit(self.capture.frame, {})
            with self.assertRaisesRegex(RuntimeError, '落盘失败'):
                writer.close()
            self.assertEqual(writer.saved, 0)

    def test_headless_preview_rejected_before_opening_device(self):
        with mock.patch.dict(camera.os.environ, {}, clear=True), mock.patch.object(cv2, 'VideoCapture') as opener:
            self.assertEqual(camera.main([]), 1)
        opener.assert_not_called()

    def test_headless_benchmark_neither_displays_nor_writes_images(self):
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / 'must_not_exist'
            with mock.patch.dict(camera.os.environ, {}, clear=True), mock.patch.object(camera.time, 'monotonic', self.clock.monotonic), mock.patch.object(cv2, 'VideoCapture', return_value=self.capture), mock.patch.object(camera, 'ImageWriter') as writer, mock.patch.object(cv2, 'imshow') as display, contextlib.redirect_stdout(output):
                result = camera.main(['--mode', 'benchmark', '--warmup', '0', '--duration', '0.1', '--output', str(destination)])
            self.assertFalse(destination.exists())
        self.assertEqual(result, 0)
        writer.assert_not_called()
        display.assert_not_called()
        summary = json.loads(output.getvalue().split('结果：', 1)[1])
        self.assertEqual(summary['mode'], 'benchmark')
        self.assertEqual(summary['status'], 'complete')
        self.assertAlmostEqual(summary['read_fps'], 120)
        self.assertNotIn('saved', summary)
        self.assertTrue(self.capture.released)

    def test_grab_benchmark_and_warmup_do_not_retrieve_images(self):
        self.capture.read = mock.Mock(side_effect=AssertionError('must not decode'))
        output = io.StringIO()
        with mock.patch.dict(camera.os.environ, {}, clear=True), mock.patch.object(camera.time, 'monotonic', self.clock.monotonic), mock.patch.object(cv2, 'VideoCapture', return_value=self.capture), contextlib.redirect_stdout(output):
            result = camera.main(['--mode', 'benchmark', '--grab-only', '--warmup', '0.02', '--duration', '0.1'])
        self.assertEqual(result, 0)
        self.capture.read.assert_not_called()
        summary = json.loads(output.getvalue().split('结果：', 1)[1])
        self.assertAlmostEqual(summary['grab_fps'], 120)
        self.assertNotIn('read_fps', summary)
        self.assertTrue(summary['grab_only'])
        self.assertEqual(summary['timestamp_source'], 'host_after_grab')
        self.assertTrue(self.capture.released)

    def test_benchmarks_separate_slow_image_retrieval_from_grabbing(self):
        def slow_read():
            success, frame = Capture.read(self.capture)
            self.clock.now += self.capture.interval
            return success, frame

        self.capture.read = slow_read
        with mock.patch.object(camera.time, 'monotonic', self.clock.monotonic):
            decoded = camera.stream_camera(cv2, self.capture, camera.arguments(['--mode', 'benchmark', '--duration', '0.1']))
            grabbed = camera.stream_camera(cv2, self.capture, camera.arguments(['--mode', 'benchmark', '--grab-only', '--duration', '0.1']))
        self.assertAlmostEqual(decoded['read_fps'], 60)
        self.assertAlmostEqual(grabbed['grab_fps'], 120)
        self.assertAlmostEqual(decoded['receive_mean_ms'], 1000 / 60)
        self.assertAlmostEqual(grabbed['receive_mean_ms'], 1000 / 120)

    def test_grab_benchmark_respects_deadline(self):
        self.capture.interval = 11
        with mock.patch.object(camera.time, 'monotonic', self.clock.monotonic):
            stats = camera.stream_camera(cv2, self.capture, camera.arguments(['--mode', 'benchmark', '--grab-only']))
        self.assertEqual(stats['received'], 0)
        self.assertEqual(stats['grab_fps'], 0)

    def test_failed_grabs_stop_benchmark(self):
        self.capture.grab = mock.Mock(return_value=False)
        with mock.patch.object(camera.time, 'monotonic', self.clock.monotonic), mock.patch.object(camera.time, 'sleep'):
            with self.assertRaisesRegex(RuntimeError, '连续五次读取失败'):
                camera.stream_camera(cv2, self.capture, camera.arguments(['--mode', 'benchmark', '--grab-only']))

    def test_capture_main_writes_summary_and_releases_device(self):
        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.object(camera.time, 'monotonic', self.clock.monotonic), mock.patch.object(cv2, 'VideoCapture', return_value=self.capture), contextlib.redirect_stdout(io.StringIO()):
                result = camera.main(['--mode', 'capture', '--warmup', '0', '--duration', '0.1', '--output', directory])
            self.assertEqual(result, 0)
            summary = json.loads(next(Path(directory).glob('*/session.json')).read_text())
            self.assertEqual(summary['status'], 'complete')
            self.assertGreater(summary['saved'], 0)
            self.assertEqual(summary['received'], summary['saved'])
            self.assertTrue(self.capture.released)

    def test_capture_failure_preserves_statistics_and_marks_incomplete(self):
        self.capture.read = mock.Mock(return_value=(False, None))
        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.object(camera.time, 'monotonic', self.clock.monotonic), mock.patch.object(camera.time, 'sleep'), mock.patch.object(cv2, 'VideoCapture', return_value=self.capture), contextlib.redirect_stdout(io.StringIO()):
                result = camera.main(['--mode', 'capture', '--warmup', '0', '--output', directory])
            self.assertEqual(result, 1)
            summary = json.loads(next(Path(directory).glob('*/session.json')).read_text())
            self.assertEqual(summary['status'], 'failed')
            self.assertEqual(summary['stop_reason'], 'error')
            self.assertEqual(summary['read_failures'], 5)
            self.assertEqual(summary['saved'], 0)
            self.assertTrue(self.capture.released)

    def test_zero_frames_is_not_reported_as_success(self):
        self.capture.interval = 11
        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.object(camera.time, 'monotonic', self.clock.monotonic), mock.patch.object(cv2, 'VideoCapture', return_value=self.capture), contextlib.redirect_stdout(io.StringIO()):
                result = camera.main(['--mode', 'capture', '--warmup', '0', '--output', directory])
            self.assertEqual(result, 1)
            summary = json.loads(next(Path(directory).glob('*/session.json')).read_text())
            self.assertEqual(summary['status'], 'failed')
            self.assertEqual(summary['saved'], 0)


if __name__ == '__main__':
    unittest.main()
