#!/usr/bin/env python3
"""V4L2 camera preview, throughput checks and ten-second YOLO collection; no flight commands."""

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import queue
import sys
import threading
import time
import uuid


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('preview', 'capture', 'benchmark'), default='preview')
    parser.add_argument('--grab-only', action='store_true', help='benchmark grab() without retrieving decoded images')
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=float, default=120.0)
    parser.add_argument('--fourcc', default='MJPG')
    parser.add_argument('--buffer-size', type=int, default=2)
    parser.add_argument('--allow-fps-fallback', action='store_true', help='explicitly allow driver-reported FPS mismatch')
    parser.add_argument('--duration', type=float, default=10.0, help='capture window in seconds, at most 10')
    parser.add_argument('--warmup', type=float, default=1.0)
    parser.add_argument('--output', type=Path, default=Path.home() / 'vision_datasets/yolo_collect')
    parser.add_argument('--height-m', type=float, help='manually measured/tagged height, not an automatic altitude estimate')
    parser.add_argument('--save-fps', type=float, default=0.0, help='0 saves every received frame; positive values subsample')
    parser.add_argument('--jpeg-quality', type=int, default=95)
    parser.add_argument('--queue-size', type=int, default=32)
    parser.add_argument('--preview', action='store_true', help='also display during capture; never drawn into saved images')
    parser.add_argument('--preview-fps', type=float, default=30.0)
    args = parser.parse_args(argv)
    if args.grab_only and args.mode != 'benchmark':
        parser.error('--grab-only requires --mode benchmark')
    if args.mode == 'benchmark' and args.preview:
        parser.error('benchmark must run without --preview')
    for name in ('fps', 'duration', 'preview_fps'):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(name + ' must be finite and positive')
    if args.duration > 10:
        parser.error('duration must not exceed 10 seconds')
    if not math.isfinite(args.warmup) or not 0 <= args.warmup <= 10:
        parser.error('warmup must be in [0, 10] seconds')
    if not math.isfinite(args.save_fps) or not 0 <= args.save_fps <= args.fps:
        parser.error('save-fps must be in [0, fps]')
    if args.height_m is not None and (not math.isfinite(args.height_m) or args.height_m < 0):
        parser.error('height-m must be finite and nonnegative')
    if args.width <= 0 or args.height <= 0 or len(args.fourcc) != 4:
        parser.error('width/height must be positive and fourcc must contain four characters')
    if not 1 <= args.buffer_size <= 8 or not 1 <= args.queue_size <= 256 or not 1 <= args.jpeg_quality <= 100:
        parser.error('buffer-size in [1,8], queue-size in [1,256], jpeg-quality in [1,100] required')
    return args


def open_camera(cv, args):
    capture = cv.VideoCapture(args.device, cv.CAP_V4L2)
    try:
        if not capture.isOpened():
            raise RuntimeError('无法打开 %s；检查设备、权限以及是否已被其他相机节点占用' % args.device)
        settings = ((cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*args.fourcc), 'fourcc'),
                    (cv.CAP_PROP_FRAME_WIDTH, args.width, 'width'),
                    (cv.CAP_PROP_FRAME_HEIGHT, args.height, 'height'),
                    (cv.CAP_PROP_FPS, args.fps, 'fps'),
                    (cv.CAP_PROP_BUFFERSIZE, args.buffer_size, 'buffer_size'))
        accepted = {name: bool(capture.set(prop, value)) for prop, value, name in settings}
        actual = {name: capture.get(prop) for prop, _, name in settings}
        if not all(math.isfinite(value) for value in actual.values()):
            raise RuntimeError('相机返回非有限的协商参数')
        code = int(actual['fourcc'])
        actual['fourcc'] = ''.join(chr((code >> (8 * index)) & 255) for index in range(4)).strip('\x00')
        if actual['width'] != args.width or actual['height'] != args.height:
            raise RuntimeError('分辨率协商不匹配：请求 %dx%d，实际 %sx%s；请先用 v4l2-ctl 查询支持模式' %
                               (args.width, args.height, actual['width'], actual['height']))
        matched = math.isclose(actual['fps'], args.fps, rel_tol=0.02, abs_tol=0.5)
        if not matched and not args.allow_fps_fallback:
            raise RuntimeError('帧率协商未达到请求：请求 %.2f FPS，驱动报告 %.2f FPS；不会冒充 120 FPS。'
                               '检查支持模式；只做低帧率诊断可显式加 --allow-fps-fallback' % (args.fps, actual['fps']))
        if not matched:
            print('警告：已显式允许帧率回退；实际帧率仍须测量。', file=sys.stderr)
        info = dict(device=args.device, requested=dict(width=args.width, height=args.height, fps=args.fps,
                                                       fourcc=args.fourcc, buffer_size=args.buffer_size),
                    actual=actual, property_set_results=accepted)
        print('相机协商参数：' + json.dumps(info, ensure_ascii=False), flush=True)
        print('驱动报告的 FPS 不是实测 FPS；脚本不复制旧帧补足数量。', flush=True)
        return capture, info
    except BaseException:
        capture.release()
        raise


class ImageWriter:
    def __init__(self, cv, session, quality, capacity):
        self.cv, self.session, self.quality = cv, session, quality
        self.pending = queue.Queue(maxsize=capacity)
        self.stop = threading.Event()
        self.error = None
        self.saved = 0
        self.thread = threading.Thread(target=self.run, name='image-writer', daemon=True)
        self.thread.start()

    def check(self):
        if self.error is not None:
            raise RuntimeError('图像落盘失败：' + str(self.error))

    def submit(self, frame, metadata):
        self.check()
        try:
            self.pending.put_nowait((frame.copy(), metadata))
            return True
        except queue.Full:
            return False

    def run(self):
        try:
            with (self.session / 'frames.jsonl').open('x', encoding='utf-8') as stream:
                while not self.stop.is_set() or not self.pending.empty():
                    try:
                        frame, metadata = self.pending.get(timeout=0.05)
                    except queue.Empty:
                        continue
                    filename = 'frame_%06d.jpg' % self.saved
                    destination = self.session / 'images' / filename
                    if not self.cv.imwrite(str(destination), frame, [self.cv.IMWRITE_JPEG_QUALITY, self.quality]):
                        raise OSError('imwrite returned false: ' + str(destination))
                    metadata = dict(metadata, file='images/' + filename)
                    stream.write(json.dumps(metadata, ensure_ascii=False, allow_nan=False) + '\n')
                    self.saved += 1
        except Exception as error:
            self.error = error

    def close(self):
        self.stop.set()
        self.thread.join()
        self.check()


def session_directory(output):
    output = output.expanduser()
    session = output / (datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_' + uuid.uuid4().hex[:8])
    (session / 'images').mkdir(parents=True, exist_ok=False)
    return session


def warmup_camera(capture, duration, grab_only=False):
    deadline = time.monotonic() + duration
    failures = 0
    while time.monotonic() < deadline:
        success, frame = (capture.grab(), None) if grab_only else capture.read()
        if not success or (not grab_only and frame is None):
            failures += 1
            if failures >= 5:
                raise RuntimeError('相机预热期间连续五次读取失败')
            time.sleep(0.01)
        else:
            failures = 0


def stream_camera(cv, capture, args, writer=None, stats=None):
    started = time.monotonic()
    deadline = started + args.duration if args.mode != 'preview' else math.inf
    grab_only = args.mode == 'benchmark' and args.grab_only
    rate_name = 'grab_fps' if grab_only else 'read_fps'
    rate_label = '抓取（未取出解码图像）' if grab_only else '读取'
    if stats is None:
        stats = {}
    stats.update(received=0, submitted=0, queue_dropped=0, rate_skipped=0, read_failures=0,
                 stop_reason='duration' if args.mode != 'preview' else 'user')
    first_frame = last_frame = None
    last_preview = last_report = started
    next_save = started
    failures = 0
    receive_calls = 0
    receive_elapsed = receive_max = 0.0
    try:
        while time.monotonic() < deadline:
            if writer is not None:
                writer.check()
            receive_started = time.monotonic()
            success, frame = (capture.grab(), None) if grab_only else capture.read()
            now = time.monotonic()
            receive_duration = now - receive_started
            receive_calls += 1
            receive_elapsed += receive_duration
            receive_max = max(receive_max, receive_duration)
            if now >= deadline:
                break
            if not success or (not grab_only and frame is None):
                failures += 1
                stats['read_failures'] += 1
                if failures >= 5:
                    raise RuntimeError('连续五次读取失败；检查相机连接和带宽')
                time.sleep(0.01)
                continue
            failures = 0
            if not grab_only and frame.shape[:2] != (args.height, args.width):
                raise RuntimeError('实际图像尺寸与请求不匹配：' + str(frame.shape))
            stats['received'] += 1
            first_frame = now if first_frame is None else first_frame
            last_frame = now
            measured = (stats['received'] - 1) / (now - first_frame) if now > first_frame else 0.0
            if writer is not None:
                if args.save_fps == 0 or now + 1e-9 >= next_save:
                    metadata = dict(sequence=stats['received'], elapsed_s=now - started,
                                    received_unix_s=time.time(), width=frame.shape[1], height=frame.shape[0],
                                    tagged_height_m=args.height_m, timestamp_source='host_after_read')
                    if writer.submit(frame, metadata):
                        stats['submitted'] += 1
                    else:
                        stats['queue_dropped'] += 1
                    if args.save_fps:
                        next_save = now + 1.0 / args.save_fps
                else:
                    stats['rate_skipped'] += 1
            if args.mode == 'preview' or args.preview:
                if now - last_preview >= 1.0 / args.preview_fps:
                    display = frame.copy()
                    text = 'request %.0f FPS | read %.1f FPS | %.1fs' % (args.fps, measured, now - started)
                    cv.putText(display, text, (10, 24), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    if args.height_m is not None:
                        cv.putText(display, 'height TAG %.2fm (manual)' % args.height_m, (10, 48),
                                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv.imshow('Camera preview - q / ESC to quit', display)
                    if cv.waitKey(1) & 255 in (27, ord('q')):
                        stats['stop_reason'] = 'user'
                        break
                    last_preview = now
            if now - last_report >= 1:
                print('%s %.1f FPS；已接收 %d，入队 %d，队列丢帧 %d' %
                      (rate_label, measured, stats['received'], stats['submitted'], stats['queue_dropped']), flush=True)
                last_report = now
    except KeyboardInterrupt:
        stats['stop_reason'] = 'interrupted'
    except Exception:
        stats['stop_reason'] = 'error'
        raise
    finally:
        stats['window_elapsed_s'] = min(time.monotonic(), deadline) - started
        stats[rate_name] = ((stats['received'] - 1) / (last_frame - first_frame)
                            if first_frame is not None and last_frame > first_frame else 0.0)
        stats['receive_mean_ms'] = receive_elapsed * 1000 / receive_calls if receive_calls else 0.0
        stats['receive_max_ms'] = receive_max * 1000
    return stats


def main(argv=None):
    args = arguments(argv)
    show = args.mode == 'preview' or args.preview
    if show and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        print('预览需要桌面显示；无显示环境请使用 --mode capture 且不加 --preview', file=sys.stderr)
        return 1
    try:
        import cv2
    except ImportError:
        print('缺少 OpenCV；请使用安装了 python3-opencv 的 Python 环境', file=sys.stderr)
        return 1
    capture = writer = session = None
    grab_only = args.mode == 'benchmark' and args.grab_only
    report = dict(status='failed', error=None, mode=args.mode, grab_only=grab_only,
                  opencv_version=cv2.__version__, opencv_module=cv2.__file__,
                  requested_duration_s=args.duration, tagged_height_m=args.height_m,
                  timestamp_source='host_after_grab' if grab_only else 'host_after_read',
                  jpeg_quality=args.jpeg_quality, save_fps=args.save_fps)
    stats = {}
    exit_code = 0
    try:
        capture, report['camera'] = open_camera(cv2, args)
        warmup_camera(capture, args.warmup, grab_only)
        if args.mode == 'benchmark':
            print('测速最多 %.2f 秒；不显示、不保存图片；%s' %
                  (args.duration, '仅 grab，不验证解码图像或曝光时间' if grab_only else 'read 包含图像取出/解码'), flush=True)
        if args.mode == 'capture':
            session = session_directory(args.output)
            writer = ImageWriter(cv2, session, args.jpeg_quality, args.queue_size)
            print('采集目录：%s；预热结束，现在开始最多 %.2f 秒采集' % (session, args.duration), flush=True)
        stream_camera(cv2, capture, args, writer, stats)
        report.update(stats, status='complete' if stats['stop_reason'] == 'duration' else stats['stop_reason'])
        if args.mode != 'preview' and stats['received'] == 0:
            raise RuntimeError('采集/测速窗口内没有收到有效帧')
    except KeyboardInterrupt:
        report.update(status='interrupted', error='用户中断')
        exit_code = 130
    except Exception as error:
        report.update(status='failed', error=str(error))
        print('错误：' + str(error), file=sys.stderr)
        exit_code = 1
    finally:
        report.update(stats)
        if capture is not None:
            capture.release()
        if show:
            cv2.destroyAllWindows()
        if writer is not None:
            try:
                writer.close()
            except Exception as error:
                report.update(status='failed', error=str(error))
                print(str(error), file=sys.stderr)
                exit_code = 1
            report['saved'] = writer.saved
        if session is not None:
            report['total_finished_unix_s'] = time.time()
            try:
                with (session / 'session.json').open('x', encoding='utf-8') as stream:
                    json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            except OSError as error:
                print('无法保存 session.json：' + str(error), file=sys.stderr)
                exit_code = 1
        if session is not None or args.mode == 'benchmark':
            print('结果：' + json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        rate_name = 'grab_fps' if grab_only else 'read_fps'
        if report.get(rate_name, args.fps) < args.fps * 0.9:
            print('警告：实测 %s 明显低于请求 %.1f FPS；请求值不是实测值。' % (rate_name, args.fps), file=sys.stderr)
        if report.get('queue_dropped', 0):
            print('警告：落盘队列满，有图像未保存；请检查 saved/queue_dropped。', file=sys.stderr)
    if report.get('stop_reason') == 'interrupted':
        return 130
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
