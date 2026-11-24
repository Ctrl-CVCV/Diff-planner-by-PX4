#!/usr/bin/env python3
"""用 USB 摄像头录一段 640x480 视频，Ctrl-C（或 GUI 的停止按钮）结束。

用法：
    python3 usb_camera_record.py --device /dev/video0 --output ~/Videos/test.mp4
    python3 usb_camera_record.py --duration 30

输出容器按扩展名决定（.mp4 用 mp4v，.avi 用 MJPG）。摄像头被 YOLO 节点占用时
VideoCapture 会打不开，先停掉占用进程。
"""

import argparse
import signal
import sys
import time
from pathlib import Path

import cv2


STOP = {'flag': False}


def _handle_signal(_signum, _frame):
    STOP['flag'] = True


def parse_args(argv):
    parser = argparse.ArgumentParser(description='USB 摄像头录制 640x480 视频')
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=float, default=30.0)
    parser.add_argument('--output', type=Path,
                        default=Path.home() / 'Videos' / 'usb_640x480.mp4')
    parser.add_argument('--duration', type=float, default=0.0, help='录制秒数；0 表示一直录到停止')
    return parser.parse_args(argv)


def fourcc_for(path):
    return 'mp4v' if path.suffix.lower() == '.mp4' else 'MJPG'


def main(argv=None):
    args = parse_args(argv)
    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    capture = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not capture.isOpened():
        print('错误：打不开摄像头 %s（可能被 YOLO/其它程序占用）' % args.device, file=sys.stderr)
        return 2
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    capture.set(cv2.CAP_PROP_FPS, args.fps)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH) or args.width)
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT) or args.height)
    fps = float(capture.get(cv2.CAP_PROP_FPS) or args.fps)
    if fps <= 1.0 or fps > 240.0:
        fps = args.fps

    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(args.output),
                             cv2.VideoWriter_fourcc(*fourcc_for(args.output)), fps, (width, height))
    if not writer.isOpened():
        print('错误：无法创建视频文件 %s（检查扩展名/磁盘空间）' % args.output, file=sys.stderr)
        capture.release()
        return 2

    print('开始录制：%s  %dx%d @ %.1f FPS -> %s' % (args.device, width, height, fps, args.output))
    started = time.monotonic()
    frames = 0
    misses = 0
    last_report = started
    try:
        while not STOP['flag']:
            ok, frame = capture.read()
            if not ok:
                misses += 1
                if misses > 100:
                    print('错误：连续 %d 次读不到画面，停止录制' % misses, file=sys.stderr)
                    break
                time.sleep(0.01)
                continue
            misses = 0
            if frame.shape[1] != width or frame.shape[0] != height:
                frame = cv2.resize(frame, (width, height))
            writer.write(frame)
            frames += 1
            now = time.monotonic()
            if args.duration > 0 and now - started >= args.duration:
                break
            if now - last_report >= 1.0:
                last_report = now
                print('已录 %.1fs，%d 帧' % (now - started, frames))
    finally:
        writer.release()
        capture.release()
    elapsed = max(1e-6, time.monotonic() - started)
    print('录制结束：%s（%.1fs，%d 帧，平均 %.1f FPS）' % (args.output, elapsed, frames, frames / elapsed))
    return 0


if __name__ == '__main__':
    sys.exit(main())
