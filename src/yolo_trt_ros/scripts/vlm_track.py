"""Follow one confirmed target with OpenCV while the aircraft moves.

A danger or non-danger sign is boxed on its triangular border. A portrait or
non-portrait image is boxed on its blue square frame. The VLM is not called
again; each camera frame only updates the box.
"""
import math

import cv2
import numpy as np

from vlm_frame import list_square_frames

# OpenCV hue is 0-180. This is the yellow fill inside a dark triangular border.
_YELLOW_LOW = (15, 70, 70)
_YELLOW_HIGH = (40, 255, 255)


def follow_target(image, shape, previous=None):
    """Return {'bbox', 'polygon'} for the same target, or None when it is lost.

    bbox is [x, y, width, height] in full-image pixels. polygon is the triangle
    vertices when the target is a sign, and None for a square frame.
    """
    if shape == 'triangle':
        candidates = _triangles(image)
    elif shape == 'square':
        candidates = [{'bbox': box, 'polygon': None}
                      for box in list_square_frames(image, 'blue')]
    else:
        raise ValueError('shape must be triangle or square')
    return _choose(candidates, previous)


def _choose(candidates, previous):
    if not candidates:
        return None
    if previous is None:
        return max(candidates, key=lambda item: item['bbox'][2] * item['bbox'][3])
    px, py, pw, ph = [float(value) for value in previous]
    center_x = px + pw / 2.0
    center_y = py + ph / 2.0
    # A later frame may shift with the aircraft, but not jump to another target.
    limit = max(pw, ph, 48.0) * 1.5
    best = None
    best_distance = None
    for item in candidates:
        x, y, width, height = item['bbox']
        distance = math.hypot(x + width / 2.0 - center_x, y + height / 2.0 - center_y)
        if distance > limit:
            continue
        if best is None or distance < best_distance:
            best = item
            best_distance = distance
    return best


def _triangles(image, min_area_ratio=0.004, max_area_ratio=0.8,
               border_scale=1.10, pad_px=6):
    height, width = image.shape[:2]
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('tracking expects a BGR image')
    frame_area = float(width * height)
    yellow = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), _YELLOW_LOW, _YELLOW_HIGH)
    yellow = cv2.morphologyEx(yellow, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    contours, _ = cv2.findContours(yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if not min_area_ratio * frame_area <= area <= max_area_ratio * frame_area:
            continue
        perimeter = cv2.arcLength(contour, True)
        if perimeter < 1:
            continue
        approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
        if not _usable_triangle(approx):
            continue
        points = approx.reshape(3, 2).astype(np.float32)
        center = points.mean(axis=0)
        expanded = center + (points - center) * float(border_scale)
        x, y, box_w, box_h = cv2.boundingRect(np.round(expanded).astype(np.int32))
        x0 = max(0, x - pad_px)
        y0 = max(0, y - pad_px)
        x1 = min(width, x + box_w + pad_px)
        y1 = min(height, y + box_h + pad_px)
        if x1 - x0 < 16 or y1 - y0 < 16:
            continue
        found.append({
            'bbox': [int(x0), int(y0), int(x1 - x0), int(y1 - y0)],
            'polygon': [[int(round(px)), int(round(py))] for px, py in expanded],
        })
    return found


def _usable_triangle(approx):
    if len(approx) != 3 or not cv2.isContourConvex(approx):
        return False
    points = approx.reshape(3, 2).astype(np.float32)
    sides = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
    longest = float(sides.max())
    if longest < 12 or float(sides.min()) < 12:
        return False
    altitude = 2.0 * cv2.contourArea(points) / longest
    if altitude / longest < 0.28:
        return False
    box_w = float(points[:, 0].max() - points[:, 0].min())
    box_h = float(points[:, 1].max() - points[:, 1].min())
    return box_w >= 16 and box_h >= 16 and 0.45 <= box_w / box_h <= 2.4
