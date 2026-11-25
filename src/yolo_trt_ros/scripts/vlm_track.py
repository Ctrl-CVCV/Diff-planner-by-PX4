"""Follow one confirmed target with OpenCV while the aircraft moves.

The image sent to the VLM is always the outer rectangular frame. Afterward the
published box differs: a sign uses the rectangle tangent to its triangle's
three corners, and a portrait keeps the outermost frame. Frame color does not
choose the VLM question, and the VLM is not called again.
"""
import math

import cv2
import numpy as np

from vlm_frame import list_square_frames

# OpenCV hue is 0-180. This is the yellow fill inside a dark triangular border.
_YELLOW_LOW = (15, 70, 70)
_YELLOW_HIGH = (40, 255, 255)


def follow_target(image, shape, previous=None):
    """Return the rectangle to publish, or None when the target is lost.

    A sign box is tangent to the triangle's three corners. A portrait box is
    the outermost square frame. polygon is always None.
    """
    if shape == 'triangle':
        candidates = [{'bbox': triangle['bbox'], 'polygon': None}
                      for triangle in _triangles(image)]
    elif shape == 'square':
        candidates = [{'bbox': box, 'polygon': None} for box in _square_boxes(image)]
    else:
        raise ValueError('shape must be triangle or square')
    return _choose(candidates, previous)


def _sign_outer_frame(image):
    """Outer rectangular frame around the largest warning triangle, or None."""
    squares = _square_boxes(image)
    best = None
    best_area = None
    for triangle in _triangles(image):
        outer = _enclosing_square(triangle['center'], squares)
        if outer is None:
            continue
        area = triangle['bbox'][2] * triangle['bbox'][3]
        if best is None or area > best_area:
            best = {'bbox': list(outer), 'polygon': None}
            best_area = area
    return best


def _enclosing_square(center, squares):
    """Smallest square frame that contains the triangle center."""
    cx, cy = center
    containing = [box for box in squares
                  if box[0] <= cx <= box[0] + box[2] and box[1] <= cy <= box[1] + box[3]]
    if not containing:
        return None
    return min(containing, key=lambda box: box[2] * box[3])


def _square_boxes(image):
    boxes = []
    for color in ('red', 'blue'):
        boxes.extend(list_square_frames(image, color))
    return boxes


def recognition_crop(image, shape):
    """Return the outer-frame crop sent to the VLM, or None while it is not visible.

    Signs are not sent from the tight triangle box. bbox stays in full-image pixels.
    """
    if shape == 'triangle':
        found = _sign_outer_frame(image)
    elif shape == 'square':
        found = follow_target(image, 'square')
    else:
        raise ValueError('shape must be triangle or square')
    if not found:
        return None
    x, y, width, height = [int(value) for value in found['bbox']]
    crop = image[y:y + height, x:x + width]
    if crop.size == 0:
        return None
    return {'image': crop.copy(), 'bbox': found['bbox'], 'polygon': found.get('polygon')}


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


def _triangles(image, min_area_ratio=0.004, max_area_ratio=0.8):
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
        x, y, box_w, box_h = cv2.boundingRect(np.round(points).astype(np.int32))
        x0 = max(0, x)
        y0 = max(0, y)
        x1 = min(width, x + box_w)
        y1 = min(height, y + box_h)
        if x1 - x0 < 16 or y1 - y0 < 16:
            continue
        found.append({
            'bbox': [int(x0), int(y0), int(x1 - x0), int(y1 - y0)],
            'center': (float(center[0]), float(center[1])),
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
