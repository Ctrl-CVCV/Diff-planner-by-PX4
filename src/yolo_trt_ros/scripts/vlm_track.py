"""Follow one confirmed target with OpenCV while the aircraft moves.

The image sent to the VLM is the outer rectangular frame when all four sides are
visible. With only three sides it is the whole picture, because there is no
ROI. Afterward the
published box differs: a sign uses the rectangle tangent to its triangle's
three corners, and a portrait keeps the outermost frame. The VLM is not called
again, and the question still comes only from /yolo/enable.
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
    the outermost square frame. Once a box exists, one missing edge still
    keeps the track. polygon is always None.
    """
    if shape == 'triangle':
        candidates = [{'bbox': triangle['bbox'], 'polygon': None}
                      for triangle in _triangles(image)]
    elif shape == 'square':
        candidates = [{'bbox': box, 'polygon': None} for box in _square_boxes(image)]
    else:
        raise ValueError('shape must be triangle or square')
    chosen = _choose(candidates, previous)
    if chosen is not None or previous is None:
        return chosen
    if shape == 'square':
        return _square_with_missing_edge(image, previous)
    return _triangle_with_missing_edge(image, previous)


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
            best = {'bbox': list(outer), 'polygon': None,
                    'track_bbox': list(triangle['bbox'])}
            best_area = area
    return best


def _enclosing_square(center, squares):
    """Outermost square frame that contains the triangle center."""
    cx, cy = center
    containing = [box for box in squares
                  if box[0] <= cx <= box[0] + box[2] and box[1] <= cy <= box[1] + box[3]]
    if not containing:
        return None
    return max(containing, key=lambda box: box[2] * box[3])


def _square_boxes(image):
    return list_square_frames(image)


def recognition_crop(image, shape):
    """Return the image sent to the VLM, or None while no frame is visible.

    Four sides are cropped to the outer rectangle. Three sides send the whole
    picture and remember a box for the first track. bbox stays in full-image pixels.
    """
    if shape == 'triangle':
        found = _sign_outer_frame(image)
    elif shape == 'square':
        found = follow_target(image, 'square')
    else:
        raise ValueError('shape must be triangle or square')
    if found:
        return _crop_found(image, found)
    return _full_frame_if_three_sides(image, shape)


def _crop_found(image, found):
    x, y, width, height = [int(value) for value in found['bbox']]
    crop = image[y:y + height, x:x + width]
    if crop.size == 0:
        return None
    track_bbox = found.get('track_bbox') or found['bbox']
    return {'image': crop.copy(), 'bbox': list(found['bbox']),
            'track_bbox': [int(value) for value in track_bbox],
            'polygon': found.get('polygon'), 'full_frame': False}


def _full_frame_if_three_sides(image, shape):
    """No closed rectangle: send the whole picture when three sides remain."""
    partial = _partial_square(image)
    if partial is None:
        return None
    height, width = image.shape[:2]
    if shape == 'triangle':
        inside = [item for item in _triangles(image)
                  if _box_contains(partial['bbox'], item['center'])]
        if not inside:
            return None
        triangle = max(inside, key=lambda item: item['bbox'][2] * item['bbox'][3])
        track = list(triangle['bbox'])
    else:
        track = list(partial['bbox'])
    return {'image': image.copy(), 'bbox': [0, 0, int(width), int(height)],
            'track_bbox': track, 'polygon': None, 'full_frame': True}


def _box_contains(box, center):
    x, y, width, height = box
    cx, cy = center
    return x <= cx <= x + width and y <= cy <= y + height


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
    contours, _ = cv2.findContours(_yellow_mask(image), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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


def _yellow_mask(image):
    yellow = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), _YELLOW_LOW, _YELLOW_HIGH)
    return cv2.morphologyEx(yellow, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))


def _center_limit(width, height):
    return max(float(width), float(height), 48.0) * 1.5


def _edge_image(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return cv2.dilate(cv2.Canny(gray, 40, 120), np.ones((3, 3), np.uint8))


def _partial_square(image):
    """Largest roughly square outline that still has at least three sides."""
    if image.ndim != 3 or image.shape[2] != 3:
        return None
    height, width = image.shape[:2]
    edges = _edge_image(image)
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    frame_area = float(width * height)
    best = None
    best_area = 0
    for contour in contours:
        x, y, box_w, box_h = cv2.boundingRect(contour)
        if box_w < 48 or box_h < 48 or not 0.75 <= box_w / float(box_h) <= 1.35:
            continue
        if not 0.01 * frame_area <= box_w * box_h <= 0.92 * frame_area:
            continue
        completed = _square_with_missing_edge(image, (x, y, box_w, box_h), edges)
        if completed is None:
            continue
        area = completed['bbox'][2] * completed['bbox'][3]
        if area > best_area:
            best = completed
            best_area = area
    return best


def _square_with_missing_edge(image, previous, edges=None):
    """Keep a square track when exactly one side has dropped out of the edges."""
    px, py, pw, ph = [float(value) for value in previous]
    if pw < 16 or ph < 16 or image.ndim != 3 or image.shape[2] != 3:
        return None
    height, width = image.shape[:2]
    if edges is None:
        edges = _edge_image(image)
    across_x = 0.25 * pw
    across_y = 0.25 * ph
    left = _side_position(edges, px - across_x, py, px + across_x, py + ph, 'x')
    right = _side_position(edges, px + pw - across_x, py, px + pw + across_x, py + ph, 'x')
    top = _side_position(edges, px, py - across_y, px + pw, py + across_y, 'y')
    bottom = _side_position(edges, px, py + ph - across_y, px + pw, py + ph + across_y, 'y')
    found = sum(value is not None for value in (left, right, top, bottom))
    if found < 3:
        return None
    if left is None:
        left = right - pw
    elif right is None:
        right = left + pw
    if top is None:
        top = bottom - ph
    elif bottom is None:
        bottom = top + ph
    if right - left <= 0 or bottom - top <= 0:
        return None
    box_w = right - left
    box_h = bottom - top
    if abs(box_w - pw) / pw > 0.4 or abs(box_h - ph) / ph > 0.4:
        return None
    center_x = (left + right) / 2.0
    center_y = (top + bottom) / 2.0
    if math.hypot(center_x - (px + pw / 2.0), center_y - (py + ph / 2.0)) > _center_limit(pw, ph):
        return None
    x0 = max(0, int(round(left)))
    y0 = max(0, int(round(top)))
    x1 = min(width, int(round(right)))
    y1 = min(height, int(round(bottom)))
    if x1 - x0 < 16 or y1 - y0 < 16:
        return None
    return {'bbox': [x0, y0, x1 - x0, y1 - y0], 'polygon': None}


def _side_position(edges, x0, y0, x1, y1, axis):
    """Median edge coordinate when the band still covers most of one side."""
    height, width = edges.shape[:2]
    x0 = max(0, int(math.floor(x0)))
    y0 = max(0, int(math.floor(y0)))
    x1 = min(width, int(math.ceil(x1)))
    y1 = min(height, int(math.ceil(y1)))
    if x1 - x0 < 2 or y1 - y0 < 2:
        return None
    patch = edges[y0:y1, x0:x1]
    rows, cols = np.nonzero(patch)
    if axis == 'x':
        covered = len(np.unique(rows))
        length = y1 - y0
        middle = cols[(rows > 0.2 * length) & (rows < 0.8 * length)] if length else cols[:0]
        if length < 1 or covered / float(length) < 0.45 or not _straight_edge(middle):
            return None
        return float(np.median(middle)) + x0
    covered = len(np.unique(cols))
    length = x1 - x0
    middle = rows[(cols > 0.2 * length) & (cols < 0.8 * length)] if length else rows[:0]
    if length < 1 or covered / float(length) < 0.45 or not _straight_edge(middle):
        return None
    return float(np.median(middle)) + y0


def _straight_edge(offsets):
    """The middle of a frame side is a thin stroke, not a diagonal."""
    if len(offsets) < 8:
        return False
    low, high = np.percentile(offsets, [10, 90])
    return float(high - low) <= 36.0


def _triangle_with_missing_edge(image, previous):
    """Keep a sign track from the nearby yellow region when one side is broken."""
    px, py, pw, ph = [float(value) for value in previous]
    if pw < 16 or ph < 16 or image.ndim != 3 or image.shape[2] != 3:
        return None
    contours, _ = cv2.findContours(_yellow_mask(image), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    limit = _center_limit(pw, ph)
    previous_area = pw * ph
    best = None
    best_area = 0.0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 0.15 * previous_area:
            continue
        x, y, box_w, box_h = cv2.boundingRect(contour)
        if box_w < 16 or box_h < 16 or not 0.45 <= box_w / float(box_h) <= 2.4:
            continue
        center_x = x + box_w / 2.0
        center_y = y + box_h / 2.0
        if math.hypot(center_x - (px + pw / 2.0), center_y - (py + ph / 2.0)) > limit:
            continue
        overlap_w = min(px + pw, x + box_w) - max(px, x)
        overlap_h = min(py + ph, y + box_h) - max(py, y)
        if overlap_w <= 0 or overlap_h <= 0 or overlap_w * overlap_h < 0.35 * previous_area:
            continue
        if area > best_area:
            best = [int(x), int(y), int(box_w), int(box_h)]
            best_area = area
    if best is None:
        return None
    return {'bbox': best, 'polygon': None}


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
