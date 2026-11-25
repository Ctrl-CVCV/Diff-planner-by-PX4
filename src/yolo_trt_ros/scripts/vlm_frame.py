"""Find a red or blue square frame in a camera image.

The color only locates the frame. It does not decide whether the VLM is asked
about a sign or a portrait; /yolo/enable does that.
"""
import cv2
import numpy as np

_RED_RANGES = (((0, 80, 70), (12, 255, 255)), ((168, 80, 70), (180, 255, 255)))
_BLUE_RANGE = ((95, 60, 50), (135, 255, 255))


def extract_frame_roi(image, min_area_ratio=0.01, max_area_ratio=0.92, pad_px=4,
                      only_color=None):
    """Return (roi, info). roi is None when no matching square frame is found."""
    height, width = _require_bgr(image)
    if only_color not in (None, 'red', 'blue'):
        raise ValueError('only_color must be red, blue, or None')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    masks = {'red': _color_mask(hsv, _RED_RANGES), 'blue': _color_mask(hsv, (_BLUE_RANGE,))}
    colors = (only_color,) if only_color else ('red', 'blue')
    frame_area = float(width * height)
    best = None
    for color in colors:
        mask = masks[color]
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if not min_area_ratio * frame_area <= area <= max_area_ratio * frame_area:
                continue
            perimeter = cv2.arcLength(contour, True)
            if perimeter < 1:
                continue
            approx = cv2.approxPolyDP(contour, 0.03 * perimeter, True)
            if not _usable_square(approx):
                continue
            points = approx.reshape(4, 2).astype(np.float32)
            x, y, box_w, box_h = cv2.boundingRect(np.round(points).astype(np.int32))
            if box_w < 24 or box_h < 24:
                continue
            fill = float(mask[y:y + box_h, x:x + box_w].mean()) / 255.0
            if not 0.05 <= fill <= 0.45:
                continue
            winner, _counts = _ring_vote(points, masks, (height, width))
            if winner != color:
                continue
            if best is None or area > best[0]:
                best = (area, color, points)
    if best is None:
        return None, {'found': False, 'source_width': width, 'source_height': height}
    _area, color, points = best
    x, y, box_w, box_h = cv2.boundingRect(np.round(points).astype(np.int32))
    x0 = max(0, x - pad_px)
    y0 = max(0, y - pad_px)
    x1 = min(width, x + box_w + pad_px)
    y1 = min(height, y + box_h + pad_px)
    if x1 - x0 < 16 or y1 - y0 < 16:
        return None, {'found': False, 'source_width': width, 'source_height': height}
    return image[y0:y1, x0:x1].copy(), {
        'found': True,
        'source_width': width,
        'source_height': height,
        'border_color': color,
        'bbox': [int(x0), int(y0), int(x1 - x0), int(y1 - y0)],
    }


def list_square_frames(image, color, min_area_ratio=0.01, max_area_ratio=0.92, pad_px=4):
    """Every hollow square frame of one color, as [x, y, width, height]."""
    height, width = _require_bgr(image)
    if color not in ('red', 'blue'):
        raise ValueError('color must be red or blue')
    if not 0 < min_area_ratio < max_area_ratio <= 1 or pad_px < 0:
        raise ValueError('invalid square frame limits')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    masks = {'red': _color_mask(hsv, _RED_RANGES), 'blue': _color_mask(hsv, (_BLUE_RANGE,))}
    frame_area = float(width * height)
    boxes = []
    contours, _ = cv2.findContours(masks[color], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if not min_area_ratio * frame_area <= area <= max_area_ratio * frame_area:
            continue
        perimeter = cv2.arcLength(contour, True)
        if perimeter < 1:
            continue
        approx = cv2.approxPolyDP(contour, 0.03 * perimeter, True)
        if not _usable_square(approx):
            continue
        points = approx.reshape(4, 2).astype(np.float32)
        x, y, box_w, box_h = cv2.boundingRect(np.round(points).astype(np.int32))
        if box_w < 24 or box_h < 24:
            continue
        fill = float(masks[color][y:y + box_h, x:x + box_w].mean()) / 255.0
        if not 0.05 <= fill <= 0.45:
            continue
        winner, _counts = _ring_vote(points, masks, (height, width))
        if winner != color:
            continue
        x0 = max(0, x - pad_px)
        y0 = max(0, y - pad_px)
        x1 = min(width, x + box_w + pad_px)
        y1 = min(height, y + box_h + pad_px)
        if x1 - x0 >= 16 and y1 - y0 >= 16:
            boxes.append([int(x0), int(y0), int(x1 - x0), int(y1 - y0)])
    return boxes


def _require_bgr(image):
    if image is None or getattr(image, 'size', 0) == 0 or len(getattr(image, 'shape', ())) < 2:
        raise ValueError('empty image')
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('frame ROI expects a BGR image')
    return image.shape[:2]


def _color_mask(hsv, ranges):
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for low, high in ranges:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, low, high))
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))


def _usable_square(approx):
    if len(approx) != 4 or not cv2.isContourConvex(approx):
        return False
    points = approx.reshape(4, 2).astype(np.float32)
    sides = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
    if float(sides.min()) < 20 or float(sides.max()) / float(sides.min()) > 1.45:
        return False
    box_w = float(points[:, 0].max() - points[:, 0].min())
    box_h = float(points[:, 1].max() - points[:, 1].min())
    return box_w >= 24 and box_h >= 24 and 0.75 <= box_w / box_h <= 1.35


def _ring_vote(points, masks, shape):
    ring = np.zeros(shape, np.uint8)
    center = points.mean(axis=0)
    outer = np.round(center + (points - center) * 1.04).astype(np.int32)
    inner = np.round(center + (points - center) * 0.78).astype(np.int32)
    cv2.fillPoly(ring, [outer], 255)
    cv2.fillPoly(ring, [inner], 0)
    counts = {name: int(cv2.countNonZero(cv2.bitwise_and(mask, ring)))
              for name, mask in masks.items()}
    total = counts['red'] + counts['blue']
    if total < 40:
        return None, counts
    winner = max(counts, key=counts.get)
    if counts[winner] / float(total) < 0.7:
        return None, counts
    return winner, counts
