"""Find a square outer frame by its edges.

Red and blue are not used. /yolo/enable chooses the question, and this only
locates the rectangle.
"""
import cv2
import numpy as np


def list_square_frames(image, min_area_ratio=0.01, max_area_ratio=0.92, pad_px=4):
    """Every square frame, as [x, y, width, height], largest first."""
    height, width = _require_bgr(image)
    if not 0 < min_area_ratio < max_area_ratio <= 1 or pad_px < 0:
        raise ValueError('invalid square frame limits')
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 40, 120)
    edges = cv2.dilate(edges, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    frame_area = float(width * height)
    boxes = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if not min_area_ratio * frame_area <= area <= max_area_ratio * frame_area:
            continue
        perimeter = cv2.arcLength(contour, True)
        if perimeter < 1:
            continue
        approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
        if not _usable_square(approx):
            continue
        x, y, box_w, box_h = cv2.boundingRect(approx)
        if not _frame_ring(edges, x, y, box_w, box_h):
            continue
        x0 = max(0, x - pad_px)
        y0 = max(0, y - pad_px)
        x1 = min(width, x + box_w + pad_px)
        y1 = min(height, y + box_h + pad_px)
        if x1 - x0 >= 16 and y1 - y0 >= 16:
            boxes.append([int(x0), int(y0), int(x1 - x0), int(y1 - y0)])
    return _dedupe(boxes)


def _require_bgr(image):
    if image is None or getattr(image, 'size', 0) == 0 or len(getattr(image, 'shape', ())) < 2:
        raise ValueError('empty image')
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('frame ROI expects a BGR image')
    return image.shape[:2]


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


def _frame_ring(edges, x, y, width, height):
    """A frame has an inner edge. A filled block only has the outer edge."""
    patch = edges[y:y + height, x:x + width]
    if patch.shape[0] != height or patch.shape[1] != width:
        return False
    outer = max(3, int(0.012 * min(width, height)))
    inner = max(outer + 4, int(0.08 * min(width, height)))
    if inner >= min(width, height) // 2:
        return False
    ring = np.zeros((height, width), np.uint8)
    cv2.rectangle(ring, (outer, outer), (width - 1 - outer, height - 1 - outer), 255, -1)
    cv2.rectangle(ring, (inner, inner), (width - 1 - inner, height - 1 - inner), 0, -1)
    ring_area = cv2.countNonZero(ring)
    if ring_area < 1:
        return False
    return cv2.countNonZero(cv2.bitwise_and(patch, ring)) / float(ring_area) >= 0.08


def _dedupe(boxes):
    kept = []
    for box in sorted(boxes, key=lambda item: item[2] * item[3], reverse=True):
        if any(_iou(box, other) > 0.85 for other in kept):
            continue
        kept.append(box)
    return kept


def _iou(first, second):
    ax, ay, aw, ah = first
    bx, by, bw, bh = second
    x1 = max(ax, bx)
    y1 = max(ay, by)
    x2 = min(ax + aw, bx + bw)
    y2 = min(ay + ah, by + bh)
    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    union = aw * ah + bw * bh - intersection
    if union <= 0:
        return 0.0
    return intersection / float(union)
