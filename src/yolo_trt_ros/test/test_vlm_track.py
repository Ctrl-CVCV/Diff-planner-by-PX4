"""OpenCV follows the triangle of a sign and the square frame of a portrait."""
import math
import unittest

import cv2
import numpy as np

from vlm_track import follow_target, recognition_crop


def _yellow_triangle(origin=(260, 230), scale=1.0):
    image = np.full((480, 640, 3), (190, 185, 175), np.uint8)
    prototype = np.array([[0, -150], [-160, 150], [160, 150]], np.float32) * scale
    points = np.round(prototype + np.array(origin, np.float32)).astype(np.int32)
    center = points.mean(axis=0)
    inner = np.round(center + (points - center) * 0.82).astype(np.int32)
    cv2.fillPoly(image, [points], (15, 15, 15))
    cv2.fillPoly(image, [inner], (0, 220, 240))
    return image


def _blue_square(origin=(180, 80), size=280):
    image = np.full((480, 640, 3), (235, 235, 235), np.uint8)
    x, y = origin
    cv2.rectangle(image, (x, y), (x + size, y + size), (210, 90, 20), 12)
    return image


class FollowTargetTests(unittest.TestCase):
    def test_sign_box_stays_on_the_triangle_as_it_shifts(self):
        first = follow_target(_yellow_triangle((260, 230)), 'triangle')
        self.assertIsNotNone(first)
        self.assertIsNone(first['polygon'])
        x, y, width, height = first['bbox']
        self.assertLess(width * height, 640 * 480 / 2)
        self.assertGreater(width, 80)

        moved = _yellow_triangle((300, 250))
        # A second, larger triangle on the far side must not steal the track.
        far = np.array([[40, 30], [20, 220], [180, 220]], np.int32)
        center = far.mean(axis=0)
        inner = np.round(center + (far - center) * 0.82).astype(np.int32)
        cv2.fillPoly(moved, [far], (15, 15, 15))
        cv2.fillPoly(moved, [inner], (0, 220, 240))
        followed = follow_target(moved, 'triangle', first['bbox'])
        self.assertIsNotNone(followed)
        self.assertGreater(followed['bbox'][0], x)

        rectangle = np.full((480, 640, 3), (180, 180, 180), np.uint8)
        cv2.rectangle(rectangle, (200, 150), (420, 330), (0, 220, 240), -1)
        self.assertIsNone(follow_target(rectangle, 'triangle'))

    def test_portrait_box_follows_a_square_frame_of_either_color(self):
        first = follow_target(_blue_square((120, 60)), 'square')
        self.assertIsNotNone(first)
        self.assertIsNone(first['polygon'])
        followed = follow_target(_blue_square((160, 90)), 'square', first['bbox'])
        self.assertIsNotNone(followed)
        self.assertGreater(followed['bbox'][0], first['bbox'][0])
        self.assertGreater(followed['bbox'][1], first['bbox'][1])

        red = np.full((480, 640, 3), (235, 235, 235), np.uint8)
        cv2.rectangle(red, (160, 60), (440, 340), (20, 20, 210), 14)
        self.assertIsNotNone(follow_target(red, 'square'))
        self.assertIsNone(follow_target(red, 'triangle'))
        self.assertIsNone(follow_target(_blue_square(), 'triangle'))

    def test_sign_crop_is_the_outer_frame_and_output_touches_the_corners(self):
        image = np.full((480, 640, 3), (235, 235, 235), np.uint8)
        cv2.rectangle(image, (70, 30), (510, 450), (20, 20, 210), 16)
        triangle = _yellow_triangle((290, 230), scale=0.55)
        mask = np.any(triangle != np.array((190, 185, 175), np.uint8), axis=2)
        image[mask] = triangle[mask]
        output = follow_target(image, 'triangle')
        self.assertIsNotNone(output)
        self.assertLess(output['bbox'][2], 250)
        cropped = recognition_crop(image, 'triangle')
        self.assertIsNotNone(cropped)
        self.assertGreater(cropped['bbox'][2], 400)
        self.assertLess(cropped['track_bbox'][2], 250)
        self.assertGreater(cropped['image'].shape[1], output['bbox'][2])
        self.assertIsNone(recognition_crop(_yellow_triangle(), 'triangle'))
        three_sides = image.copy()
        cv2.rectangle(three_sides, (478, 26), (520, 454), (235, 235, 235), -1)
        opened = recognition_crop(three_sides, 'triangle')
        self.assertTrue(opened['full_frame'])
        self.assertEqual(opened['image'].shape[:2], three_sides.shape[:2])
        self.assertLess(opened['track_bbox'][2], 250)

    def test_recognition_crop_is_the_shape_not_the_whole_frame(self):
        portrait = _blue_square()
        square = recognition_crop(portrait, 'square')
        self.assertIsNotNone(square)
        self.assertLess(square['image'].shape[0] * square['image'].shape[1],
                        portrait.shape[0] * portrait.shape[1])
        self.assertIsNone(recognition_crop(_yellow_triangle(), 'square'))
        self.assertIsNone(recognition_crop(portrait, 'triangle'))

    def test_square_track_survives_one_missing_edge(self):
        origin, size = (120, 60), 280
        whole = _blue_square(origin, size)
        first = follow_target(whole, 'square')
        self.assertIsNotNone(first)
        broken = _blue_square(origin, size)
        _erase_square_side(broken, origin, size, 'right')
        self.assertIsNone(follow_target(broken, 'square'))
        whole_picture = recognition_crop(broken, 'square')
        self.assertTrue(whole_picture['full_frame'])
        self.assertEqual(whole_picture['image'].shape[:2], broken.shape[:2])
        self.assertLess(_center_shift(first['bbox'], whole_picture['track_bbox']), 0.3 * size)
        followed = follow_target(broken, 'square', first['bbox'])
        self.assertIsNotNone(followed)
        self.assertLess(_center_shift(first['bbox'], followed['bbox']), 0.3 * size)

    def test_first_track_frame_uses_the_box_found_before_vlm(self):
        origin, size = (120, 60), 280
        whole = _blue_square(origin, size)
        crop = recognition_crop(whole, 'square')
        broken = _blue_square(origin, size)
        _erase_square_side(broken, origin, size, 'right')
        self.assertIsNone(follow_target(broken, 'square'))
        followed = follow_target(broken, 'square', crop['track_bbox'])
        self.assertIsNotNone(followed)
        self.assertLess(_center_shift(crop['track_bbox'], followed['bbox']), 0.3 * size)

    def test_missing_edge_does_not_jump_to_a_distant_square(self):
        origin, size = (300, 100), 160
        whole = _blue_square(origin, size)
        first = follow_target(whole, 'square')
        broken = _blue_square(origin, size)
        _erase_square_side(broken, origin, size, 'left')
        cv2.rectangle(broken, (10, 20), (110, 120), (210, 90, 20), 10)
        followed = follow_target(broken, 'square', first['bbox'])
        self.assertIsNotNone(followed)
        self.assertLess(_center_shift(first['bbox'], followed['bbox']), 0.3 * size)
        self.assertGreater(followed['bbox'][0], 200)

    def test_triangle_track_survives_one_missing_edge(self):
        whole = _yellow_triangle()
        first = follow_target(whole, 'triangle')
        self.assertIsNotNone(first)
        broken = _yellow_triangle()
        _erase_triangle_side(broken)
        self.assertIsNone(follow_target(broken, 'triangle'))
        cv2.fillPoly(broken, [np.array([[600, 420], [580, 470], [639, 470]], np.int32)],
                     (0, 220, 240))
        followed = follow_target(broken, 'triangle', first['bbox'])
        self.assertIsNotNone(followed)
        self.assertLess(_center_shift(first['bbox'], followed['bbox']), 80)


def _erase_square_side(image, origin, size, side):
    color = tuple(int(value) for value in image[0, 0])
    x, y = origin
    depth = 32
    if side == 'right':
        cv2.rectangle(image, (x + size - depth, y - 4), (x + size + 8, y + size + 4), color, -1)
    elif side == 'left':
        cv2.rectangle(image, (x - 8, y - 4), (x + depth, y + size + 4), color, -1)
    else:
        raise ValueError('side must be left or right')


def _erase_triangle_side(image):
    """Cover the left side so the yellow fill is no longer a triangle."""
    color = tuple(int(value) for value in image[0, 0])
    cv2.rectangle(image, (90, 90), (190, 420), color, -1)


def _center_shift(first, second):
    return math.hypot((first[0] + first[2] / 2.0) - (second[0] + second[2] / 2.0),
                       (first[1] + first[3] / 2.0) - (second[1] + second[3] / 2.0))


if __name__ == '__main__':
    unittest.main()
