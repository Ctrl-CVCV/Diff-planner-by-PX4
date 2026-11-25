"""OpenCV follows the triangle of a sign and the square frame of a portrait."""
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
        self.assertGreater(cropped['image'].shape[1], output['bbox'][2])
        self.assertIsNone(recognition_crop(_yellow_triangle(), 'triangle'))

    def test_recognition_crop_is_the_shape_not_the_whole_frame(self):
        portrait = _blue_square()
        square = recognition_crop(portrait, 'square')
        self.assertIsNotNone(square)
        self.assertLess(square['image'].shape[0] * square['image'].shape[1],
                        portrait.shape[0] * portrait.shape[1])
        self.assertIsNone(recognition_crop(_yellow_triangle(), 'square'))
        self.assertIsNone(recognition_crop(portrait, 'triangle'))


if __name__ == '__main__':
    unittest.main()
