#!/usr/bin/env python

import unittest

from sr_object_segmentation.color_segmentation import ColorSegmentation
from sr_benchmarking.drawing import BasicTest
from sr_benchmarking.drawing import NoiseTest

PKG = 'sr_object_segmentation'


class TestColorSegmentation(unittest.TestCase):
    """ Test class for color segmentation algorithm """

    def setUp(self):
        """
        Initialization
        """
        self.datasets = [BasicTest(), NoiseTest()]
        self.algo = ColorSegmentation

        for dataset in self.datasets:
            images = dataset.np_img
            for img in images:
                self.pts = self.algo(img).points

    def test_not_empty(self):
        """
        Verify that dictionaries are not empty
        """
        self.assertIsNotNone(self.pts)

    def test_background(self):
        """
        Verify that background is not take into account for Basic and Noise
        tests
        """
        theo_back = 640 * 480 - 200 * 300
        for seg in self.pts:
            self.assertLess(len(self.pts[seg]), theo_back)


if __name__ == '__main__':
    import rosunit

    rosunit.unitrun(PKG, 'test_color_segmentation', TestColorSegmentation)
