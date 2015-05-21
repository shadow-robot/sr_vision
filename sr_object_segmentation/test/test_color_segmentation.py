#!/usr/bin/env python

import unittest

from sr_object_segmentation.color_segmentation import ColorSegmentation
from sr_benchmarking.drawing import BasicTest
from sr_benchmarking.drawing import NoiseTest


class TestColorSegmentation(unittest.TestCase):

    """ Test class for color segmentation algorithm """

    def setUp(self):
        """
        Initialization
        """
        self.datasets = [BasicTest(), NoiseTest()]

    def test_not_empty(self, pts):
        """
        Verify that dictionaries are not empty
        """
        self.assertIsNotNone(pts)

    def test_background(self, seg):
        """
        Verify that background is not take into account for Basic and Noise tests
        """
        theo_back = 640 * 480 - 200 * 300
        self.assertLess(len(seg), theo_back)

    def runTest(self):
        """
        Run the tests for each image from each dataset
        """
        print "runTest"
        for dataset in self.datasets:
            images = dataset.np_img
            for img in images:
                pts = ColorSegmentation(img).points
                self.test_not_empty(pts)
                for seg in pts:
                    self.test_background(pts[seg])


if __name__ == '__main__':

    suite = unittest.TestSuite()
    suite.addTest(TestColorSegmentation())
    test = unittest.TextTestRunner(verbosity=2).run(suite)
