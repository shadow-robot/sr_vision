#!/usr/bin/env python

import unittest
import numpy as np

from sr_object_segmentation.blobs_segmentation import BlobsSegmentation
from sr_benchmarking.drawing import BasicTest


class TestBlobsSegmentation(unittest.TestCase):
    """
    Test class for the blobs segmentation algorithm
    """

    def setUp(self):
        self.dataset = BasicTest()
        self.blobs = []
        for img in self.dataset.np_img:
            self.blobs.append(BlobsSegmentation(img))

    def test_not_empty(self):
        for i in range(len(self.dataset.np_img)):
            self.assertIsNotNone(self.blobs[i].points)


if __name__ == "__main__":
    unittest.main()
