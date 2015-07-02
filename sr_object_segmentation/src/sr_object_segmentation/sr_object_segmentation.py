#!/usr/bin/env python

from sr_object_tracking.utils import Utils


class SrObjectSegmentation(object):
    """
    Base class for object segmentation
    """

    def __init__(self, image, points):
        """
        Initialize the segmentation object
        @param image - image to be segmented
        @param points - dictionary with segments and corresponding points
        @attribute nb_segments - number of segments found in the image
        """
        self.name = ''
        self.img = image
        self.frame = None
        self.points = points
        self.nb_segments = len(self.points)
        self.selection = None
        self.utils = Utils()
