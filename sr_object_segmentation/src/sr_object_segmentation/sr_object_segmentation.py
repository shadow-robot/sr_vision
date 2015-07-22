#!/usr/bin/env python

from sr_object_tracking.utils import Utils


class SrObjectSegmentation(object):
    """
    Base class for object segmentation
    """

    def __init__(self):
        """
        Initialize the segmentation object
        @param image - image to be segmented
        @param points - dictionary with segments and corresponding points
        (optional)
        @attribute nb_segments - number of segments found in the image
        """
        self.name = ''
        self.frame = None
        self.segmented_box = []
        self.poses = []
        self.nb_segments = len(self.segmented_box)
        self.selection = None
        self.utils = Utils()
