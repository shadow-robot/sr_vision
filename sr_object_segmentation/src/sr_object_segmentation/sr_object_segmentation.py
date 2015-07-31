#!/usr/bin/env python


class SrObjectSegmentation(object):
    """
    Base class for object segmentation
    """

    def __init__(self):
        """
        Initialize the segmentation object
        """
        self.name = ''
        self.segmented_box = []
        self.nb_segments = 0
