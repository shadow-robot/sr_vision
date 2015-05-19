#!/usr/bin/env python


class SrObjectSegmentation:
    """
    Base class for object segmentation
    """
    def __init__(self,image,points):  
        """
        Initialize the segmentation object
        @param image - image to be segmented
        @param points - dictionnary with segments and corresponding points (optionnal)
        @attribute nb_segments - number of segments found in the image
        """
        self.img=image
        self.points=points
        self.nb_segments=len(self.points)
        


