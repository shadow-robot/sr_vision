#!/usr/bin/env python


class SrObjectSegmentation:
    """
    Base class for object segmentation
    """
    def __init__(self,image):  
        """
        Initialize the segmentation object
        @param image - image to be segmented
        @attribute points - dictionnary with segments and corresponding points 
        @attribute nb_segments - number of segments found in the image
        """
        self.img=image
        self.points=self.segmentation()
        self.nb_segments=len(self.points)
        


