#!/usr/bin/env python

import cv2
import numpy as np

from sr_object_segmentation import SrObjectSegmentation


class HSVSegmentation(SrObjectSegmentation):
    """
    Segmentation based upon an HSV colorization (Hue, Saturation, Value parameters controlled by trackbars in the 
    display mode
    """

    def __init__(self):
        """
        Initialize the HSV_based segmentation object 
        """
        SrObjectSegmentation.__init__(self, None, {})
        self.name = 'HSV segmentation algorithm'
        self.points = None
        self.nb_segments = 0

    def segmentation(self, frame, size=100):
        """
        Segment the image into blobs
        @param frame - image to be segmented (numpy format)
        @param size - approximate size of the object of interest
        @return - coordinates of the ROI box (tuple)
        
        """
        self.img = frame

        (lower, upper) = ([0, 165, 0], [255, 255, 255])
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")

        img_hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        img_thresh = cv2.inRange(img_hsv, lower, upper)
        img = cv2.bitwise_and(self.img, self.img, mask=img_thresh)

        kernel = np.ones((5, 5), np.uint8)
        opening = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

        imgray = cv2.cvtColor(closing, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(imgray, 100, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        max_area = size
        self.points = contours[0][0][0]
        self.nb_segments = len(hierarchy)
        if len(hierarchy) > 0:
            for i in range(len(hierarchy)):
                area = cv2.contourArea(contours[i])
                if area > max_area:
                    self.points = contours[i][0][0]
                    size = area

        return (
            self.points[0] - size / 2, self.points[1] - size / 2, self.points[0] + size / 2, self.points[1] + size / 2)
