#!/usr/bin/env python

import cv2
import numpy as np
from scipy.ndimage.measurements import label, find_objects, center_of_mass
from sr_object_segmentation import SrObjectSegmentation


class HSVSegmentation(SrObjectSegmentation):
    """
    Segmentation based upon an HSV (Hue, Saturation, Value) colorization, according to a color given as parameter.
    """

    def __init__(self, color):
        """
        Initialize the HSV_based segmentation object 
        """
        SrObjectSegmentation.__init__(self, None, {})
        self.name = 'HSV segmentation algorithm'
        self.segmented_box = None
        self.nb_segments = 0
        self.color = color

    def segmentation(self, frame):
        """
        Segment the image with the proper color. Update the box attribute according to the biggest segment found.
        @param frame - image to be segmented (numpy format)
        """
        self.img = frame

        closing = hsv_transform(self.img, self.color)

        labeled_array, num_features = label(closing)
        objects = find_objects(labeled_array)
        self.nb_segments = num_features

        self.poses = center_of_mass(closing, labeled_array)

        segments = []
        for seg in objects:
            seg_x = seg[1]
            seg_y = seg[0]

            segments.append((int(seg_x.start), int(seg_y.start),
                             int(seg_x.stop - seg_x.start),
                             int(seg_y.stop - seg_y.start)))

        # Order the slices according to their size
        segments.sort(key=lambda s: s[2] * s[3], reverse=True)

        # Pick the biggest one
        self.segmented_box = segments[0]


def hsv_transform(img, color):
    """
    Convert an RGB image into an HSV unique color one
    @param img - input image to be formatted
    @param color - color wanted
    @return - output image with black background and "color" segments highlighted
    """

    boundaries = {
        'red': ([145, 140, 0], [255, 255, 255]),
        'blue': ([100, 110, 0], [125, 255, 255]),
        'green': ([30, 115, 0], [65, 255, 255]),
        'yellow': ([10, 80, 150], [20, 255, 255])
    }
    (lower, upper) = boundaries[color]
    lower = np.array(lower, dtype="uint8")
    upper = np.array(upper, dtype="uint8")

    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_thresh = cv2.inRange(img_hsv, lower, upper)
    img_col = cv2.bitwise_and(img, img, mask=img_thresh)

    kernel = np.ones((5, 5), np.uint8)
    opening = cv2.morphologyEx(img_col, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

    return closing
