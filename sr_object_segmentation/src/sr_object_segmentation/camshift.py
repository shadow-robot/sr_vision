#!/usr/bin/env python

import cv2
import numpy as np

from sr_object_segmentation import SrObjectSegmentation


class CamshiftSegmentation(SrObjectSegmentation):
    """
    Segmentation based upon the CamShift algorithm, from the OpenCV library
    """
    def __init__(self, image):
        """
        Initialize the CamShift segmentation object
        @param image - image to be segmented (numpy format)
        """
        SrObjectSegmentation.__init__(self, image, {})


    def tracking(self, hsv, mask, track_window, hist):
        """
        Track the RegionOfInterest and return the track box updating the attribute
        @param hsv - image in HSV (or HUE)
        @param mask - keep only the most colored regions of the image
        @param track_window - window for the tracking
        @param hist - color histogram of the selected region
        """
        prob = cv2.calcBackProject([hsv], [0], hist, [0, 180], 1)
        prob &= mask
        term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
        try:
            self.track_box, self.track_window = cv2.CamShift(prob, track_window, term_crit)

        except:
            self.track_box, self.track_window = None, None



    def show_hist(self):
        bin_count = self.hist.shape[0]
        bin_w = 24
        img = np.zeros((256, bin_count * bin_w, 3), np.uint8)
        for i in xrange(bin_count):
            h = int(self.hist[i])
            cv2.rectangle(img, (i * bin_w + 2, 255), ((i + 1) * bin_w - 2, 255 - h),
                          (int(180.0 * i / bin_count), 255, 255), -1)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        cv2.imshow('histogram', img)
