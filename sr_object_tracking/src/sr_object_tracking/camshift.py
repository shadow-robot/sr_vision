#!/usr/bin/env python

import cv2
import numpy as np
from sr_object_tracking import SrObjectTracking


class CamshiftTracking(SrObjectTracking):
    """
    Tracking based upon the CamShift algorithm, from the OpenCV library
    """

    def __init__(self):
        """
        Initialize the CamShift segmentation object
        """
        SrObjectTracking.__init__(self)

    def tracking(self):
        """
        Track the RegionOfInterest and return the track box as attribute
        @return - Success of the tracking as a booleen
        """
        self.vis = self.frame.copy()

        # Seems better with a blur
        self.vis = cv2.blur(self.vis, (5, 5))
        hsv = cv2.cvtColor(self.vis, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array((0., self.smin, 54)),
                           np.array((180., 255., 255)))

        if self.selection != (0, 0, 0, 0):
            x0, y0, x1, y1 = self.selection
            self.track_window = (x0, y0, x1 - x0, y1 - y0)
            hsv_roi = hsv[y0:y1, x0:x1]
            mask_roi = mask[y0:y1, x0:x1]
            hist = cv2.calcHist([hsv_roi], [0], mask_roi, [16], [0, 180])
            cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
            self.hist = hist.reshape(-1)
            self.tracking_state = 1

        if self.tracking_state == 1:
            prob = cv2.calcBackProject([hsv], [0], self.hist, [0, 180], 1)
            prob &= mask
            term_crit = (
                cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
            nb_iter = cv2.meanShift(prob, self.track_window, term_crit)[0]
            if nb_iter != 0:
                self.track_box, self.track_window = cv2.CamShift(
                    prob, self.track_window, term_crit)

        roi = self.utils.publish_box(self.track_box)

        # Make sure that the object is still tracked, otherwise launch the
        # segmentation
        if roi.width * roi.height < 50:
            return False

        self.roi_pub.publish(roi)
        return True
