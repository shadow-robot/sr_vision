#!/usr/bin/env python

import cv2
import rospy
import numpy as np
from sr_object_tracking import SrObjectTracking


class SequentialTracking(SrObjectTracking):
    """
    Tracking based upon the Camshift algorithm improved by a motion detection.
    """

    def __init__(self):
        """
        Initialize the tracking object
        """
        SrObjectTracking.__init__(self)

    def tracking(self):
        """
        Track the RegionOfInterest return the track box updating the attribute
        @return - Success of the tracking as a booleen
        """
        self.vis = self.frame.copy()

        self.frame = cv2.blur(self.frame, (5, 5))
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array((0., self.smin, 54)), np.array((180., 255., 255)))

        seq = self.sequential_process(hsv)

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
            prob = prob + seq
            prob &= mask
            term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
            nb_iter = cv2.meanShift(prob, self.track_window, term_crit)[0]
            if nb_iter != 0:
                self.track_box, self.track_window = cv2.CamShift(prob, self.track_window, term_crit)

        roi = self.utils.publish_box(self.track_box)

        # Make sure that the object is still tracked, otherwise launch the segmentation
        if roi.width * roi.height < 20:
            return False

        self.roi_pub.publish(roi)
        return True


    def sequential_process(self, hsv):

        # Motion detection using a differential image calculated from three consecutive frames
        d1 = cv2.absdiff(self.next_frame, self.frame)
        d2 = cv2.absdiff(self.frame, self.prev_frame)
        diff_frame = cv2.bitwise_and(d1, d2)
        diff_frame = cv2.cvtColor(diff_frame, cv2.COLOR_BGR2GRAY)

        img_thresh = cv2.inRange(hsv, self.lower, self.upper)
        img = cv2.bitwise_and(self.frame, self.frame, mask=img_thresh)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        return img + diff_frame