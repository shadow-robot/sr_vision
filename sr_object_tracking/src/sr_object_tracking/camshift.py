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
        self.name = 'Camshift'
        SrObjectTracking.__init__(self, self.name)

    def tracking(self):
        """
        Track the RegionOfInterest and return the track box updating the attribute
        """
        self.vis = self.frame.copy()
        hsv = cv2.cvtColor(self.vis, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array((0., 60., 32.)), np.array((180., 255., 255.)))
        if self.selection:
            x0, y0, x1, y1 = self.selection
            self.track_window = (x0, y0, x1 - x0, y1 - y0)
            hsv_roi = hsv[y0:y1, x0:x1]
            mask_roi = mask[y0:y1, x0:x1]
            hist = cv2.calcHist([hsv_roi], [0], mask_roi, [16], [0, 180])
            cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
            self.hist = hist.reshape(-1)
            self.show_hist()

            vis_roi = self.vis[y0:y1, x0:x1]
            cv2.bitwise_not(vis_roi, vis_roi)
            self.vis[mask == 0] = 0

        if self.tracking_state == 1:
            self.selection = None
            prob = cv2.calcBackProject([hsv], [0], self.hist, [0, 180], 1)
            prob &= mask
            term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
            self.track_box, self.track_window = cv2.CamShift(prob, self.track_window, term_crit)
            try:
                cv2.ellipse(self.vis, self.track_box, (0, 0, 255), 2)
            except:
                print self.track_box

    def show_hist(self):
        """
        Show the color histogram of the selected region
        """
        bin_count = self.hist.shape[0]
        bin_w = 24
        img = np.zeros((256, bin_count * bin_w, 3), np.uint8)
        for i in xrange(bin_count):
            h = int(self.hist[i])
            cv2.rectangle(img, (i * bin_w + 2, 255), ((i + 1) * bin_w - 2, 255 - h),
                          (int(180.0 * i / bin_count), 255, 255), -1)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        cv2.imshow('histogram', img)
