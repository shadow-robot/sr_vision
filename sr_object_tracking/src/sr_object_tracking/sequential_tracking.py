#!/usr/bin/env python

# Copyright 2019 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

import cv2
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
        self.hsv = None
        self.mask = None

    def tracking(self, segment):
        """
        Tracks the segment given as parameter.
        @param segment - SrObjectTracked object to be tracked
        @return - False if the tracking fails, and the new coordinates of the
         tracked object otherwise.
        """
        self.frame = cv2.blur(self.frame, (5, 5))
        self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        self.mask = cv2.inRange(self.hsv, np.array((0., 150, 54)),
                                np.array((180., 255., 255)))

        if segment.selection != (0, 0, 0, 0):
            x0, y0, x1, y1 = segment.selection
            segment.track_window = (x0, y0, x1 - x0, y1 - y0)
            hsv_roi = self.hsv[y0:y1, x0:x1]
            mask_roi = cv2.inRange(hsv_roi, np.array((0., 60., 32.)),
                                   np.array((180., 255., 255.)))
            hist = cv2.calcHist([hsv_roi], [0], mask_roi, [16], [0, 180])
            cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
            segment.hist = hist.reshape(-1)
            segment.tracking_state = 1

        if segment.tracking_state == 1:
            prob = cv2.calcBackProject([self.hsv], [0], segment.hist, [0, 180],
                                       1)
            seq = self.sequential_process()
            prob = prob + seq
            prob &= self.mask
            term_crit = (
                cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.011)
            nb_iter = cv2.meanShift(prob, segment.track_window, term_crit)[0]
            if nb_iter != 0:
                segment.track_box, segment.track_window = \
                    cv2.CamShift(prob, segment.track_window, term_crit)
            else:
                return False

        return segment.track_box

    def sequential_process(self):
        """
        Motion detection using a differential image calculated from three
        consecutive frames
        @return - processed image
        """
        img = self.utils.mask

        try:
            d1 = cv2.absdiff(self.next_frame, self.frame)
            d2 = cv2.absdiff(self.frame, self.prev_frame)
            diff_frame = cv2.bitwise_and(d1, d2)
            diff_frame = cv2.cvtColor(diff_frame, cv2.COLOR_BGR2GRAY)

            return diff_frame + img
        except cv2.error:
            return img
