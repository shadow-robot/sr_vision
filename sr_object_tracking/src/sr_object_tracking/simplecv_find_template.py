#!/usr/bin/env python

import SimpleCV as sCV
import cv2
import numpy as np
from sr_object_tracking import SrObjectTracking


class SimpleCV_FindTemplate(SrObjectTracking):
    """
    Tracking based upon searching a specified feature, with a SimpleCV algorithm
    """

    def __init__(self):
        """
        Initialize the CamShift segmentation object
        """
        self.name = 'SimpleCV_FindTemplate'
        SrObjectTracking.__init__(self, self.name)


    def tracking(self):
        """
        Track the RegionOfInterest and return the track box updating the attribute
        """
        self.vis = self.frame.copy()
        self.scv_vis = sCV.Image(self.vis,cv2image=True)
        hsv = cv2.cvtColor(self.vis, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array((0., 60., 32.)), np.array((180., 255., 255.)))
        if self.selection:
            x0, y0, x1, y1 = self.selection
            self.track_window = self.scv_vis[x0:x1, y0:y1]
            self.track_window.show()

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
            match = self.scv_vis.findTemplate(self.track_window,5,'CCOEFF')
            if len(match) != 0 :
                self.match = match
                xmin, ymin = self.match.x() - self.match.width() / 2 , self.match.y() - self.match.height() / 2
                width, height = self.match.width(), self.match.height()
                xmin, ymin, width, height = xmin[0], ymin[0], width[0], height[0]

                center = (xmin + width / 2, ymin + height / 2)
                size = (width, height)
                angle = 0
                self.track_box = (center, size, angle)

                try:
                    print track_box
                    cv2.ellipse(self.vis, self.track_box, (0, 0, 255), 2)

                except:
                    print self.track_box


    def segmentation(self):
        """
        Segment the image searching the feature given as parameter
        @return - dictionary of the feature points coordinates
        """

        #methods = ["SQR_DIFF", "SQR_DIFF_NORM", "CCOEFF", "CCOEFF_NORM", "CCORR","CCORR_NORM"]  # the various types of template matching available
        methods = ['CCOEFF']
        t = 5  # Threshold
        found = False
        matches = []
        source=sCV.Image(self.img)
        while not found and t > 0:
            for m in methods:
                #print "current method:", m  # print the method being used
                #result = sCV.Image('/home/glassbot/Desktop/strawberry_table.jpg', sample=True)
                #dl = sCV.DrawingLayer((source.width, source.height))

                temp = source.findTemplate(self.feature, threshold=t, method=m)
                if len(temp) != 0:
                    matches.append((temp, m))
                    found = True
            t -= 1

        if matches:
            dic = {}
            match = matches[0][0]
            points = []
            xmin, ymin = match.x() - match.width() / 2 , match.y() - match.height() / 2
            xmax, ymax = match.x() + match.width() / 2 , match.y() + match.height() / 2

            for x in range(xmin, xmax):
                for y in range(ymin, ymax):
                    points.append((x, y))
            dic[0] = points

            # Sort by descending size of segments
            seg_by_length = sorted(dic.values(), key=len, reverse=True)

            for i in range(len(dic)):
                dic[i] = seg_by_length[i]

            return dic
        else:
            print 'Feature not found'


