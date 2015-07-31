#!/usr/bin/env python

import cv2

from sr_object_segmentation import SrObjectSegmentation


class ShapeColorSegmentation(SrObjectSegmentation):
    """
    Shape and color based segmentation
    """

    def __init__(self, color, shape, shape_threshold, size):
        SrObjectSegmentation.__init__(self)
        self.name = 'Shape and color based segmentation algorithm'
        self.color = color
        self.shape = shape
        self.shape_threshold = shape_threshold
        self.size = size

    def segmentation(self, frame):
        """
        Segment the image with the proper color and a weight according to the
        shape. Update the box attribute.
        @param frame - image to be segmented (numpy format)
        """
        self.frame = frame

        closing = self.utils.hsv_transform(self.frame, self.color, None)
        closing_gray = cv2.cvtColor(closing, cv2.COLOR_BGR2GRAY)

        seg_cnts = self.match_shapes(closing_gray)

        if seg_cnts:
            for seg in seg_cnts:
                self.segmented_box.append(cv2.boundingRect(seg))
                M = cv2.moments(seg)
                centroid = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
                self.poses.append(centroid)
                self.nb_segments = len(self.segmented_box)

    def match_shapes(self, img):
        """
        Compare the shape of the segments found with a model (loaded from the
        launch file) in order to eliminate the ones below a threshold
        @param img - pre-processed 8-bit image showing the segments found
        @return - contours of the best corresponding segments
        """
        model_cnt = self.shape

        (cnts, _) = cv2.findContours(img.copy(), cv2.RETR_EXTERNAL,
                                     cv2.CHAIN_APPROX_SIMPLE)
        cnts[:] = [cnt for cnt in cnts if cv2.contourArea(cnt) > self.size]

        match_shapes = []
        for cnt in cnts:
            cnt = cv2.convexHull(cnt)
            score = cv2.matchShapes(model_cnt, cnt,
                                    cv2.cv.CV_CONTOURS_MATCH_I1, 100)
            if score < self.shape_threshold:
                match_shapes.append(cnt)

        return match_shapes
