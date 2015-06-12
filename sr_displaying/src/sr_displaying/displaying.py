#!/usr/bin/env python

import cv2
import numpy as np


class DisplayImage(object):
    """
    Visualization class (using OpenCV tools)
    """
    def __init__(self):
        self.cv_window_name = 'Video'

        self.hist = None
        self.drag_start = None
        self.track_box = None
        self.tracking_state = 0
        self.selection = None
        self.frame = None
        self.frame_width = None
        self.frame_height = None
        self.frame_size = None

        # Minimum saturation of the tracked color in HSV space, and a threshold on the backprojection probability image
        self.smin = 85
        self.threshold = 50

    def display(self, algo):
        """
        Display the main window as well as the parameters choice window and the histogram one (Camshift backrpojection).
        Draw an ellipse around the tracking box
        @param algo - Tracking algorithm (Camshift for now)
        """
        # Create the main display window and the histogram one
        cv2.namedWindow(self.cv_window_name, cv2.CV_WINDOW_AUTOSIZE)
        cv2.namedWindow('Histogram', cv2.CV_WINDOW_AUTOSIZE)
        cv2.moveWindow("Histogram", 700, 20)

        # Set a call back on mouse clicks on the image window
        cv2.setMouseCallback(self.cv_window_name, self.on_mouse_click, None)

        # Create parameters window with the slider controls for saturation, value and threshold
        cv2.namedWindow("Parameters", cv2.CV_WINDOW_AUTOSIZE)
        cv2.moveWindow("Parameters", 700, 325)
        cv2.createTrackbar("Saturation", "Parameters", self.smin, 150, self.set_smin)
        cv2.createTrackbar("Threshold", "Parameters", self.threshold, 255, self.set_threshold)

        # Update the variables according to the tracking
        self.frame = algo.vis
        self.frame_size = (self.frame.shape[1], self.frame.shape[0])
        self.frame_width, self.frame_height = self.frame_size
        self.track_box = algo.track_box
        self.hist = algo.hist
        if self.hist is not None:
            self.show_hist()

        # Draw the ellipse if possible
        try:
            cv2.ellipse(self.frame, self.track_box, (0, 0, 255), 2)
        except:
            pass

        # Display the main window
        cv2.imshow(self.cv_window_name, self.frame)
        cv2.waitKey(1)

    def on_mouse_click(self, event, x, y, flags, param):
        """
        Select a ROI using the dragging
        """
        x, y = np.int16([x, y])
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag_start = (x, y)
            self.tracking_state = 0
        if self.drag_start:
            if flags & cv2.EVENT_FLAG_LBUTTON:
                h, w = self.frame.shape[:2]
                xo, yo = self.drag_start
                x0, y0 = np.maximum(0, np.minimum([xo, yo], [x, y]))
                x1, y1 = np.minimum([w, h], np.maximum([xo, yo], [x, y]))
                self.selection = None
                if x1 - x0 > 0 and y1 - y0 > 0:
                    self.selection = (x0, y0, x1, y1)

            else:
                self.drag_start = None
                if self.selection is not None:
                    self.tracking_state = 1
                    self.selection = None

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
        cv2.imshow('Histogram', img)

    # Set the callbacks for the slider controls
    def set_smin(self, pos):
        self.smin = pos

    def set_threshold(self, pos):
        self.threshold = pos
