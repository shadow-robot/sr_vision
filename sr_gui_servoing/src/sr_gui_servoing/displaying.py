#!/usr/bin/env python

import cv2
import numpy as np
import rospy

from sr_object_tracking.utils import Utils

from sensor_msgs.msg import Image
from sr_vision_msgs.msg import TrackBoxes


class DisplayImage(object):
    """
    Visualization class (using OpenCV tools)
    """

    def __init__(self, node_name):

        rospy.init_node(node_name)

        self.color = rospy.get_param('~color')
        self.cv_window_name = 'Video'

        # Initialize a number of global variables
        self.hist = None
        self.drag_start = (-1, -1)
        self.track_boxes = []
        self.frame = None
        self.vis = None

        self.utils = Utils(self.color)

        self.image_sub = rospy.Subscriber('camera/image_raw', Image,
                                          self.display)
        self.roi_sub = rospy.Subscriber("/roi/track_box", TrackBoxes,
                                        self.roi_callback)

        self.selection_pub = rospy.Publisher("/roi/segmented_box",
                                             TrackBoxes, queue_size=1)

    def display(self, data):
        """
        Display the main window as well as the parameters choice window and
        the histogram one (Camshift backprojection).
        Draw a rectangle around the tracking box
        """

        # Create the main display window and the histogram one
        cv2.namedWindow(self.cv_window_name, cv2.CV_WINDOW_AUTOSIZE)
        if self.hist is not None:
            cv2.namedWindow('Histogram', cv2.CV_WINDOW_AUTOSIZE)
            cv2.moveWindow("Histogram", 700, 600)

        # Set a call back on mouse clicks on the image window
        cv2.setMouseCallback(self.cv_window_name, self.on_mouse_click, None)

        # Convert the ROS image to OpenCV format using a cv_bridge helper
        # function and make a copy
        self.frame = self.utils.convert_image(data, "bgr8")
        self.vis = self.frame.copy()

        try:
            roi = self.utils.box_to_roi(self.selection)
            self.selection_pub.publish(roi)
        except (AttributeError, TypeError, IndexError):
            pass

        img = self.utils.closing

        for box in self.track_boxes:
            pt1 = (int(box.top_left.x), int(box.top_left.y))
            pt2 = (int(box.bottom_right.x), int(box.bottom_right.y))
            cv2.rectangle(self.vis, pt1, pt2, (0, 0, 255), 2)

        # Display the main window
        cv2.imshow(self.cv_window_name, np.hstack((self.vis, img)))
        cv2.waitKey(1)

    def roi_callback(self, data):
        """
        Get the ROI coordinates to display it
        """
        self.track_boxes = data.boxes

    def on_mouse_click(self, event, x, y, flags, _):
        """
        Select a ROI using the dragging
        """
        x, y = np.int16([x, y])
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag_start = (x, y)
            self.tracking_state = 0
            return
        if self.drag_start != (-1, -1):
            if flags & cv2.EVENT_FLAG_LBUTTON:
                h, w = self.frame.shape[:2]
                xo, yo = self.drag_start
                x0, y0 = np.maximum(0, np.minimum([xo, yo], [x, y]))
                x1, y1 = np.minimum([w, h], np.maximum([xo, yo], [x, y]))
                self.selection = (0, 0, 0, 0)
                if x1 - x0 > 0 and y1 - y0 > 0:
                    self.selection = x0, y0, x1 - x0, y1 - y0
                    self.track_boxes = []
                    box = self.utils.roi_to_box(self.selection, num=0)
                    self.track_boxes.append(box)
                    self.selection_pub.publish(self.track_boxes)

            else:
                self.drag_start = (-1, -1)
                if self.selection != (0, 0, 0, 0):
                    self.tracking_state = 1
                    self.selection = (0, 0, 0, 0)

    def show_hist(self):
        """
        Show the color histogram of the selected region
        """
        bin_count = self.hist.shape[0]
        bin_w = 24
        img = np.zeros((256, bin_count * bin_w, 3), np.uint8)
        for i in xrange(bin_count):
            h = int(self.hist[i])
            cv2.rectangle(img, (i * bin_w + 2, 255),
                          ((i + 1) * bin_w - 2, 255 - h),
                          (int(180.0 * i / bin_count), 255, 255), -1)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        cv2.imshow('Histogram', img)


def main():
    try:
        DisplayImage("viewer")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down sr_gui_servoing node."
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
