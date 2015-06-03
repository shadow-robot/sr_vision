#!/usr/bin/env python
import sys

import rospy

import cv
import cv2
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
from sr_object_segmentation.camshift import CamshiftSegmentation
import numpy as np


class Tracking:
    """
    Track a RegionOfInterest (ROI), selected with mouse, from video camera.
    """

    def __init__(self, node_name):
        self.node_name = node_name

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Initialize the Region of Interest and its publisher
        self.ROI = RegionOfInterest()
        self.roi_pub = rospy.Publisher("/roi", RegionOfInterest, queue_size=1)

        # Subscribe to the image topic and set the appropriate callback
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.image_callback)

        # Initialize a number of global variables
        self.cv_window_name = None
        self.hist = None
        self.selection = None
        self.drag_start = None
        self.track_box = None
        self.track_window = None
        self.tracking_state = 0
        self.frame = None
        self.frame_width = None
        self.frame_height = None
        self.frame_size = None

    def image_callback(self, data):
        """
        Process the image from video camera (Freenect topic) in order to track a ROI, and show it.
        """

        # Convert the ROS image to OpenCV format using a cv_bridge helper function and make a copy
        self.frame = self.convert_image(data)

        # Create the main display window and the histogram one
        self.cv_window_name = self.node_name
        cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_AUTOSIZE)
        cv.NamedWindow('histogram', cv.CV_WINDOW_AUTOSIZE)

        # Store the frame width and height in a pair of global variables
        if self.frame_width is None:
            self.frame_size = (self.frame.shape[1], self.frame.shape[0])
            self.frame_width, self.frame_height = self.frame_size

        # Set a call back on mouse clicks on the image window
        cv.SetMouseCallback(self.node_name, self.on_mouse_click, None)

        # Initialize the tracking algorithm
        self.algo = CamshiftSegmentation(None)

        # Make a copy and process the image
        vis = self.frame.copy()
        hsv = cv2.cvtColor(vis, cv2.COLOR_BGR2HSV)
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

            vis_roi = vis[y0:y1, x0:x1]
            cv2.bitwise_not(vis_roi, vis_roi)
            vis[mask == 0] = 0

        if self.tracking_state == 1:
            self.selection = None
            self.algo.tracking(hsv, mask, self.track_window, self.hist)
            self.track_window = self.algo.track_window
            self.track_box = self.algo.track_box

            try:
                cv2.ellipse(vis, self.track_box, (0, 0, 255), 2)
            except:
                print self.track_box

        self.publish_roi()

        cv2.imshow(self.cv_window_name, vis)
        cv2.waitKey(5)

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

    def convert_image(self, ros_image):
        """
        Use cv_bridge() to convert the ROS image to OpenCV format
        @param ros_image - image in ROS format, to be converted in OpenCV format
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            # cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            return np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e

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

    def publish_roi(self):
        if not self.drag_start:
            if self.track_box is not None:
                roi_box = self.track_box
            else:
                return
        try:
            roi_box = box_to_rect(roi_box)
        except:
            return

        # Watch out for negative offsets
        roi_box[0] = max(0, roi_box[0])
        roi_box[1] = max(0, roi_box[1])

        try:
            roi = RegionOfInterest()
            roi.x_offset = int(roi_box[0])
            roi.y_offset = int(roi_box[1])
            roi.width = int(roi_box[2])
            roi.height = int(roi_box[3])
            self.roi_pub.publish(roi)
        except:
            rospy.loginfo("Publishing ROI failed")


def box_to_rect(roi):
    try:
        if len(roi) == 3:
            (center, size, angle) = roi
            pt1 = (int(center[0] - size[0] / 2), int(center[1] - size[1] / 2))
            pt2 = (int(center[0] + size[0] / 2), int(center[1] + size[1] / 2))
            rect = [pt1[0], pt1[1], pt2[0] - pt1[0], pt2[1] - pt1[1]]
        else:
            rect = list(roi)
    except:
        return [0, 0, 0, 0]

    return rect


def main(args):
    try:
        node_name = "tracking"
        rospy.init_node(node_name)
        rospy.loginfo("Starting node " + str(node_name))
        Tracking(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down tracking node."
        cv.DestroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
