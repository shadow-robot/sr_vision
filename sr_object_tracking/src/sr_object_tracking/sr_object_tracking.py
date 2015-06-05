#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class SrObjectTracking(object):
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
        self.vis = None

    def image_callback(self, data):
        """
        Process the image from video camera (Freenect topic) in order to track a ROI, and show it.
        """

        # Convert the ROS image to OpenCV format using a cv_bridge helper function and make a copy
        self.frame = self.convert_image(data)

        # Create the main display window and the histogram one
        self.cv_window_name = self.node_name
        cv2.namedWindow(self.cv_window_name, cv2.CV_WINDOW_AUTOSIZE)
        cv2.namedWindow('Histogram', cv2.CV_WINDOW_AUTOSIZE)
        cv2.moveWindow("Histogram", 700, 20)

        # Store the frame width and height in a pair of global variables
        if self.frame_width is None:
            self.frame_size = (self.frame.shape[1], self.frame.shape[0])
            self.frame_width, self.frame_height = self.frame_size

        # Set a call back on mouse clicks on the image window
        cv2.setMouseCallback(self.node_name, self.on_mouse_click, None)

        # Process the tracking initializing the algorithm
        self.tracking()
        # self.publish_roi()

        cv2.imshow(self.cv_window_name, self.vis)
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
            return np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e

    def publish_roi(self):
        """
        Publish the region of interest
        """
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
    """
    Convert a region of interest as box format into a rect format
    @param roi - region of interest (box format)
    @return - region of interest (rect format)
    """
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


def run(algo):
    """
    Initialize the tracking node and process the tracking
    @param algo - Algorithm to be used for tracking
    """
    try:
        node_name = "tracking"
        rospy.init_node(node_name)
        rospy.loginfo("Starting node " + str(node_name))
        algo()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down tracking node."
        cv2.destroyAllWindows()
