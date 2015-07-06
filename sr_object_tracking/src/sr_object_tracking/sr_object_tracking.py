#!/usr/bin/env python
import rospy
import numpy as np

from sensor_msgs.msg import Image, RegionOfInterest

from utils import Utils


class SrObjectTracking(object):
    """
    Base class for the tracking.
    """

    def __init__(self):
        self.utils = Utils()

        # Initialize a number of global variables
        self.smin = rospy.get_param('/tracking/saturation_min')
        self.size = rospy.get_param('/tracking/size')
        self.color = rospy.get_param('/tracking/color')
        self.track_box = None
        self.track_window = None
        self.tracking_state = 0
        self.selection = (0, 0, 0, 0)
        self.prev_frame = None
        self.frame = None
        self.next_frame = None
        self.depth_image = None
        self.vis = None
        self.hist = None

        boundaries = {
            'red': ([145, 140, 0], [255, 255, 255]),
            'blue': ([100, 110, 0], [125, 255, 255]),
            'green': ([30, 115, 0], [65, 255, 255]),
            'yellow': ([10, 80, 150], [20, 255, 255])
        }
        (self.lower, self.upper) = boundaries[self.color]
        self.lower = np.array(self.lower, dtype="uint8")
        self.upper = np.array(self.upper, dtype="uint8")

        # Subscribe to the image topic and set the appropriate callback
        self.image_sub = rospy.Subscriber('/camera/rgb/image_color', Image,
                                          self.image_callback)
        self.selection_sub = rospy.Subscriber("roi/segmented_box",
                                              RegionOfInterest,
                                              self.selection_callback)
        self.selection_sub = rospy.Subscriber("roi/selection",
                                              RegionOfInterest,
                                              self.selection_callback)

        # Initialize the Region of Interest publishers
        self.roi_pub = rospy.Publisher("roi/track_box", RegionOfInterest,
                                       queue_size=1)

    def image_callback(self, data):
        """
        Convert the ROS image to OpenCV format using a cv_bridge helper
        function and make a copy
        """
        self.prev_frame = self.frame
        self.frame = self.next_frame
        self.next_frame = self.utils.convert_image(data, "bgr8")

    def selection_callback(self, data):
        """
        Get the ROI box (selected or segmented)
        """
        self.selection = (
        data.x_offset, data.y_offset, data.x_offset + data.width,
        data.y_offset + data.height)
