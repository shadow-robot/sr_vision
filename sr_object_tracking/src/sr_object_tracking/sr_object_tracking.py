#!/usr/bin/env python
import rospy

from sr_vision_msgs.msg import tracking_parameters
from sensor_msgs.msg import Image, RegionOfInterest
from utils import Utils


class SrObjectTracking(object):
    """
    Base class for the tracking.
    """

    def __init__(self):

        self.utils = Utils()

        # Initialize a number of global variables
        self.smin = 150
        self.threshold = 50
        self.track_box = None
        self.track_window = None
        self.tracking_state = 0
        self.selection = [0,0,0,0]
        self.frame = None
        self.depth_image = None
        self.vis = None
        self.hist = None

        # Subscribe to the image topic and set the appropriate callback
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.image_callback)
        self.selection_sub = rospy.Subscriber("/roi/segmented_box", RegionOfInterest, self.selection_callback)
        self.param_sub = rospy.Subscriber("/roi/parameters", tracking_parameters, self.param_callback)

        # Initialize the Region of Interest publishers
        self.roi_pub = rospy.Publisher("/roi/track_box", RegionOfInterest, queue_size=1)

    def image_callback(self, data):
        """
        Convert the ROS image to OpenCV format using a cv_bridge helper function and make a copy
        """
        self.frame = self.utils.convert_image(data, "bgr8")

    def selection_callback(self, data):
        """
        Get the ROI box (selected or segmented)
        """
        self.selection = (data.x_offset, data.y_offset, data.x_offset + data.width, data.y_offset + data.height)

    def param_callback(self, data):
        """
        Update several algorithm parameters that could be changed by the user
        """
        self.smin = data.smin
        self.threshold = data.threshold
        # self.tracking_state = data.tracking_state
