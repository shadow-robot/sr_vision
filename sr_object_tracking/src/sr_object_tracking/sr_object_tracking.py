#!/usr/bin/env python
import rospy

from cv_bridge import CvBridge, CvBridgeError

from sr_gui_servoing.msg import tracking_parameters
from sensor_msgs.msg import Image, PointCloud2, RegionOfInterest, PointField


class SrObjectTracking(object):
    """
    Base class for the tracking.
    """

    def __init__(self):

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Initialize a number of global variables
        self.smin = 85
        self.threshold = 50
        self.track_box = None
        self.track_window = None
        self.tracking_state = 0
        self.selection = None
        self.frame = None
        self.depth_image = None
        self.vis = None
        self.hist = None

        # Subscribe to the image topic and set the appropriate callback
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.image_callback)
        self.selection_sub = rospy.Subscriber("/roi/selection", RegionOfInterest, self.selection_callback)
        self.param_sub = rospy.Subscriber("/roi/parameters", tracking_parameters, self.param_callback)

        # Initialize the Region of Interest publishers
        self.roi_pub = rospy.Publisher("/roi/track_box", RegionOfInterest, queue_size=1)

    def image_callback(self, data):
        """
        Convert the ROS image to OpenCV format using a cv_bridge helper function and make a copy
        """
        self.frame = self.convert_image(data, "bgr8")

    def selection_callback(self, data):
        """
        Get the ROI box (selected or segmented)
        """
        self.selection = (data.x_offset, data.y_offset, data.x_offset + data.width, data.y_offset + data.height)

    def camera_cloud_callback(self, data):
        self.camera_cloud = data

    def param_callback(self, data):
        """
        Update several algorithm parameters that could be changed by the user
        """
        self.smin = data.smin
        self.threshold = data.threshold
        self.tracking_state = data.tracking_state

    def publish_roi(self):
        """
        Publish the region of interest as a RegionOfInterest message (2D box)
        """
        roi_box = box_to_rect(self.track_box)

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

    def convert_image(self, ros_image, encode):
        """
        Use cv_bridge() to convert the ROS image to OpenCV format
        @param ros_image - image in ROS format, to be converted in OpenCV format
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, encode)
            return cv_image
        except CvBridgeError, e:
            print e


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
