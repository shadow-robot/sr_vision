#!/usr/bin/env python

import rospy

from sensor_msgs.msg import RegionOfInterest
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, RegionOfInterest
from geometry_msgs.msg import PoseStamped


class Utils(object):
    def __init__(self):
        # Create the cv_bridge object
        self.bridge = CvBridge()
        self.depth = None

        self.depth_sub = rospy.Subscriber('camera/depth/image_rect_raw', Image,
                                          self.depth_callback)

        # Initialize the Region of Interest publishers
        self.roi_pub = rospy.Publisher("roi/track_box", RegionOfInterest,
                                       queue_size=1)
        self.pose_pub = rospy.Publisher("roi/pose", PoseStamped, queue_size=1)

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

    def depth_callback(self, data):
        """
        Convert the ROS image to OpenCV format using a cv_bridge helper
        function and make a copy
        """

        self.depth = self.convert_image(data, "passthrough")

    def publish_pose(self, roi=None, moment=None):
        """
        Publish the region of interest as a geometry_msgs/PoseStamped message
        from the ROI or the center of mass coordinates
        @param roi - sensor_msgs/RegionOfInterest
        @param moment - object's center of mass
        """
        if not moment:
            moment = (roi.x_offset + int(roi.width / 2),
                      roi.y_offset + int(roi.height / 2))

        ps = PoseStamped()
        ps.header.frame_id = "/camera_link"
        ps.pose.orientation.w = 1
        x, y = int(moment[0]), int(moment[1])
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = self.depth[y, x]

        return ps

    def publish_box(self, box):
        """
        Publish the region of interest as a RegionOfInterest message (2D box)
        """
        roi_box = self.box_to_rect(box)

        # Watch out for negative offsets
        roi_box[0] = max(0, roi_box[0])
        roi_box[1] = max(0, roi_box[1])

        roi = RegionOfInterest()
        roi.x_offset = int(roi_box[0])
        roi.y_offset = int(roi_box[1])
        roi.width = int(roi_box[2])
        roi.height = int(roi_box[3])
        return roi

    def box_to_rect(self, roi):
        """
        Convert a region of interest as box format into a rect format
        @param roi - region of interest (box format)
        @return - region of interest (rect format)
        """
        try:
            if len(roi) == 3:
                (center, size, _) = roi
                pt1 = (
                    int(center[0] - size[0] / 2), int(center[1] - size[1] / 2))
                pt2 = (
                    int(center[0] + size[0] / 2), int(center[1] + size[1] / 2))
                rect = [pt1[0], pt1[1], pt2[0] - pt1[0], pt2[1] - pt1[1]]
            else:
                rect = list(roi)
        except (IndexError, TypeError, ValueError):
            return [0, 0, 0, 0]
        return rect
