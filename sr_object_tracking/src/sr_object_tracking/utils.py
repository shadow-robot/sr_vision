#!/usr/bin/env python

from sensor_msgs.msg import RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError


class Utils(object):
    def __init__(self):
        # Create the cv_bridge object
        self.bridge = CvBridge()

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
                pt1 = (int(center[0] - size[0] / 2), int(center[1] - size[1] / 2))
                pt2 = (int(center[0] + size[0] / 2), int(center[1] + size[1] / 2))
                rect = [pt1[0], pt1[1], pt2[0] - pt1[0], pt2[1] - pt1[1]]
            else:
                rect = list(roi)
        except (IndexError, TypeError, ValueError):
            return [0, 0, 0, 0]
        return rect
