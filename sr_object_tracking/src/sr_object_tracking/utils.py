#!/usr/bin/env python

import os
import rospy
import yaml

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from geometry_msgs.msg import Pose

from image_geometry import PinholeCameraModel


class Utils(object):
    def __init__(self):

        self.bridge = CvBridge()
        self.depth = None
        self.cam_info = None

        self.depth_sub = rospy.Subscriber('camera/depth/image_rect_raw', Image,
                                          self.depth_callback)
        self.cam_info = rospy.Subscriber('camera/camera_info', CameraInfo,
                                         self.cam_info_callback)

        # Initialize the Region of Interest publishers
        self.roi_pub = rospy.Publisher("roi/track_box", RegionOfInterest,
                                       queue_size=1)
        self.pose_pub = rospy.Publisher("roi/pose", Pose, queue_size=1)

    def convert_image(self, ros_image, encode):
        """
        Use cv_bridge() to convert the ROS image to OpenCV format
        @param ros_image - image in ROS format, to be converted
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

    def cam_info_callback(self, data):
        """
        Get the camera information from its calibration
        """
        self.cam_info = data

    def publish_pose(self, roi=None, moment=None):
        """
        Publish the region of interest as a geometry_msgs/Pose message from
        the ROI or the center of mass coordinates
        @param roi - sensor_msgs/RegionOfInterest
        @param moment - object's center of mass
        """
        if not moment:
            moment = (roi.x_offset + int(roi.width / 2),
                      roi.y_offset + int(roi.height / 2))

        u, v = int(moment[0]), int(moment[1])

        ps = Pose()
        ps.orientation.x = 0.0
        ps.orientation.y = 0.0
        ps.orientation.z = 0.0
        ps.orientation.w = 1.0

        cam = PinholeCameraModel()
        cam.fromCameraInfo(self.cam_info)
        (x, y, z) = cam.projectPixelTo3dRay((u, v))

        ps.position.x = x
        ps.position.y = y
        ps.position.z = z

        return ps

    def box_to_roi(self, box):
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

    @staticmethod
    def box_to_rect(box):
        """
        Convert a region of interest as box format into a rect format
        @param box - region of interest (box format)
        @return - region of interest (rect format)
        """
        try:
            if len(box) == 3:
                (center, size, _) = box
                pt1 = (
                    int(center[0] - size[0] / 2), int(center[1] - size[1] / 2))
                pt2 = (
                    int(center[0] + size[0] / 2), int(center[1] + size[1] / 2))
                rect = [pt1[0], pt1[1], pt2[0] - pt1[0], pt2[1] - pt1[1]]
            else:
                rect = list(box)
        except (IndexError, TypeError, ValueError):
            return [0, 0, 0, 0]
        return rect

    @staticmethod
    def hsv_transform(img, color, boundaries=None):
        """
        Convert an RGB image into an HSV unique color one
        @param img - input image to be formatted
        @param color - color wanted
        @return - output image with black background and "color" segments
        highlighted
        """
        if boundaries:
            (lower, upper) = boundaries
        else:
            boundaries = {
                'red': ([0, 120, 0], [83, 255, 255]),
                'blue': ([100, 110, 0], [125, 255, 255]),
                'green': ([30, 115, 0], [65, 255, 255]),
                'yellow': ([10, 80, 150], [20, 255, 255])
            }
            if color == 'custom':
                path = os.path.dirname(os.path.dirname(os.path.dirname(
                    os.path.realpath(__file__)))) + '/config/boundaries.yaml'
                with open(path, 'r') as param:
                    boundaries['custom'] = yaml.load(param)

            (lower, upper) = boundaries[color]

        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")

        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_thresh = cv2.inRange(img_hsv, lower, upper)

        img_col = cv2.bitwise_and(img, img, mask=img_thresh)

        kernel = np.ones((5, 5), np.uint8)
        opening = cv2.morphologyEx(img_col, cv2.MORPH_OPEN, kernel)
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

        return closing
