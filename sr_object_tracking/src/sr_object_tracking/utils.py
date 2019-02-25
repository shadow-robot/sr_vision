#!/usr/bin/env python

# Copyright 2019 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

import os
import rospy
import yaml

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Point
from sr_vision_msgs.msg import Box

from image_geometry import PinholeCameraModel


class Utils(object):
    """
    Utils methods for the sr_vision package
    """

    def __init__(self, color):
        self.color = color

        self.bridge = CvBridge()
        self.depth = None
        self.cam_info = None

        path = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.realpath(__file__)))) + '/config/boundaries.yaml'
        with open(path, 'r') as param:
            self.boundaries = yaml.load(param)

        self.image_sub = rospy.Subscriber('camera/image_raw', Image,
                                          self.image_callback)
        self.depth_sub = rospy.Subscriber('camera/depth/image_rect_raw', Image,
                                          self.depth_callback)
        self.cam_info = rospy.Subscriber('camera/camera_info', CameraInfo,
                                         self.cam_info_callback)

    def depth_callback(self, data):
        """
        Convert the ROS image to OpenCV format using a cv_bridge helper
        function and make a copy
        """
        self.depth = self.convert_image(data, "passthrough")

    def image_callback(self, data):
        """
        Convert the ROS image to OpenCV format using a cv_bridge helper
        function and make a copy
        """
        self.frame = self.convert_image(data, "bgr8")
        self.mask = self.get_mask(self.color, boundaries=None)
        self.closing = self.get_closing(self.mask)

    def cam_info_callback(self, data):
        """
        Get the camera information from its calibration
        """
        self.cam_info = data

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

    def roi_to_box(self, roi, moment=None, num=0):

        rect = self.box_to_rect(roi)

        box = Box()
        box.id = num
        box.top_left = Point(rect[0], rect[1], 0)
        box.bottom_right = Point(rect[0] + rect[2], rect[1] + rect[3], 0)
        box.ray_to_centroid = self.centroid_to_pose(rect, moment)

        return box

    def centroid_to_pose(self, rect, moment):
        """
        Publish the region of interest as a geometry_msgs/Pose message from
        the ROI or the center of mass coordinates
        @param rect - sensor_msgs/RegionOfInterest
        @param moment - object's center of mass
        """
        if not moment:
            moment = (int(rect[0] + rect[2] / 2), int(rect[1] + rect[3] / 2))
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

    def get_mask(self, color, boundaries):
        """
        @param color - Color wanted
        @return - Mask corresponding to the color
        """
        if boundaries:
            (lower, upper) = boundaries
        else:
            (lower, upper) = self.boundaries[color]

        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")

        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)

        return mask

    def get_closing(self, mask):
        """
        Color closing image processing
        @param mask - color of interest mask
        @return - output image with black background and "color" segments
        highlighted
        """
        img_col = cv2.bitwise_and(self.frame, self.frame, mask=mask)

        kernel = np.ones((5, 5), np.uint8)
        opening = cv2.morphologyEx(img_col, cv2.MORPH_OPEN, kernel)
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

        return closing
