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

import rospy
import math

from sensor_msgs.msg import Image
from sr_vision_msgs.msg import TrackBoxes

from utils import Utils


class SrObjectTracking(object):
    """
    Base class for the tracking.
    """

    def __init__(self):
        # Initialize a number of global variables
        self.size = rospy.get_param('~size')
        self.color = rospy.get_param('~color')

        self.utils = Utils(self.color)

        self.track_boxes = []
        self.seg_boxes = []

        self.prev_frame = None
        self.frame = None
        self.next_frame = None
        self.mask = None
        self.vis = None

        # Subscribe to the image topic and set the appropriate callback
        self.image_sub = rospy.Subscriber('camera/image_raw', Image,
                                          self.image_callback)
        self.segmentation_sub = rospy.Subscriber("/roi/segmented_box",
                                                 TrackBoxes,
                                                 self.segmentation_callback)

        # Initialize the Region of Interest publishers
        self.roi_pub = rospy.Publisher("roi/track_box", TrackBoxes,
                                       queue_size=1)

    def image_callback(self, data):
        """
        Convert the ROS image to OpenCV format using a cv_bridge helper
        function and make a copy
        """
        self.prev_frame = self.frame
        self.frame = self.next_frame
        self.next_frame = self.utils.convert_image(data, "bgr8")

    def segmentation_callback(self, data):
        """
        Get the ROI box (selected or segmented)
        """
        self.seg_boxes = []
        for segment in data.boxes:
            box = (int(segment.top_left.x), int(segment.top_left.y),
                   int(segment.bottom_right.x), int(segment.bottom_right.y))
            seg = SrObjectTracked(box)
            self.seg_boxes.append(seg)

    def run(self):
        """
        Track the object(s) and publish the result
        @return - Success of the tracking as a booleen
        """
        self.temp_track_boxes = self.track_boxes
        self.track_boxes = []

        if len(self.seg_boxes) > 0:
            for i, seg in enumerate(self.seg_boxes):
                s = self.tracking(seg)
                if not s or not self.checking(s):
                    self.seg_boxes.pop(i)
                    break
                else:
                    box = self.utils.roi_to_box(s, num=i)
                    self.track_boxes.append(box)
                self.roi_pub.publish(self.track_boxes)
            return True
        else:
            return False

    def tracking(self, _):
        """
        Tracking main algorithm to be redefined in the child classes
        """
        return

    def checking(self, box):
        """
        Checks if the box returned by the Camshift tracking seems correct and
         avoid the duplicates
        @param box - tracked box
        """
        # Check the size
        rect = self.utils.box_to_rect(box)
        x, y, width, height = rect
        size = width * height
        if size < 1000:
            return False

        # Check the duplication
        for box in self.temp_track_boxes:
            pt1 = (int(box.top_left.x), int(box.top_left.y))
            pt2 = (int(box.bottom_right.x), int(box.bottom_right.y))
            x2, y2 = int((pt2[0] - pt1[0]) / 2), int((pt2[0] - pt1[0]) / 2)
            distance = math.hypot(x2 - x, y2 - y)
            if distance < 100:
                return False

        return True


class SrObjectTracked(object):
    """
    Class for tracked object
    """

    def __init__(self, box):
        self.selection = box
        self.track_box = None
        self.hist = None
        self.tracking_state = 0
        self.track_window = None
