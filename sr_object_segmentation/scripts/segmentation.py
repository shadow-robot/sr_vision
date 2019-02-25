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
import numpy as np

from sr_object_segmentation.shape_color_segmentation import \
    ShapeColorSegmentation
from sr_object_tracking.utils import Utils

from sensor_msgs.msg import Image
from sr_vision_msgs.srv import SegmentationControl
from sr_vision_msgs.msg import TrackBoxes


class Segmentation(object):
    def __init__(self):

        self.color = rospy.get_param('~color')
        self.shape = np.load(rospy.get_param('~shape'))
        self.shape_threshold = rospy.get_param('~shape_threshold')
        self.size = rospy.get_param('~size')

        self.utils = Utils(self.color)
        self.seg = ShapeColorSegmentation(self.color, self.shape,
                                          self.shape_threshold, self.size,
                                          self.utils)

        self.image_sub = rospy.Subscriber('camera/image_raw', Image,
                                          self.image_callback)

        self.segmentation_pub = rospy.Publisher("/roi/segmented_box",
                                                TrackBoxes, queue_size=1)

        self.server = rospy.Service('~start', SegmentationControl,
                                    self.segment)

    def image_callback(self, data):
        """
        Convert the ROS image to OpenCV format using a cv_bridge helper
        function and make a copy
        """
        self.frame = self.utils.convert_image(data, "bgr8")

    def segment(self, req):
        """
        Launch the segmentation
        @return - Success of the segmentation as a boolean
        """
        self.track_boxes = []
        try:
            self.seg.segmentation(self.frame)
            for i, seg in enumerate(self.seg.segmented_box):
                box = self.utils.roi_to_box(seg, num=i)
                self.track_boxes.append(box)

            nb_segments = len(self.track_boxes)

            if req.check_new:
                return nb_segments
            elif nb_segments > 0:
                self.segmentation_pub.publish(self.track_boxes)
                return nb_segments
            else:
                return 0

        except (IndexError, TypeError, AttributeError):
            return 0


def main():
    try:
        rospy.init_node('segmentation')
        Segmentation()
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down sr_object_segmentation node."


if __name__ == '__main__':
    main()
