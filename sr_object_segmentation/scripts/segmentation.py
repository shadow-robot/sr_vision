#!/usr/bin/env python

import rospy
import numpy as np

from sr_object_segmentation.shape_color_segmentation import \
    ShapeColorSegmentation
from sr_object_tracking.utils import Utils

from sensor_msgs.msg import Image, RegionOfInterest
from geometry_msgs.msg import Pose
from sr_vision_msgs.srv import SegmentationControl


class Segmentation(object):
    def __init__(self):

        self.color = rospy.get_param('~color')
        self.shape = np.load(rospy.get_param('~shape'))
        self.size = rospy.get_param('~size')

        self.seg = ShapeColorSegmentation(self.color, self.shape, self.size)
        self.utils = Utils()

        self.image_sub = rospy.Subscriber('camera/image_raw', Image,
                                          self.image_callback)

        self.pose_pub = rospy.Publisher("roi/pose",
                                        Pose, queue_size=1)

        self.selection_pub = rospy.Publisher("roi/segmented_box",
                                             RegionOfInterest, queue_size=1)

        self.server = rospy.Service('~start', SegmentationControl,
                                    self.segment)

    def image_callback(self, data):
        """
        Convert the ROS image to OpenCV format using a cv_bridge helper
        function and make a copy
        """
        self.frame = self.utils.convert_image(data, "bgr8")

    def segment(self, _):
        """
        Launch the segmentation
        @return - Success of the segmentation as a boolean
        """

        try:
            self.seg.segmentation(self.frame)
            for i, seg in enumerate(self.seg.segmented_box):
                roi = self.utils.box_to_roi(seg)
                self.selection_pub.publish(roi)

                pose = self.utils.publish_pose(seg, moment=self.seg.poses[i])
                self.pose_pub.publish(pose)
            return True
        except (IndexError, TypeError, AttributeError):
            return False


def main():
    try:
        rospy.init_node('segmentation')
        Segmentation()
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down sr_object_segmentation node."


if __name__ == '__main__':
    main()
