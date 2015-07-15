#!/usr/bin/env python

import rospy

from sr_object_segmentation.hsv_segmentation import HSVSegmentation
from sr_object_tracking.utils import Utils

from sensor_msgs.msg import Image, RegionOfInterest
from geometry_msgs.msg import PoseStamped
from sr_vision_msgs.srv import SegmentationControl


class Segmentation(object):
    def __init__(self):

        self.color = rospy.get_param('~color')

        self.seg = HSVSegmentation(self.color)
        self.utils = Utils()

        self.image_sub = rospy.Subscriber('camera/rgb/image_color', Image,
                                          self.image_callback)

        self.pose_pub = rospy.Publisher("roi/pose",
                                        PoseStamped, queue_size=1)

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

            roi = self.utils.publish_box(self.seg.segmented_box)
            pose = self.utils.publish_pose(moment=self.seg.poses)

            self.selection_pub.publish(roi)
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
