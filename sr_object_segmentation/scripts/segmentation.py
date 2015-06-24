#!/usr/bin/env python

import rospy

from sr_object_segmentation.hsv_segmentation import HSVSegmentation
from sr_object_tracking.utils import Utils

from sensor_msgs.msg import Image, RegionOfInterest


class Segmentation(object):
    def __init__(self):
        rospy.init_node('sr_object_segmentation')
        self.color = rospy.get_param('/color')

        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.spin)
        self.selection_pub = rospy.Publisher("/roi/segmented_box", RegionOfInterest, queue_size=1)

        self.seg = HSVSegmentation(self.color)
        self.utils = Utils()

        self.frame = None

    def spin(self, data):
        """
        Convert the ROS image to OpenCV format using a cv_bridge helper function and make a copy
        """
        self.frame = self.utils.convert_image(data, "bgr8")

        try:
            self.seg.segmentation(self.frame)
            roi = self.utils.publish_box(self.seg.segmented_box)
            self.selection_pub.publish(roi)
        except rospy.ROSInterruptException:
            pass


def main():
    try:
        Segmentation()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down sr_object_segmentation node."


if __name__ == '__main__':
    main()
