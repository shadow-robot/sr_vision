#!/usr/bin/env python

from sr_vision_msgs.srv import SegmentationControl
import rospy


def segmentation_control(req):
    """
    Control the segmentation process
    """
    self.seg.segmentation(self.frame)
    roi = self.utils.publish_box(self.seg.segmented_box)
    self.selection_pub.publish(roi)
    self.seg_control(True)


def seg_control_server():
    rospy.init_node('~start')
    rospy.Service('~start', SegmentationControl, segmentation_control)
    rospy.spin()


if __name__ == "__main__":
    seg_control_server()
