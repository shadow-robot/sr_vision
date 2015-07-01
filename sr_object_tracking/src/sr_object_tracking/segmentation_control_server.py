#!/usr/bin/env python

from sr_vision_msgs.srv import SegmentationControl
import rospy


def segmentation_control(req):
    """
    Return the booleen controlling the segmentation.
    """
    return req.stop_seg


def seg_control_server():
    rospy.init_node('segmentation_controller')
    rospy.Service('segmentation_controller', SegmentationControl, segmentation_control)
    rospy.spin()


if __name__ == "__main__":
    seg_control_server()
