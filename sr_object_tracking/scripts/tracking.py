#!/usr/bin/env python

import rospy
import cv2

from sr_object_tracking.camshift import CamshiftTracking
from sr_vision_msgs.srv import SegmentationControl


def main():
    try:
        rospy.init_node('sr_object_tracking')
        rospy.wait_for_service('/segmentation/start')
        seg = rospy.ServiceProxy('/segmentation/start', SegmentationControl)

        seg_success = seg().success
        while not seg_success:
            seg_success = seg().success

        node = CamshiftTracking()
        while not rospy.is_shutdown():
            try:
                success_track = node.tracking()
                if not success_track:
                    seg()
            except (AttributeError, cv2.error, rospy.ROSInterruptException):
                pass
        rospy.spin()

    except (rospy.ROSInterruptException, rospy.ServiceException):
        pass


if __name__ == '__main__':
    main()
