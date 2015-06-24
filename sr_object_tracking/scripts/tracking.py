#!/usr/bin/env python

import rospy
import cv2

from sr_object_tracking.camshift import CamshiftTracking


def main():
    try:
        rospy.init_node('sr_object_tracking')
        node = CamshiftTracking()
        while not rospy.is_shutdown():
            try:
                node.tracking()
            except (AttributeError, cv2.error, rospy.ROSInterruptException) :
                pass
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
