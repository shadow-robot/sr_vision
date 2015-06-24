#!/usr/bin/env python
import sys
import rospy

from sr_object_tracking.camshift import CamshiftTracking


def main(args):
    try:
        rospy.init_node('sr_object_tracking')
        node = CamshiftTracking()
        while not rospy.is_shutdown():
            try:
                node.tracking()
            except:
                pass
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main(sys.argv)
