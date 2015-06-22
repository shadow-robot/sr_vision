#!/usr/bin/env python
import sys
import rospy

from sr_object_tracking.camshift import CamshiftTracking
from sr_object_segmentation.hsv_segmentation import HSVSegmentation


def main(args):
    try:
        rospy.init_node('sr_object_tracking')
        node = CamshiftTracking()
        seg = HSVSegmentation()

        while not rospy.is_shutdown():
            while node.frame is not None:
                try:
                    if seg.points == None:
                        node.tracking_state = 0
                        segmented_box = seg.segmentation(node.frame)
                    else:
                        if segmented_box is not None:
                            node.tracking_state = 1
                            segmented_box = None
                    node.tracking(node.frame, segmented_box)
                except:
                    pass

        rospy.spin()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main(sys.argv)
