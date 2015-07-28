#!/usr/bin/env python

import rospy
import cv2

from sr_object_tracking.sequential_tracking import SequentialTracking
from sr_vision_msgs.srv import SegmentationControl


class Tracking(object):
    """
    sr_vision base class. Coordinates tracking and segmentation launches.
    """

    def __init__(self):
        """
        Initializes the tracking node and launches the initial segmentation.
        """
        self.node = SequentialTracking()

        rospy.wait_for_service('segmentation/start')
        self.seg = rospy.ServiceProxy('segmentation/start',
                                      SegmentationControl)

        seg_success = self.seg(False).nb_segments > 0
        while not seg_success:
            seg_success = self.seg(False).nb_segments > 0

        self.run()

    def run(self):
        """
        Tracks the segmented objects, segments again if they're lost and checks
         if new ones appear.
        """
        while not rospy.is_shutdown():
            if self.node.frame is not None:
                try:
                    success_track = self.node.run()

                    if not success_track:
                        seg_success = self.seg(False).nb_segments > 0
                        while not seg_success:
                            seg_success = self.seg(False).nb_segments > 0

                    new_segments = self.seg(True).nb_segments > len(
                        self.node.seg_boxes)
                    if new_segments:
                        self.seg(False)

                except (
                        AttributeError, cv2.cv.error,
                        rospy.ROSInterruptException):
                    pass

        rospy.spin()


def main():
    try:
        rospy.init_node('tracking')
        Tracking()
    except rospy.ROSInterruptException:
        print "Shutting down sr_object_tracking node."


if __name__ == '__main__':
    main()
