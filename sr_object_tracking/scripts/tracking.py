#!/usr/bin/env python

# Copyright 2019 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

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
