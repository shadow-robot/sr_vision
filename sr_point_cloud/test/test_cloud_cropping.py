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

import unittest
import rospy
import rostest
from sensor_msgs.msg import PointCloud2, RegionOfInterest

PKG = 'sr_point_cloud'
NAME = 'test_cloud_tracker'


class TestCloudCropping(unittest.TestCase):
    # Util methods
    ###############

    def setUp(self):
        self.messages = []

    def lastMsg(self):
        return self.messages[-1]

    def assertIsPublishing(self, topic, topic_type, timeout=3.0):
        """
        Test if a topic is publishing something of the correct type.
        The message received if this works is appended to the self.messages
        array.
        """
        is_publishing = False
        err = None
        try:
            msg = rospy.wait_for_message(topic, topic_type, timeout=timeout)
            self.messages.append(msg)
        except Exception as err:
            pass
        else:
            is_publishing = True
        self.assertTrue(is_publishing,
                        topic + " is not publishing : " + str(err))
        return is_publishing

    def assertIsNotPublishing(self, topic, topic_type, timeout=3.0):
        """
        Test if a topic is not publishing something of the correct type.
        """
        is_publishing = False
        try:
            rospy.wait_for_message(topic, topic_type, timeout=timeout)
        except rospy.ROSException as err:
            if str(err).startswith("timeout exceeded"):
                is_publishing = False
            else:
                raise
        except:
            raise
        else:
            is_publishing = True
        self.assertFalse(is_publishing, topic + " is publishing")
        return is_publishing

    # Tests
    ########

    def test_basic(self):
        """
        Test if the camera's running and the simulated track_box bag published
        # track_box.bag contains a single RegionOfInterest message.
        # Can be created with something like:
        # $ rosbag record -l1 -o track_box /roi/track_box
        # When a region of interest had been initialized.
        """
        self.assertIsPublishing('/camera/depth_registered/points', PointCloud2)
        self.assertIsPublishing('/roi/track_box', RegionOfInterest)

    def test_crop_publish_cloud(self):
        """
        Test starting cropping from cloud.
        """
        self.assertIsPublishing('/roi/track_cloud', PointCloud2)
        self.assertNotEqual(len(self.lastMsg().data), 0)

    def test_crop_good_cloud(self):
        """
        Test the dimensions of the published cloud.
        """
        self.assertIsPublishing('/roi/track_cloud', PointCloud2)

        box = rospy.wait_for_message('/roi/track_box', RegionOfInterest,
                                     timeout=4)
        box_size = box.width * box.height
        box_centroid = (box.x_offset + box.width / 2,
                        box.y_offset + box.y_offset / 2)

        cloud = self.lastMsg()
        cloud_size = cloud.width

        self.assertAlmostEqual(box_size, cloud_size, delta=5)


if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestCloudCropping)
