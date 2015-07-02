#!/usr/bin/env python

PKG = 'sr_point_cloud'
NAME = 'test_cloud_tracker'

import os
import unittest
import rospy
import rostest
import rospkg
import rosbag
import actionlib
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from sr_point_cloud.msg import *


class TestCloudTracker(unittest.TestCase):
    # Util methods
    ###############

    def setUp(self):
        self.messages = []

    def lastMsg(self):
        return self.messages[-1]

    def assertIsPublishing(self, topic, topic_type, timeout=3.0):
        """
        Test if a topic is publishing something of the correct type.
        The message recieved if this works is appended to the self.messages
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
                        topic + " is not publishing : " + str(err)
        return is_publishing

    def assertIsNotPublishing(self, topic, topic_type, timeout=3.0):
        """Test if a topic is not publishing something of the correct type."""
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

    def bag_read_all(self, f):
        """Read all messages from the given bag file. Path is relative to
        test dir."""
        rp = rospkg.RosPack()
        bag_file = os.path.join(rp.get_path(PKG), 'test', f)
        bag = rosbag.Bag(bag_file)
        msgs = []
        for topic, msg, t in bag.read_messages():
            msgs.append(msg)
        return msgs

    # Tests
    ########

    def test_basic(self):
        """At startup we should see a target cloud but no result yet."""
        self.assertIsPublishing(
            '/point_cloud_tracker/cloud_downsampled/points', PointCloud2)
        self.assertGreater(len(self.lastMsg().data), 1000)
        self.assertIsNotPublishing('/point_cloud_tracker/result/points',
                                   PointCloud2)
        self.assertIsNotPublishing('/point_cloud_tracker/result/pose',
                                   PoseStamped)

    def test_track_action(self):
        """Test starting tracker on centered object."""
        goal = TrackGoal()
        goal.target = "centered"
        self._test_track_action(goal)

    def test_track_action_cloud(self):
        """Test starting tracker from cloud."""
        # track_me.bag contains a single Pointcloud2 message.
        # Can be created with something like:
        # $ rosbag record -l1 -Otrack_me /point_cloud_tracker/result/points
        # When the tracker s tracking an object.
        goal = TrackGoal()
        goal.cloud = self.bag_read_all('track_me.bag')[0]
        self._test_track_action(goal)

    def _test_track_action(self, goal):
        """
        Send a track goal, test we are tracking something, then stop the
        goal and test we stop tracking things.
        """
        # Send goal to the action server
        client = actionlib.SimpleActionClient('point_cloud_tracker/track',
                                              TrackAction)
        client.wait_for_server()
        client.send_goal(goal)
        rospy.sleep(1)

        # Check that tracking has started
        self.assertIsPublishing('/point_cloud_tracker/result/points',
                                PointCloud2)
        self.assertGreater(len(self.lastMsg().data), 100)

        # Check the pose isn't just an empty default pose
        self.assertIsPublishing('/point_cloud_tracker/result/pose',
                                PoseStamped)
        pose = self.lastMsg().pose
        self.assertNotEqual(pose.position.x, 0.0)
        self.assertNotEqual(pose.position.y, 0.0)
        self.assertNotEqual(pose.position.z, 0.0)
        self.assertNotEqual(pose.orientation.x, 0.0)
        self.assertNotEqual(pose.orientation.y, 0.0)
        self.assertNotEqual(pose.orientation.z, 0.0)
        self.assertNotEqual(pose.orientation.w, 0.0)

        # Stop the action, check we stop track
        client.cancel_all_goals()
        rospy.sleep(1)
        self.assertIsNotPublishing('/point_cloud_tracker/result/points',
                                   PointCloud2)
        self.assertIsNotPublishing('/point_cloud_tracker/result/pose',
                                   PoseStamped)


if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestCloudTracker)
