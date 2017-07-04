#!/usr/bin/env python
import unittest
import rospy
import actionlib
from object_recognition_msgs.msg import *

PKG = 'sr_point_cloud'


class TestClusterSegmentor(unittest.TestCase):
    def test_recognise_action(self):
        """Test starting tracker on centered object."""
        # Send goal to the action server
        timeout = rospy.Duration.from_sec(2000.0) # Set from 300.0
        client = actionlib.SimpleActionClient(
            'cluster_segmentor/recognize_objects', ObjectRecognitionAction)
        goal = ObjectRecognitionGoal()
        self.assertTrue(client.wait_for_server(timeout),
                        "Action server not found")
        client.send_goal(goal)
        self.assertTrue(client.wait_for_result(timeout),
                        "Action did not return within the timelimit")
        res = client.get_result()

        num_cluster = len(res.recognized_objects.objects)
        self.assertGreater(num_cluster, 9,
                           "Didn't get enough clusters: %s" % num_cluster)

        for obj in res.recognized_objects.objects:
            # Should have the same frame as the camera
            self.assertEqual(obj.header.frame_id, '/camera_rgb_optical_frame')
            # Should have had a position set, ie not the default pose.
            self.assertEqual(obj.pose.header.frame_id,
                             '/camera_rgb_optical_frame')
            self.assertNotEqual(obj.pose.pose.pose.position.x, 0.0)
            self.assertNotEqual(obj.pose.pose.pose.position.y, 0.0)
            self.assertNotEqual(obj.pose.pose.pose.position.z, 0.0)
            # Check the clouds have some data in them
            self.assertEqual(len(obj.point_clouds), 1)
            self.assertGreater(len(obj.point_clouds[0].data), 1000)
            # Cloud should have the same frame as the camera
            self.assertEqual(obj.point_clouds[0].header.frame_id,
                             '/camera_rgb_optical_frame')


if __name__ == '__main__':
    import rostest

    rospy.init_node('test_cluster_segmentor')
    rostest.rosrun('sr_point_cloud', 'test_cluster_segmentor',
                   TestClusterSegmentor)
