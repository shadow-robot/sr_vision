#!/usr/bin/env python

PKG='sr_point_cloud'

import os, unittest;
import rospy, actionlib;
from object_recognition_msgs.msg import *

class TestClusterSegmentor(unittest.TestCase):

    def test_recognise_action(self):
        """Test starting tracker on centered object."""
        # Send goal to the action server
        timeout = rospy.Duration.from_sec(20.0)
        client = actionlib.SimpleActionClient(
                'cluster_segmentor/recognize_objects', ObjectRecognitionAction)
        goal = ObjectRecognitionGoal()
        self.assertTrue(client.wait_for_server(timeout), "Action server not found")
        client.send_goal(goal)
        self.assertTrue(client.wait_for_result(timeout),
                "Action did not return within the timelimit")
        res = client.get_result()

        num_cluster = len(res.recognized_objects.objects)
        self.assertGreater(num_cluster, 9, "Didn't get enough clusters: %s"%num_cluster)

        for obj in res.recognized_objects.objects:
            # Should have the same farme as the camera
            self.assertEqual(obj.header.frame_id, '/camera_rgb_optical_frame')
            # Check the clouds have some data in them
            self.assertEqual(len(obj.point_clouds), 1)
            self.assertGreater(len(obj.point_clouds[0].data), 1000)
            # Should have the same farme as the camera
            self.assertEqual(obj.point_clouds[0].header.frame_id, '/camera_rgb_optical_frame')

if __name__ == '__main__':
    import rostest
    rospy.init_node('test_cluster_segmentor')
    rostest.rosrun('sr_point_cloud', 'test_cluster_segmentor', TestClusterSegmentor)


