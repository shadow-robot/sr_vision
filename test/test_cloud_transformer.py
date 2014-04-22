#!/usr/bin/env python

PKG='sr_point_cloud'

import os, unittest;
import rospy, actionlib;
from sr_grasp_msgs.srv import PCL_Transform
from sensor_msgs.msg import PointCloud2

class TestCloudTransformer(unittest.TestCase):
    def setUp(self):
        self.messages = []

    def lastMsg(self):
        return self.messages[-1]
    
    def assertIsPublishing(self, topic, topic_type, timeout = 3.0):
        """
        Test if a topic is publishing something of the correct type.
        The message receieved if this works is appended to the self.messages
        array.
        """
        is_publishing = False;
        err = None
        try:
            msg = rospy.wait_for_message(topic, topic_type, timeout = timeout)
            self.messages.append(msg)
        except Exception as err:
            pass
        else:
            is_publishing = True;
        self.assertTrue(is_publishing, topic + " is not publishing : " + str(err));
        return is_publishing
    
    def test_transform_cloud(self):
        """Test transforming a point cloud message from its original frame to world frame."""
        
        self.assertIsPublishing('/camera/depth_registered/points', PointCloud2 )
        self.assertGreater(len(self.lastMsg().data), 1000)
        
        service_available = True
        try:
            rospy.wait_for_service('/cloud_transformer/transform_cloud', 10.0)
        except rospy.ROSException:
            service_available = False
        
        self.assertTrue(service_available, "Server not found")
        
        transform_cloud = rospy.ServiceProxy('/cloud_transformer/transform_cloud', PCL_Transform)
        resp = None
        # Set the timestamp to now for the transform lookup to work
        self.lastMsg().header.stamp = rospy.Time.now()
        try:
            resp = transform_cloud(self.lastMsg(), "world")
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
        self.assertEqual(resp.point_cloud.header.frame_id, 'world')
        self.assertEqual(len(resp.point_cloud.data), len(self.lastMsg().data))

if __name__ == '__main__':
    import rostest
    rospy.init_node('test_cloud_transformer')
    rostest.rosrun('sr_point_cloud', 'test_cloud_transformer', TestCloudTransformer)


