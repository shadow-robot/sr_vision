#!/usr/bin/env python
import unittest
import rospy
from sr_grasp_msgs.srv import PclTransform, PclFilter
from sensor_msgs.msg import PointCloud2

PKG = 'sr_point_cloud'

class TestCloudServices(unittest.TestCase):
    def setUp(self):
        self.messages = []

    def lastMsg(self):
        return self.messages[-1]

    def assertIsPublishing(self, topic, topic_type, timeout=3.0):
        """
        Test if a topic is publishing something of the correct type.
        The message receieved if this works is appended to the self.messages
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

    def test_transform_cloud(self):
        """Test transforming a point cloud message from its original frame to
        world frame."""

        self.assertIsPublishing('/camera/depth_registered/points', PointCloud2)
        self.assertGreater(len(self.lastMsg().data), 1000)

        service_available = True
        try:
            rospy.wait_for_service('/cloud_services/transform_cloud', 10.0)
        except rospy.ROSException:
            service_available = False

        self.assertTrue(service_available, "Server not found")

        transform_cloud = rospy.ServiceProxy('/cloud_services/transform_cloud',
                                             PclTransform)
        resp = None
        # Set the timestamp to now for the transform lookup to work
        self.lastMsg().header.stamp = rospy.Time.now()
        try:
            resp = transform_cloud(self.lastMsg(), "world")
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        self.assertEqual(resp.point_cloud.header.frame_id, 'world')
        self.assertEqual(len(resp.point_cloud.data), len(self.lastMsg().data))

    def test_filter_cloud(self):
        """Test filtering a point cloud message using VoxelGrid."""

        self.assertIsPublishing('/camera/depth_registered/points', PointCloud2)
        self.assertGreater(len(self.lastMsg().data), 1000)

        service_available = True
        try:
            rospy.wait_for_service('/cloud_services/filter_cloud', 10.0)
        except rospy.ROSException:
            service_available = False

        self.assertTrue(service_available, "Server not found")

        filter_cloud = rospy.ServiceProxy('/cloud_services/filter_cloud',
                                          PclFilter)
        resp = None
        downsampling_grid_size = 0.01

        try:
            resp = filter_cloud(self.lastMsg(), downsampling_grid_size)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        # Check that the frame_id is still the same
        self.assertEqual(resp.point_cloud.header.frame_id,
                         self.lastMsg().header.frame_id)
        # Check that the number of points has been reduced
        self.assertGreater(len(self.lastMsg().data),
                           len(resp.point_cloud.data))


if __name__ == '__main__':
    import rostest

    rospy.init_node('test_cloud_services')
    rostest.rosrun('sr_point_cloud', 'test_cloud_services', TestCloudServices)
