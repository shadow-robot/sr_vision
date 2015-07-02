#!/usr/bin/env python
"""
See README.md
"""
from threading import Thread
import threading
from geometry_msgs.msg import TransformStamped
import rospy
from rospy.exceptions import ROSException
import tf2_ros

from base_calibration_test import BaseCalibrationTest


class BroadcastThread(Thread):

    def __init__(self, shift, camera_frame):
        Thread.__init__(self)
        rospy.init_node("test_node_for_calibration", anonymous=True)
        self._shift = shift
        self._camera_frame = camera_frame

    def set_shift(self, shift):
        self._shift = shift

    def _create_transformation(self, source_frame_index, x, y):
        transformation = TransformStamped()
        transformation.header.stamp = rospy.Time.now() + rospy.Duration(1.0)
        transformation.header.frame_id = self._camera_frame
        transformation.child_frame_id = "ar_marker_" + str(source_frame_index)
        transformation.transform.translation.x = x
        transformation.transform.translation.y = y
        transformation.transform.translation.z = self._shift
        transformation.transform.rotation.x = 0.0
        transformation.transform.rotation.y = 0.0
        transformation.transform.rotation.z = 0.0
        transformation.transform.rotation.w = 1.0
        return transformation

    def run(self):

        tf2_broadcaster = tf2_ros.TransformBroadcaster()

        try:
            while not rospy.is_shutdown():
                tf2_broadcaster.sendTransform(self._create_transformation(1, 1.0, 1.0))
                tf2_broadcaster.sendTransform(self._create_transformation(2, -1.0, 1.0))
                tf2_broadcaster.sendTransform(self._create_transformation(3, -1.0, -1.0))
                tf2_broadcaster.sendTransform(self._create_transformation(4, 1.0, -1.0))
        except ROSException:
            pass


class TestContinuousCalibration(BaseCalibrationTest):

    def test_calibration(self):

        new_shift_delta = 10
        self.run_simple_test()
        transformation_before = self._get_recent_transformation(self._base_frame, self._camera_frame)

        broadcast_thread.set_shift(initial_shift + new_shift_delta)
        self.run_simple_test()
        transformation_after = self._get_recent_transformation(self._base_frame, self._camera_frame)

        self.assertAlmostEqual(transformation_before.transform.translation.z - new_shift_delta,
                               transformation_after.transform.translation.z, msg="Shift delta should be as defined")

if __name__ == '__main__':
    import rostest
    initial_shift = 2
    broadcast_thread = BroadcastThread(initial_shift, "camera")
    broadcast_thread.start()
    rostest.rosrun(BaseCalibrationTest.get_package_name(), 'test_continuous_calibration', TestContinuousCalibration)
