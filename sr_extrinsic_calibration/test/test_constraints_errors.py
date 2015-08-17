#!/usr/bin/env python
"""
See README.md
"""
from unittest import TestCase
from sr_extrinsic_calibration.camera_calibration import CameraCalibration
from geometry_msgs.msg import TransformStamped
import rospy
import tf2_ros

PKG = "sr_extrinsic_calibration"


class SimpleMockMetaClass(type):
    """
    Metaclass of mock which contains any attribute. Just to stub any class not
    needed for error checking.
    """

    def __getattr__(cls, name):
        def function(*args):
            return object()

        return function


class SimpleMock(object):
    """
    Mock class which contains any attribute. Just to stub any class not needed
    for error checking.
    """
    __metaclass__ = SimpleMockMetaClass

    def __init__(self, *args):
        pass

    def __getattr__(self, name):
        def function(*args):
            return object()

        return function


class BufferMock(object):
    """
    Mock for tf buffer to put correct translations
    """

    @staticmethod
    def _is_case_for_robot_frame(target_frame, source_frame, index1, index2):
        frames = ["marker_holder_" + str(index1),
                  "marker_holder_" + str(index2)]

        if target_frame in frames and source_frame in frames:
            return True
        return False

    @staticmethod
    def _is_case_for_marker_frame(target_frame, source_frame, index1, index2):
        frames = ["ar_marker_" + str(index1), "ar_marker_" + str(index2)]

        if target_frame in frames and source_frame in frames:
            return True
        return False

    @staticmethod
    def _get_transformation(x, y, z):
        transformation = TransformStamped()
        transformation.transform.translation.x = x
        transformation.transform.translation.y = y
        transformation.transform.translation.z = z
        return transformation

    def lookup_transform(self, target_frame, source_frame, time):
        if self._is_case_for_robot_frame(target_frame, source_frame, 1, 2):
            return self._get_transformation(1, 1, 1)
        if self._is_case_for_robot_frame(target_frame, source_frame, 1, 3):
            return self._get_transformation(1, 2, 1)
        if self._is_case_for_robot_frame(target_frame, source_frame, 2, 3):
            return self._get_transformation(1, 1, 1)

        if self._is_case_for_marker_frame(target_frame, source_frame, 1, 2):
            return self._get_transformation(1, 1, 1)
        if self._is_case_for_marker_frame(target_frame, source_frame, 1, 3):
            return self._get_transformation(1, 1, 1)
        if self._is_case_for_marker_frame(target_frame, source_frame, 2, 3):
            return self._get_transformation(1, 1, 1)


class TestConstraintsErrors(TestCase):
    def setUp(self):
        self._backup_get_param_back = rospy.get_param
        self._backup_Time = rospy.Time
        self._backup_Rate = rospy.Rate
        self._backup_shutdown = rospy.is_shutdown
        self._backup_Buffer = tf2_ros.Buffer
        self._backup_TransformListener = tf2_ros.TransformListener
        self._backup_TransformBroadcaster = tf2_ros.TransformBroadcaster
        self._backup_error_log = rospy.logerr

    def tearDown(self):
        rospy.get_param = self._backup_get_param_back
        tf2_ros.Buffer = self._backup_Buffer
        tf2_ros.TransformListener = self._backup_TransformListener
        tf2_ros.TransformBroadcaster = self._backup_TransformBroadcaster
        rospy.Time = self._backup_Time
        rospy.Rate = self._backup_Rate
        rospy.is_shutdown = self._backup_shutdown
        rospy.logerr = self._backup_error_log

    @staticmethod
    def _params_three_frames_mock(param_name, default=None):
        return {
            "~camera_frame": "camera",
            "~base_frame": "delta_base",
            "~marker_holder_frames": ["marker_holder_1", "marker_holder_2",
                                      "marker_holder_3"],
            "~markers_ids": ["ar_marker_1", "ar_marker_2", "ar_marker_3"],
        }.get(param_name, default)

    @staticmethod
    def _params_four_frames_mock(param_name, default=None):
        return {
            "~camera_frame": "camera",
            "~base_frame": "delta_base",
            "~marker_holder_frames": [
                "marker_holder_1", "marker_holder_2",
                "marker_holder_3", "marker_holder_4"],
            "~markers_ids": [
                "ar_marker_1", "ar_marker_2", "ar_marker_3", "ar_marker_4"],
        }.get(param_name, default)

    @staticmethod
    def _params_different_frames_lengths_mock(param_name, default=None):
        return {
            "~camera_frame": "camera",
            "~base_frame": "delta_base",
            "~marker_holder_frames": [
                "marker_holder_1", "marker_holder_2", "marker_holder_3"],
            "~markers_ids": [
                "ar_marker_1", "ar_marker_2", "ar_marker_3", "ar_marker_4"],
        }.get(param_name, default)

    @staticmethod
    def _params_not_3_or_4_parameters_mock(param_name, default=None):
        return {
            "~camera_frame": "camera",
            "~base_frame": "delta_base",
            "~marker_holder_frames": ["marker_holder_1"],
            "~markers_ids": ["ar_marker_1"],
        }.get(param_name, default)

    def test_different_frames_count(self):

        tf2_ros.TransformListener = SimpleMock
        tf2_ros.TransformBroadcaster = SimpleMock
        tf2_ros.Buffer = BufferMock

        rospy.get_param = \
            TestConstraintsErrors._params_different_frames_lengths_mock
        try:
            CameraCalibration()
        except Exception as e:
            self.assertEqual(
                "Amount of marker_holder_frames and markers_ids "
                "should be equal",
                e.message,
                "Incorrect validation exception message: " + e.message)

        rospy.get_param = \
            TestConstraintsErrors._params_not_3_or_4_parameters_mock
        try:
            CameraCalibration()
        except Exception as e:
            self.assertEqual("marker_holder_frames should have "
                             "3 or 4 elements",
                             e.message,
                             "Incorrect validation exception message " +
                             e.message)

    @staticmethod
    def _return_false():
        return False

    @staticmethod
    def _check_error_log_for_distance_error(message):
        if "Distance between ar_marker_1 and ar_marker_3 is not matching " \
           "appropriate robot marker distances" == message:
            raise Exception(message)

    def test_different_distances_between_frames(self):
        rospy.get_param = TestConstraintsErrors._params_three_frames_mock
        tf2_ros.TransformListener = SimpleMock
        tf2_ros.TransformBroadcaster = SimpleMock
        rospy.Time = SimpleMock
        rospy.Rate = SimpleMock
        rospy.is_shutdown = TestConstraintsErrors._return_false
        tf2_ros.Buffer = BufferMock
        rospy.logerr = \
            TestConstraintsErrors._check_error_log_for_distance_error

        try:
            camera_calibration = CameraCalibration()
            camera_calibration.start()
        except Exception as e:
            self.assertEqual(
                "Distance between ar_marker_1 and ar_marker_3 is not matching "
                "appropriate robot marker distances",
                e.message, "Incorrect validation exception message")


if __name__ == '__main__':
    import rosunit

    rosunit.rosrun(PKG, 'test_constraints_errors', TestConstraintsErrors)
