#!/usr/bin/env python
"""
See README.md
"""

from unittest import TestCase
import rospy
import math
from sr_extrinsic_calibration.camera_calibration import TransformationManager


class BaseCalibrationTest(TestCase):
    @staticmethod
    def get_package_name():
        return "sr_extrinsic_calibration"

    def _get_test_namespace(self):
        """
        This method was introduced in order to support debugging. In debug
         approach which I am currently using
        test is run separately from test file so namespace is not passed
        """
        result = rospy.get_namespace()
        if not result or "/" == result:
            result = "/camera_calibration/"

        return result

    def _read_parameters(self):
        """
        Reads parameters from camera calibration node because test has the
         same namespace as node name
        """
        self._marker_holder_frames = rospy.get_param(
            self._get_test_namespace() + "marker_holder_frames")
        self._camera_markers_ids = rospy.get_param(
            self._get_test_namespace() + "markers_ids")

        self._base_frame = rospy.get_param(
            self._get_test_namespace() + "base_frame")
        self._camera_frame = rospy.get_param(
            self._get_test_namespace() + "camera_frame")

    def _initialize_transformation_manager(self):
        # TODO Find out how to not use it and have access to ROS time
        rospy.init_node("test_node_for_calibration", anonymous=True)
        self._transformations_manager = TransformationManager()

    def _get_recent_transformation(self, target_frame, source_frame):
        return self._transformations_manager.get_recent_transformation(
            target_frame, source_frame)

    def _get_recent_camera_transformation(self, source_frame):
        return self._transformations_manager.get_recent_transformation(
            self._camera_frame, source_frame)

    def assertVector3Close(self, vector1, vector2, delta=0.00001, msg=None):
        result = math.sqrt(math.pow(vector1.x - vector2.x, 2) + math.pow(
            vector1.y - vector2.y, 2) + math.pow(vector1.z - vector2.z, 2))
        self.assertLess(result, delta, msg)

    def setUp(self):
        self._initialize_transformation_manager()
        self._read_parameters()

    def run_simple_test(self):

        rospy.sleep(rospy.Duration(2))

        for robot_frame, marker_frame in zip(self._marker_holder_frames,
                                             self._camera_markers_ids):
            robot_transformation = self._get_recent_camera_transformation(
                robot_frame)
            real_marker_transformation = \
                self._get_recent_camera_transformation(marker_frame)
            self.assertVector3Close(
                robot_transformation.transform.translation,
                real_marker_transformation.transform.translation, 0.01,
                "Marker and real robot marker holder are not close enough"
                " after transformation:\n" +
                str(robot_transformation.transform.translation) + "\nand\n" +
                str(real_marker_transformation.transform.translation))
