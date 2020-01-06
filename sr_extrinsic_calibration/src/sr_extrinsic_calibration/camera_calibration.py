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

"""
See README.md
"""
import numpy
import math

import rospy
from tf.transformations import superimposition_matrix, \
    quaternion_from_matrix, translation_from_matrix, quaternion_multiply, \
    quaternion_from_euler, vector_norm, euler_from_quaternion
from geometry_msgs.msg import TransformStamped
from sr_extrinsic_calibration.transformation_manager import \
    TransformationManager


class CameraCalibration(TransformationManager):
    _camera_pose = TransformStamped()

    _acceptable_distance_error = 0.05
    _distance_error_tolerance = 0.1
    _angle_error_tolerance = 0.02

    def __init__(self):
        """
        Object for camera position calculation and correction
        It continuously publishes transformation from base frame to camera
         frame
        """
        super(CameraCalibration, self).__init__()
        self._load_parameters()

    def _load_parameters(self):
        """
        Loads parameters for calibration node
        """
        camera_pose = rospy.get_param("~initial_tf",
                                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        if len(camera_pose) < 6:
            raise Exception(
                "Parameter initial_tf should contain 6 items [x, y, z and 3 "
                "Euler angles]")

        self._marker_holder_frames = rospy.get_param("~marker_holder_frames")
        self._camera_markers_ids = rospy.get_param("~markers_ids")

        if len(self._marker_holder_frames) != len(self._camera_markers_ids):
            raise Exception(
                "Amount of marker_holder_frames and markers_ids should be "
                "equal")

        if not len(self._marker_holder_frames) in [3, 4]:
            raise Exception("marker_holder_frames should have 3 or 4 elements")

        self._base_frame = rospy.get_param("~base_frame")
        self._camera_frame = rospy.get_param("~camera_frame")

        self._camera_pose.header.frame_id = self._base_frame
        self._camera_pose.child_frame_id = self._camera_frame
        self._camera_pose.transform.translation.x = camera_pose[0]
        self._camera_pose.transform.translation.y = camera_pose[1]
        self._camera_pose.transform.translation.z = camera_pose[2]
        q = quaternion_from_euler(camera_pose[3], camera_pose[4],
                                  camera_pose[5])
        self._camera_pose.transform.rotation.x = q[0]
        self._camera_pose.transform.rotation.y = q[1]
        self._camera_pose.transform.rotation.z = q[2]
        self._camera_pose.transform.rotation.w = q[3]

    def _is_distance_between_frames_similar(self, index1, index2):
        """
        Check if distance between frames similar

        @param index1 index of the first value
        @param index2 index of the second value
        @return boolean
        """
        transformation1 = self.get_recent_transformation(
            self._marker_holder_frames[index1],
            self._marker_holder_frames[index2])
        transformation2 = self.get_recent_filtered_transformation(
            self._camera_markers_ids[index1],
            self._camera_markers_ids[index2])

        vector1 = TransformationManager.point_from_tf(transformation1)
        vector2 = TransformationManager.point_from_tf(transformation2)

        norm1 = vector_norm(vector1)
        norm2 = vector_norm(vector2)

        if math.fabs(norm1 - norm2) > self._acceptable_distance_error:
            rospy.logerr("Distance between " + self._camera_markers_ids[
                index1] + " and " + self._camera_markers_ids[
                index2] + " is not matching appropriate robot "
                          "marker distances")
            rospy.logerr(
                "It is " + str(norm2) + " and should be " + str(norm1))
            return False

        return True

    def _is_robot_camera_frames_of_the_same_figure(self):
        """
        Check if distance between all frames in the robot and markers equal

        @return boolean
        """
        result = True

        if 3 <= len(self._marker_holder_frames):
            result = (self._is_distance_between_frames_similar(0, 1) and
                      self._is_distance_between_frames_similar(0, 2) and
                      self._is_distance_between_frames_similar(1, 2))

        if 4 <= len(self._marker_holder_frames):
            result = (result and
                      self._is_distance_between_frames_similar(0, 3) and
                      self._is_distance_between_frames_similar(1, 3))
        return result

    def _adjust_camera_position(self):
        """
        Recalculate position of the camera to match robot frames with markers
        """
        if not self._is_robot_camera_frames_of_the_same_figure():
            return

        positions = []
        for i in range(len(self._camera_markers_ids)):
            transformation = self.get_recent_transformation(
                self._camera_frame, self._camera_markers_ids[i])
            point = TransformationManager.point_from_tf(transformation)
            positions.append(point)

        markers_positions = numpy.array(positions)

        positions = []
        for i in range(len(self._marker_holder_frames)):
            transformation = self.get_recent_filtered_transformation(
                self._camera_frame, self._marker_holder_frames[i])
            point = TransformationManager.point_from_tf(transformation)
            positions.append(point)

        robot_marker_holders_positions = numpy.array(positions)

        if not numpy.allclose(robot_marker_holders_positions,
                              markers_positions,
                              atol=self._distance_error_tolerance):

            affine_transformation_matrix = superimposition_matrix(
                markers_positions.T, robot_marker_holders_positions.T)

            translation = translation_from_matrix(affine_transformation_matrix)

            needed_rotation = quaternion_from_matrix(
                affine_transformation_matrix)

            euler_angles = euler_from_quaternion(needed_rotation)

            apply_translation = not numpy.allclose(
                translation, numpy.zeros(3),
                atol=self._distance_error_tolerance)
            apply_rotation = not numpy.allclose(
                euler_angles, numpy.zeros(3), atol=self._angle_error_tolerance)

            if apply_translation or apply_rotation:
                rospy.loginfo("Applying camera correction")

            if apply_translation:
                rospy.loginfo("  Translation => " + str(translation))

                self._camera_pose.transform.translation.x += translation[0]
                self._camera_pose.transform.translation.y += translation[1]
                self._camera_pose.transform.translation.z += translation[2]

            if apply_rotation:
                rospy.loginfo("  Rotation => " + str(needed_rotation))

                current_rotation = [self._camera_pose.transform.rotation.x,
                                    self._camera_pose.transform.rotation.y,
                                    self._camera_pose.transform.rotation.z,
                                    self._camera_pose.transform.rotation.w]

                new_rotation = quaternion_multiply(current_rotation,
                                                   needed_rotation)

                self._camera_pose.transform.rotation.x = new_rotation[0]
                self._camera_pose.transform.rotation.y = new_rotation[1]
                self._camera_pose.transform.rotation.z = new_rotation[2]
                self._camera_pose.transform.rotation.w = new_rotation[3]

    def _spin_once(self):
        """
        Publish camera pose
        """
        self._camera_pose.header.stamp = rospy.Time.now()
        self.send_transformation(self._camera_pose)

    def start(self):
        """
        Start node with camera position recalculation and publishing
        """
        update_rate = rospy.get_param("~update_rate", 20.0)
        rate = rospy.Rate(update_rate)
        cycles_count = int(update_rate)

        self._spin_once()

        while not rospy.is_shutdown():
            self._adjust_camera_position()
            i = 0
            while not rospy.is_shutdown() and i < cycles_count:
                self._spin_once()
                rate.sleep()
                i += 1

    def run_once(self):
        """
        Run publishing and camera position recalculation only once
        """
        self._spin_once()
        self._adjust_camera_position()

    def print_camera_tf(self):
        """
        Print camera position in form of static transformation node code from
         launch file
        """
        translation = self._camera_pose.transform.translation
        quaternion = self._camera_pose.transform.rotation

        rospy.loginfo("Camera transformation static node code")
        rospy.loginfo(
            '<node pkg="tf" type="static_transform_publisher" '
            'name="base_to_camera_publisher" ' +
            'args="%f %f %f %f %f %f %f /%s /%s 100" />', translation.x,
            translation.y, translation.z,
            quaternion.x, quaternion.y, quaternion.z, quaternion.w,
            self._base_frame, self._camera_frame)
