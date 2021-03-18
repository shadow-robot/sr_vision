#! /usr/bin/python

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

import numpy as np
import roslaunch
import rospy
import sr_vision_common
import tf2_ros
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose, Transform, TransformStamped
from tf import transformations
from sr_vision_common.pose_averager import PoseAverager


class CameraTransformPublisher(object):
    def __init__(self):
        self.get_params()
        self.transform_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.transform_buffer)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.pose_averager = PoseAverager(window_width=self.window_width)
        self.ignore_first = self.window_width * 2
        rospy.loginfo("Starting camera transform publisher.")
        rospy.loginfo('Parameters:')
        self.counter = 0
        self.broadcast_root_to_camera()
        while not rospy.is_shutdown():
            rospy.spin()

    def get_params(self):
        self.marker_frame_name = rospy.get_param('~marker_frame_name')
        self.root_frame_name = rospy.get_param('~root_frame_name')
        self.camera_frame_name = rospy.get_param('~camera_frame_name')
        self.marker_to_root_cartesian = rospy.get_param('~marker_to_root_cartesian')
        self.continuous = rospy.get_param('~continuous')
        self.window_width = rospy.get_param('~window_width')
        self.filtering = rospy.get_param('~filtering')
        self.static_launch_output = rospy.get_param("~output_static", "")

    def broadcast_root_to_camera(self):
        while not rospy.is_shutdown() and (self.continuous or self.counter < (self.window_width + self.ignore_first)):
            if not self.continuous:
                self.counter += 1
            marker_to_root_pose = cartesian_to_pose(self.marker_to_root_cartesian)
            root_to_camera_transform = self.get_root_to_camera_transform(marker_to_root_pose)
            if root_to_camera_transform is not None:
                root_to_camera_transform_stamped = transform_to_transform_stamped(root_to_camera_transform,
                                                                                  self.root_frame_name,
                                                                                  self.camera_frame_name)
                if self.static_launch_output != "":
                    launch_file_from_tf(root_to_camera_transform_stamped, self.static_launch_output)
                    self.static_launch_output = ""

                self.broadcaster.sendTransform(root_to_camera_transform_stamped)

    def get_root_to_camera_transform(self, marker_to_root_pose):
        try:
            camera_to_marker_transform = self.transform_buffer.lookup_transform(self.camera_frame_name,
                                                                                self.marker_frame_name,
                                                                                rospy.Time())
            camera_to_marker_pose = transform_to_pose(camera_to_marker_transform.transform)

            if self.filtering:
                camera_to_marker_pose = self.filter_pose(camera_to_marker_pose)

            camera_to_marker_matrix = matrix_from_pose(camera_to_marker_pose)
            marker_to_marker_root_matrix = matrix_from_pose(marker_to_root_pose)
            camera_to_root_matrix = np.dot(camera_to_marker_matrix, marker_to_marker_root_matrix)
            root_to_camera_matrix = transformations.inverse_matrix(camera_to_root_matrix)
            return transform_from_matrix(root_to_camera_matrix)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to find world -> AR marker ({} -> {})transform."
                          .format(self.camera_frame_name, self.marker_frame_name))
            rospy.sleep(1.0)
            return None

    def filter_pose(self, pose):
        if self.window_width > 1:
            return self.pose_averager.new_value(pose)
        else:
            return pose


def cartesian_to_pose(coordinates):
    pose = Pose()
    pose.position.x = coordinates[0]
    pose.position.y = coordinates[1]
    pose.position.z = coordinates[2]
    quat = transformations.quaternion_from_euler(coordinates[3], coordinates[4], coordinates[5])
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose


def matrix_from_transform(transform):
    trans = [transform.translation.x, transform.translation.y, transform.translation.z]
    rot = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
    return matrix_from_trans_rot(trans, rot)


def matrix_from_pose(pose):
    trans = [pose.position.x, pose.position.y, pose.position.z]
    rot = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    return matrix_from_trans_rot(trans, rot)


def matrix_from_trans_rot(trans, rot):
    trans_mat = transformations.translation_matrix(trans)
    rot_mat = transformations.quaternion_matrix(rot)
    return transformations.concatenate_matrices(trans_mat, rot_mat)


def transform_from_matrix(matrix):
    trans = transformations.translation_from_matrix(matrix)
    rot = transformations.quaternion_from_matrix(matrix)
    transform = Transform()
    transform.translation.x = trans[0]
    transform.translation.y = trans[1]
    transform.translation.z = trans[2]
    transform.rotation.x = rot[0]
    transform.rotation.y = rot[1]
    transform.rotation.z = rot[2]
    transform.rotation.w = rot[3]
    return transform


def transform_to_transform_stamped(trans, name, child_name):
    trans_stamped = TransformStamped()
    trans_stamped.header.stamp = rospy.Time.now()
    trans_stamped.header.frame_id = name
    trans_stamped.child_frame_id = child_name
    trans_stamped.transform = trans
    return trans_stamped


def transform_to_pose(trans):
    pose = Pose()
    pose.position.x = trans.translation.x
    pose.position.y = trans.translation.y
    pose.position.z = trans.translation.z
    pose.orientation.x = trans.rotation.x
    pose.orientation.y = trans.rotation.y
    pose.orientation.z = trans.rotation.z
    pose.orientation.w = trans.rotation.w
    return pose


def launch_file_from_tf(tf, output_file):
    text = ("<launch>\n"
            "  <node pkg='tf' name='camera_calibration_publisher'\n"
            "        type='static_transform_publisher'\n"
            "        args='%f %f %f %f %f %f %f %s %s 100'/>\n"
            "</launch>\n") % (
                tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z,
                tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w,
                tf.header.frame_id, tf.child_frame_id)
    with open(output_file, "w") as output:
        output.write(text)
    rospy.loginfo("Wrote static tf to %s" % output_file)

if __name__ == "__main__":
    rospy.init_node("sat_camera_transform_publisher")
    CameraTransformPublisher()
