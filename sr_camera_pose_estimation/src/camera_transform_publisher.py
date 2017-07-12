#! /usr/bin/python

import numpy as np
import roslaunch
import rospy
import tf2_ros
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Transform, TransformStamped
from tf import transformations
from pose_averager import PoseAverager


class CameraTransformPublisher(object):

    def __init__(self):
        self.get_params()
        self.transform_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.transform_buffer)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.alvar_process = None
        self.pose_averager = PoseAverager(window_width=self.window_width)
        rospy.loginfo("Starting camera transform publisher.")
        rospy.loginfo('Parameters:')
        rospy.loginfo('AR Marker Topic:     {}'.format(self.ar_marker_topic))
        rospy.loginfo('Camera Topic:        {}'.format(self.camera_image_topic))
        rospy.loginfo('Marker Transform:   {}'.format(self.marker_static_tf_name))
        rospy.loginfo('Marker ID:           {}'.format(self.marker_id))
        rospy.loginfo('Camera Root Frame:   {}'.format(self.camera_root_frame))
        self.counter = 0
        self.run()
        rospy.spin()

    def get_params(self):
        self.ar_marker_topic = rospy.get_param('~ar_marker_topic')
        self.camera_image_topic = rospy.get_param('~camera_image_topic')
        self.marker_static_tf_name = rospy.get_param('~marker_static_tf_name')
        self.marker_id = rospy.get_param('~marker_id')
        self.camera_root_frame = rospy.get_param('~camera_root_frame')
        self.desired_camera_parent_frame = rospy.get_param('~desired_camera_parent_frame')
        self.continuous = rospy.get_param('~continuous')
        self.window_width = rospy.get_param('~window_width')
        self.alvar_node_type = rospy.get_param('~alvar_node_type')
        self.alvar_marker_size = rospy.get_param('~alvar_marker_size')
        self.alvar_max_new_marker_error = rospy.get_param('~alvar_max_new_marker_error')
        self.alvar_max_track_error = rospy.get_param('~alvar_max_track_error')
        self.alvar_cam_image_topic = rospy.get_param('~alvar_cam_image_topic')
        self.alvar_cam_info_topic = rospy.get_param('~alvar_cam_info_topic')
        self.alvar_output_frame = rospy.get_param('~alvar_output_frame')
        self.alvar_med_filt_size = rospy.get_param('~alvar_med_filt_size')
        self.alvar_bundle_files = rospy.get_param('~alvar_bundle_files')

    def run(self):
        self.start_ar_track_alvar()
        rospy.loginfo('World Root Frame:    {}'.format(self.desired_camera_parent_frame))
        rospy.loginfo('Waiting for AR marker pose topic...')
        rospy.wait_for_message(self.ar_marker_topic, AlvarMarkers)
        rospy.loginfo('AR marker pose topic found!')
        while not rospy.is_shutdown() and (self.continuous or self.counter < self.window_width):
            ar_track_alvar_markers = rospy.wait_for_message(self.ar_marker_topic, AlvarMarkers,
                                                            timeout=1)
            self.on_ar_marker_message(ar_track_alvar_markers)
        self.stop_ar_track_alvar()
        self.counter = 0

    def on_ar_marker_message(self, ar_track_alvar_markers):
        for marker in ar_track_alvar_markers.markers:
            if marker.id == self.marker_id:
                self.on_new_pose(marker.pose.pose)
            else:
                rospy.loginfo('Ignoring pose for marked id {}'.format(marker.id))

    def on_new_pose(self, pose):
        self.counter += 1
        self.publish_camera_pose(self.pose_averager.new_value(pose))

    def publish_camera_pose(self, lens_marker_pose):
        try:
            world_marker_trans = self.transform_buffer. \
                                 lookup_transform(self.desired_camera_parent_frame,
                                                  self.marker_static_tf_name, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to find world -> AR marker ({} -> {})transform."
                         .format(self.desired_camera_parent_frame, self.marker_static_tf_name))
            return
        self.transform = self.generate_world_camera_tf(lens_marker_pose, world_marker_trans)
        self.publish_transform()

    def generate_world_camera_tf(self, lens_marker_pose, world_marker_tf):
        lens_marker_matrix = matrix_from_pose(lens_marker_pose)
        world_marker_matrix = matrix_from_transform(world_marker_tf.transform)
        world_lens_matrix = np.dot(world_marker_matrix,
                                   transformations.inverse_matrix(lens_marker_matrix))
        return transform_from_matrix(world_lens_matrix)

    def publish_transform(self):
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.get_rostime()
        transform_stamped.transform = self.transform
        transform_stamped.header.frame_id = self.desired_camera_parent_frame
        transform_stamped.child_frame_id = self.camera_root_frame
        self.broadcaster.sendTransform(transform_stamped)

    def start_ar_track_alvar(self):
        if self.alvar_process is None:
            package = 'ar_track_alvar'
            executable = self.alvar_node_type
            args = '{} {} {} {} {} {}{} {}'.format(self.alvar_marker_size,
                                                   self.alvar_max_new_marker_error,
                                                   self.alvar_max_track_error,
                                                   self.alvar_cam_image_topic,
                                                   self.alvar_cam_info_topic,
                                                   self.alvar_output_frame,
                                                   ' {}'.format(self.alvar_med_filt_size) if
                                                   (executable == 'findMarkerBundles') else '',
                                                   self.alvar_bundle_files)
            node = roslaunch.core.Node(package, executable, args=args)
            launch = roslaunch.scriptapi.ROSLaunch()
            launch.start()
            self.alvar_process = launch.launch(node)

    def stop_ar_track_alvar(self):
        if self.alvar_process is not None:
            self.alvar_process.stop()


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

if __name__ == "__main__":
    rospy.init_node("sat_camera_transform_publisher")
    CameraTransformPublisher()
