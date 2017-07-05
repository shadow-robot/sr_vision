#! /usr/bin/python
import argparse
import rospy
import tf2_ros
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Transform, TransformStamped
from tf import transformations, TransformerROS


class CameraTransformPublisher:
    # Default values - defined here to achieve common defaults between main and imported function
    default_ar_marker_topic = 'ar_pose_marker'
    default_camera_image_topic = '/camera/rgb/image_raw'
    default_marker_static_tf_names = ['ar_marker_0_static']
    default_marker_ids = ['0']
    default_camera_root_frame = 'camera_link'
    default_desired_camera_parent_frame = 'ra_base_link'

    def __init__(self, ar_marker_topic=default_ar_marker_topic, camera_image_topic=default_camera_image_topic,
                 marker_static_tf_names=default_marker_static_tf_names, marker_ids=default_marker_ids,
                 camera_root_frame=default_camera_root_frame,
                 desired_camera_parent_frame=default_desired_camera_parent_frame):
        if not ar_marker_topic:
            ar_marker_topic = rospy.get_param('~ar_marker_topic', CameraTransformPublisher.default_ar_marker_topic)
        if not camera_image_topic:
            camera_image_topic = rospy.get_param('~camera_image_topic',
                                                 CameraTransformPublisher.default_camera_image_topic)
        if not marker_static_tf_names:
            marker_static_tf_names = rospy.get_param('~marker_static_tf_names',
                                                     CameraTransformPublisher.default_marker_static_tf_names)
        if not marker_ids:
            marker_ids = rospy.get_param('~marker_ids', CameraTransformPublisher.default_marker_ids)
        if not camera_root_frame:
            camera_root_frame = rospy.get_param('~camera_root_frame',
                                                CameraTransformPublisher.default_camera_root_frame)
        if not desired_camera_parent_frame:
            desired_camera_parent_frame = rospy.get_param('~desired_camera_parent_frame',
                                                          CameraTransformPublisher.default_desired_camera_parent_frame)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.ar_marker_topic = ar_marker_topic
        self.camera_image_topic = camera_image_topic
        self.marker_static_tf_names = marker_static_tf_names
        self.marker_ids = marker_ids
        self.camera_root_frame = camera_root_frame
        self.desired_camera_parent_frame = desired_camera_parent_frame
        self.run()

    def run(self):
        rospy.loginfo("Starting camera transform publisher.")
        rospy.loginfo('Parameters:')
        rospy.loginfo('AR Marker Topic:     {}'.format(self.ar_marker_topic))
        rospy.loginfo('Camera Topic:        {}'.format(self.camera_image_topic))
        rospy.loginfo('Marker Transforms:   {}'.format(self.marker_static_tf_names))
        rospy.loginfo('Marker IDs:          {}'.format(self.marker_ids))
        rospy.loginfo('Camera Root Frame:   {}'.format(self.camera_root_frame))
        rospy.loginfo('World Root Frame:    {}'.format(self.desired_camera_parent_frame))
        rospy.loginfo('Waiting for AR marker pose topic...')
        rospy.wait_for_message(self.ar_marker_topic, AlvarMarkers)
        rospy.loginfo('AR marker pose topic found!')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                ar_track_alvar_markers = rospy.wait_for_message(self.ar_marker_topic, AlvarMarkers, timeout=1)
                self.on_ar_marker_message(ar_track_alvar_markers)
            except rospy.ROSException, e:
                pass
            rate.sleep()

    def on_ar_marker_message(self, ar_track_alvar_markers):
        for marker in ar_track_alvar_markers.markers:
            if marker.id in self.marker_ids:
                static_marker_frame_index = self.marker_ids.index(marker.id)
                static_marker_frame_name = self.marker_static_tf_names[static_marker_frame_index]
                try:
                    lens_marker_trans = self.tfBuffer.lookup_transform(self.camera_root_frame,
                                                                       'ar_marker_{}'.format(marker.id), rospy.Time())
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr("Failed to find lens -> AR marker transform.")
                    return
                try:
                    world_marker_trans = self.tfBuffer.lookup_transform(self.desired_camera_parent_frame,
                                                                        static_marker_frame_name, rospy.Time())
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr("Failed to find world -> AR marker ({} -> {})transform.".format(
                        self.desired_camera_parent_frame, static_marker_frame_name))
                    return
                transform = self.generate_world_camera_tf(lens_marker_trans, world_marker_trans)
                self.publish_transform(transform)
            else:
                rospy.loginfo('Ignoring pose for marked id {}'.format(marker.id))

    def generate_world_camera_tf(self, lens_marker_tf, world_marker_tf):
        lens_marker_matrix = matrix_from_transform(lens_marker_tf.transform)
        world_marker_matrix = matrix_from_transform(world_marker_tf.transform)
        world_lens_matrix = np.dot(world_marker_matrix, transformations.inverse_matrix(lens_marker_matrix))
        return transform_from_matrix(world_lens_matrix)

    def publish_transform(self, transform):
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.get_rostime()
        transform_stamped.transform = transform
        transform_stamped.header.frame_id = self.desired_camera_parent_frame
        transform_stamped.child_frame_id = self.camera_root_frame
        self.broadcaster.sendTransform(transform_stamped)


def matrix_from_transform(transform):
    trans = [transform.translation.x, transform.translation.y, transform.translation.z]
    rot = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
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
    parser = argparse.ArgumentParser()
    parser.add_argument('--ar_marker_topic', dest='ar_marker_topic', required=False, default=None)
    parser.add_argument('--camera_image_topic', dest='camera_image_topic', required=False, default=None)
    parser.add_argument('--marker_static_tf_names', dest='marker_static_tf_names', nargs='*', required=False,
                        default=None)
    parser.add_argument('--marker_ids', dest='marker_ids', nargs='*', type=int, required=False, default=None)
    parser.add_argument('--camera_root_frame', dest='camera_root_frame', required=False, default=None)
    parser.add_argument('--desired_camera_parent_frame', dest='desired_camera_parent_frame', required=False,
                        default=None)
    args, unknown = parser.parse_known_args()
    rospy.init_node("sat_camera_transform_publisher")
    camera_transform_publisher = CameraTransformPublisher(ar_marker_topic=args.ar_marker_topic,
                                                          camera_image_topic=args.camera_image_topic,
                                                          marker_static_tf_names=args.marker_static_tf_names,
                                                          marker_ids=args.marker_ids,
                                                          camera_root_frame=args.camera_root_frame,
                                                          desired_camera_parent_frame=args.desired_camera_parent_frame)
