#! /usr/bin/python

import numpy as np
import roslaunch
import rospy
import tf2_ros
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose, Transform, TransformStamped
from tf import transformations
from pose_averager import PoseAverager


class CameraTransformPublisher(object):

    def __init__(self):
        self.get_params()
        self.transform_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.transform_buffer)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        #self.pose_averager = PoseAverager(window_width=self.window_width)
        #self.ignore_first = 20
        #self.counter = 0
        self.broadcast_root_to_camera()
        self.run()
        while not rospy.is_shutdown():
            rospy.spin()

    def get_params(self):
        self.marker_frame_name = rospy.get_param('~marker_frame_name')
        self.marker_root_frame_name = rospy.get_param('~marker_root_frame_name')
        self.camera_frame_name = rospy.get_param('~camera_frame_name')
        self.marker_to_root_pose = rospy.get_param('~marker_to_root_pose')
        #self.continuous = rospy.get_param('~continuous')
        #self.window_width = rospy.get_param('~window_width')
        
    def broadcast_root_to_camera(self):        
        marker_to_marker_root_pose = self.list_to_pose(self.marker_to_root_pose) #
        root_to_camera_transform = self.root_to_camera_transform(marker_to_marker_root_pose)
        root_to_camera_transform_stamped = transform_to_transform_stamped(root_to_camera_transform, self.marker_root_frame_name, self.camera_frame_name)
        self.broadcaster.sendTransform(root_to_camera_transform_stamped)
    
    def root_to_camera_transform(self, marker_to_marker_root_pose):
        waiting_for_transform = True
        while waiting_for_transform:
            try:
                camera_to_marker_transform = self.transform_buffer.lookup_transform(self.camera_frame_name, self.marker_frame_name, rospy.Time())
                waiting_for_transform = False
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("Failed to find world -> AR marker ({} -> {})transform."
                             .format(self.camera_frame_name, self.marker_frame_name))
                rospy.sleep(1.0)
                continue

        camera_to_marker_matrix = matrix_from_transform(camera_to_marker_transform.transform)
        marker_to_marker_root_matrix = matrix_from_pose(marker_to_marker_root_pose)
        camera_to_root_matrix = np.dot(camera_to_marker_matrix, marker_to_marker_root_matrix)
        root_to_camera_matrix = transformations.inverse_matrix(camera_to_root_matrix)
        return transform_from_matrix(root_to_camera_matrix)
    
    @staticmethod
    def list_to_pose(l):
        pose = Pose()

        pose.position.x = l[0]
        pose.position.y = l[1]
        pose.position.z = l[2]

        quat = transformations.quaternion_from_euler(l[3], l[4], l[5])
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

    #def on_ar_marker_message(self, ar_track_alvar_markers):
        #for marker in ar_track_alvar_markers.markers:
            #if marker.id == self.marker_id:
                #self.on_new_pose(marker.pose.pose)
            #else:
                #rospy.loginfo('Ignoring pose for marked id {}'.format(marker.id))

    #def on_new_pose(self, pose):
        #self.counter += 1
        #if self.window_width > 1:
            #self.publish_camera_pose(self.pose_averager.new_value(pose))
        #else:
            #self.publish_camera_pose(pose)

    #def publish_camera_pose(self, lens_marker_pose):
        #try:
            #world_marker_trans = self.transform_buffer. \
                                 #lookup_transform(self.desired_camera_parent_frame,
                                                  #self.marker_static_tf_name, rospy.Time())
        #except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                #tf2_ros.ExtrapolationException):
            #rospy.logerr("Failed to find world -> AR marker ({} -> {})transform."
                         #.format(self.desired_camera_parent_frame, self.marker_static_tf_name))
            #return
        #self.transform = self.generate_world_camera_tf(lens_marker_pose, world_marker_trans)
        #self.publish_transform()

    #def generate_world_camera_tf(self, lens_marker_pose, world_marker_tf):
        #lens_marker_matrix = matrix_from_pose(lens_marker_pose)
        #world_marker_matrix = matrix_from_transform(world_marker_tf.transform)
        #world_lens_matrix = np.dot(world_marker_matrix,
                                   #transformations.inverse_matrix(lens_marker_matrix))
        #return transform_from_matrix(world_lens_matrix)

    #def publish_transform(self):
        #transform_stamped = TransformStamped()
        #transform_stamped.header.stamp = rospy.get_rostime()
        #transform_stamped.transform = self.transform
        #transform_stamped.header.frame_id = self.desired_camera_parent_frame
        #transform_stamped.child_frame_id = self.camera_root_frame
        #self.broadcaster.sendTransform(transform_stamped)


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

if __name__ == "__main__":
    rospy.init_node("sat_camera_transform_publisher")
    CameraTransformPublisher()
