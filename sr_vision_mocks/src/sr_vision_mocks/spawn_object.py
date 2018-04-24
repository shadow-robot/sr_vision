#!/usr/bin/env python
#
# Copyright (C) 2017 Shadow Robot Company Ltd - All Rights Reserved. Proprietary and Confidential.
# Unauthorized copying of the content in this file, via any medium is strictly prohibited.

# example use:
# rosrun sr_vision_mocks spawn_object.py -p "duplo_2x4x1" 0.7 0.6 0.763 0 0 1.57 -p "utl5_small" 0.7 0.7 0.763 0 0 0

import rospy
import argparse
import tf
import tf2_ros
import rospkg
import re
from geometry_msgs.msg import TransformStamped, Pose
from sr_msgs_common.srv import MoveObject


class DebugFramePublisher:
    def __init__(self, sim=False):
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.moving_object_service = rospy.Service('/move_sim_object', MoveObject, self.move_object_CB)
        self.tf_object_names = []
        self.tf_object_poses = []
        self.tf_object_transforms = []
        self.sim = sim

    def process_args(self, poses_list): 
        for pose in poses_list:
            idx = 0
            tf_name = pose.pop(0) + "_"
            for object_name in self.tf_object_names:
                if re.match(tf_name, object_name):
                    idx += 1
            self.tf_object_names.append(tf_name + str(idx))
            self.tf_object_poses.append([float(val) for val in pose])

        for name, pose in zip(self.tf_object_names, self.tf_object_poses):
            self.tf_object_transforms.append(self.get_transform(name, pose[0:3], pose[3:6]))

    def broadcast_frames(self):
        self.broadcaster.sendTransform(self.tf_object_transforms)

    def change_duplo_position(self, moved_object_name, new_pose):
        for i, object_name in enumerate(self.tf_object_names):
            if moved_object_name == object_name:
                self.tf_object_poses[i] = new_pose
                self.tf_object_transforms[i] = self.get_transform(moved_object_name,
                                                                  self.tf_object_poses[i][0:3],
                                                                  self.tf_object_poses[i][3:6])
                break

    def move_object_CB(self, req):
        rospy.loginfo("Moving object {} to position {}".format(req.object_id, req.place_pose))
        place_orientation = tf.transformations.euler_from_quaternion([req.place_pose.orientation.x,
                                                                      req.place_pose.orientation.y,
                                                                      req.place_pose.orientation.z,
                                                                      req.place_pose.orientation.w])

        self.change_duplo_position(req.object_id, [req.place_pose.position.x,
                                                   req.place_pose.position.y,
                                                   req.place_pose.position.z,
                                                   place_orientation[0],
                                                   place_orientation[1],
                                                   place_orientation[2]])
        self.broadcast_frames()
        return True

    @staticmethod
    def get_transform(name, position, rotation):
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = rospy.Time.now()

        static_transform_stamped.header.frame_id = "world"
        static_transform_stamped.child_frame_id = name

        static_transform_stamped.transform.translation.x = position[0]
        static_transform_stamped.transform.translation.y = position[1]
        static_transform_stamped.transform.translation.z = position[2]

        quat = tf.transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])
        static_transform_stamped.transform.rotation.x = quat[0]
        static_transform_stamped.transform.rotation.y = quat[1]
        static_transform_stamped.transform.rotation.z = quat[2]
        static_transform_stamped.transform.rotation.w = quat[3]
        return static_transform_stamped

    @staticmethod
    def get_pose(position, rotation):
        pose = Pose()

        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]

        quat = tf.transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

if __name__ == '__main__':
    rospy.init_node('my_static_tf2_broadcaster')

    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--sim', action='store_true', help='Use this flag if running in simulation')
    parser.add_argument('-p', '--pose', action='append', nargs=7, help='Block position, use as three ' +
                                                                       'floats separated by space sign only')
    args = parser.parse_args()

    frame_pub = DebugFramePublisher(args.sim)
    frame_pub.process_args(args.pose)
    frame_pub.broadcast_frames()

    rospy.spin()
