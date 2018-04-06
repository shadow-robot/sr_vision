#!/usr/bin/env python
#
# Copyright (C) 2017 Shadow Robot Company Ltd - All Rights Reserved.
# Proprietary and Confidential. Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
import tf
import re
from sr_manipulation_msgs.msg import AvailableObjects, GraspObject
from geometry_msgs.msg import Pose
from threading import Lock


class MockKnownObjectsPublisher(object):
    def __init__(self):
        self.object_regex = rospy.get_param("object_regex", ".*duplo_2x4x1_")
        self.object_type = rospy.get_param("object_type", "duplo_2x4x1")
        self.publisher = rospy.Publisher("known_objects", AvailableObjects, queue_size=1, latch=True)
        self.tf_listener = tf.TransformListener()
        self._mutex = Lock()
        self.output_array = AvailableObjects()
        rospy.sleep(1)

    def update_known_objects(self):
        object_pose = Pose()
        duplo_frames_string_list = self.find_frame_names_in_list(self.tf_listener.getFrameStrings())
        for frame_name in duplo_frames_string_list:
            (trans, rot) = self.tf_listener.lookupTransform("world", frame_name, rospy.Time(0))
            this_object = GraspObject()
            this_object.id = frame_name
            this_object.type = self.object_type

            object_pose.position.x = trans[0]
            object_pose.position.y = trans[1]
            object_pose.position.z = trans[2]
            quat = tf.transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
            object_pose.orientation.x = quat[0]
            object_pose.orientation.y = quat[1]
            object_pose.orientation.z = quat[2]
            object_pose.orientation.w = quat[3]

            this_object.pose.pose = object_pose
            self.output_array.objects.append(this_object)

    def publish_known_objects(self):
        for object_tmp in self.output_array.objects:
            print object_tmp.id
        print "----------------"
        self.publisher.publish(self.output_array)
        rospy.sleep(1)

    def find_frame_names_in_list(self, list_to_filter):
        reg_exp = re.compile(self.object_regex)
        list_to_filter = filter(reg_exp.match, list_to_filter)
        return list_to_filter


if __name__ == "__main__":
    rospy.init_node("mock_known_objects_publisher")
    mock_publisher = MockKnownObjectsPublisher()
    mock_publisher.update_known_objects()
    while not rospy.is_shutdown():
        mock_publisher.publish_known_objects()