#!/usr/bin/env python

import rospy
import tf
import re
from sr_msgs_common.msg import SrRecognizedObject, SrRecognizedObjectArray
from geometry_msgs.msg import Pose
from threading import Lock


class MockRecognizedObjectsPublisher(object):
    def __init__(self):
        self.publisher = rospy.Publisher("recognized_objects", SrRecognizedObjectArray, queue_size=1, latch=True)
        self.regex_to_type = {"^utl5_small_": "utl5_small",
                              "^duplo_2x4x1_": "duplo_2x4x1"}
        self.recognized_object_array = SrRecognizedObjectArray()
        self.update_recognized_objects_array()

    def update_recognized_objects_array(self):
        tf_listener = tf.TransformListener()
        rospy.sleep(1) # waiting for tf listener to setup
        all_frames_string_list = tf_listener.getFrameStrings()
        self.recognized_object_array.objects = self.create_recognized_objects_array(all_frames_string_list)

    def create_recognized_objects_array(self, list_to_filter):
        recognized_objects_list = []
        for key, value in self.regex_to_type.iteritems():
            reg_exp = re.compile(key)
            for recognized_object_string in filter(reg_exp.match, list_to_filter):
                recognized_object = SrRecognizedObject()
                recognized_object.type = value
                recognized_object.frame_name = recognized_object_string
                recognized_object.instance = self.get_trailing_number(recognized_object_string)
                recognized_objects_list.append(recognized_object)
        return recognized_objects_list
    
    def get_trailing_number(self, string):
        m = re.search(r'\d+$', string)
        return int(m.group()) if m else None

    def publish_recognized_objects(self):
        for recognized_object in self.recognized_object_array.objects:
            rospy.loginfo(recognized_object.frame_name)
        rospy.loginfo("-----------------------")
        self.publisher.publish(self.recognized_object_array)

if __name__ == "__main__":
    rospy.init_node("mock_recognized_objects_publisher")
    mock_publisher = MockRecognizedObjectsPublisher()
    while not rospy.is_shutdown():
        mock_publisher.update_recognized_objects_array()
        mock_publisher.publish_recognized_objects()
