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

#   This node listens to Gazebo's published model states, and publishes a TF
#   for each object, as well as a RecognizedObjectArray. It is intended to
#   mock a perfect vision solution, in order to test grasping etc.

import rospy
import threading
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from object_recognition_msgs.msg import RecognizedObjectArray, RecognizedObject


class MockRecognizeGazeboObjects(object):
    def __init__(self):
        self.recognized_object_array = RecognizedObjectArray()
        self.tfs = []
        self.lock = threading.Lock()
        self.br = tf2_ros.TransformBroadcaster()
        self.r = rospy.Rate(10)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.on_model_states)
        self.publisher = rospy.Publisher('/recognized_objects', RecognizedObjectArray,
                                         queue_size=1)
        while not rospy.is_shutdown():
            self.r.sleep()
            self.publish()

    def publish(self):
        self.lock.acquire()
        for tf in self.tfs:
            self.br.sendTransform(tf)
        self.publisher.publish(self.recognized_object_array)
        self.lock.release()

    def on_model_states(self, model_states):
        object_counts = {}
        tfs = []
        recognized_object_array = RecognizedObjectArray()
        recognized_object_array.header.frame_id = 'world'
        recognized_object_array.header.stamp = rospy.Time.now()
        for i, model_name in enumerate(model_states.name):
            if model_name in object_counts:
                object_counts[model_name] = object_counts[model_name] + 1
            else:
                object_counts[model_name] = 0
            tf_name = '{}_{}'.format(model_name, object_counts[model_name])
            recognized_object = RecognizedObject()
            recognized_object.header.frame_id = tf_name
            recognized_object.header.stamp = rospy.Time.now()
            recognized_object.type.key = model_name
            recognized_object.pose.pose.pose = model_states.pose[i]
            recognized_object_array.objects.append(recognized_object)
            tf = TransformStamped()
            tf.child_frame_id = tf_name
            tf.header.frame_id = 'world'
            tf.header.stamp = rospy.Time.now()
            tf.transform.translation.x = model_states.pose[i].position.x
            tf.transform.translation.y = model_states.pose[i].position.y
            tf.transform.translation.z = model_states.pose[i].position.z
            tf.transform.rotation = model_states.pose[i].orientation
            tfs.append(tf)
        self.lock.acquire()
        self.recognized_object_array = recognized_object_array
        self.tfs = tfs
        self.lock.release()

if __name__ == '__main__':
    rospy.init_node('mock_recognize_gazebo_objects')
    MockRecognizeGazeboObjects()
    rospy.spin()
