#!/usr/bin/python

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

import rospy
from object_manager_object import ObjectManagerObject
from object_recognition_msgs.msg import RecognizedObjectArray


class ObjectManager(object):
    def __init__(self):
        rospy.loginfo('Initialising ObjectManager instance:')
        self.get_params()
        rospy.loginfo('Recognized object topic:     {}'.format(self.input_recognized_objects_topic))
        self.object_manager_objects = []
        self.recognized_objects_subscriber = rospy.Subscriber(self.input_recognized_objects_topic,
                                                              RecognizedObjectArray, self.on_recognized_object_array)
        while not rospy.is_shutdown():
            rospy.spin()

    def get_params(self):
        self.averaging_window_width = rospy.get_param('~averaging_window_width')
        self.input_recognized_objects_topic = rospy.get_param('~input_recognized_objects_topic')
        self.output_recognized_objects_topic = rospy.get_param('~output_recognized_objects_topic')
        self.publish_rate = rospy.get_param('~publish_rate')

    def on_recognized_object_array(self, recognized_object_array):
        rospy.logdebug('Received recognized object array containing {} recognized objects.'
                       .format(len(recognized_object_array.objects)))
        self.object_manager_objects = ObjectManagerObject.init_from_recognized_object_array(recognized_object_array,
                                                                                            self.averaging_window_width)
