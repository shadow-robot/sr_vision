#!/usr/bin/python

import rospy
import sr_object_manager
from object_manager import ObjectManager


if __name__ == '__main__':
    rospy.init_node('sr_object_manager')
    object_manager = ObjectManager()
