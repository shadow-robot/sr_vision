#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('camera_rgb_optical_frame', 'ar_marker_0', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print "yo"
            rate.sleep()
            continue

        print trans
        rate.sleep()