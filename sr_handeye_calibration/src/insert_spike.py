#!/usr/bin/env python

# This code is used for checking precision of extrinsic callibration

import moveit_python
import moveit_commander
import rospy
from geometry_msgs.msg import *

if __name__ == "__main__":
    rospy.init_node('insert_spike')
    robot = moveit_commander.RobotCommander()
    scene = moveit_python.PlanningSceneInterface('ra_base')

    scene.addCylinder("spike1", 0.306, 0.0063, -0.25, -0.68235, 0.153)