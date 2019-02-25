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
