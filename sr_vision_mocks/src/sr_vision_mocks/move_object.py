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

# Used together with spawn_object.py node
# Example use:
# rosrun sr_vision_mocks move_object.py -o duplo_2x4x1_0 -p 0.600 0.701 0.7633 0.0 0.0 1.57079632679

import rospy
import argparse
import tf
from sr_msgs_common.srv import MoveObject
from geometry_msgs.msg import Pose


def request_pick_and_place_sim(object_name, place_pose):
    pose = Pose()
    pose.position.x = place_pose[0]
    pose.position.y = place_pose[1]
    pose.position.z = place_pose[2]

    quat = tf.transformations.quaternion_from_euler(place_pose[3], place_pose[4], place_pose[5])
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    rospy.loginfo("Picking up object")
    rospy.wait_for_service('move_sim_object')
    try:
        move_sim_object = rospy.ServiceProxy('/move_sim_object', MoveObject)
        response = move_sim_object(object_name, pose)
        rospy.loginfo("Object placed in new position")
        return response
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed")

if __name__ == '__main__':
    rospy.init_node('mating_map_mock_client', anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument('-o', '--object_name', type=str, help='Name of the duplo block frame')
    parser.add_argument('-p', '--place_position', nargs=6, help='Pose to place object to')
    args = parser.parse_args()

    place_pose = [float(val) for val in args.place_position]
    chosen_object = args.object_name

    rospy.loginfo("Chose {} to be picked".format(chosen_object))
    rospy.loginfo("Moving duplo to position: {}".format(place_pose))

    pick_and_place_successful = request_pick_and_place_sim(chosen_object, place_pose)

    if pick_and_place_successful:
        rospy.loginfo("Successfully moved object".format(place_pose))
