#!/usr/bin/env python

# example use:
# rosrun sr_vision_mocks spawn_object.py -p "duplo_2x4x1" 0.7 0.6 0.763 0 0 1.57 -p "duplo_2x4x1" 0.7 0.7 0.763 0 0 0

import rospy
import argparse
import tf
import tf2_ros
import rospkg
import re
from gazebo_ros.gazebo_interface import spawn_sdf_model_client
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import TransformStamped, Pose
from sr_msgs_common.srv import MoveObject


class DebugFramePublisher:
    def __init__(self, description_path='sr_description_common', gazebo=False):
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.CONST_DESCRIPTION_PATH = rospkg.RosPack().get_path(description_path)
        self.moving_object_service = rospy.Service('/move_sim_object', MoveObject, self.move_object_CB)
        self.known_object_types = ["duplo_2x4x1", "utl5_small", "utl5_medium", "utl5_large"]
        self.spawned_objects = []
        self.gazebo = gazebo

    def process_args(self, poses_list):
        for pose in poses_list:
            idx = 0
            tf_name = pose.pop(0) + "_"
            for spawned_object in self.spawned_objects:
                if re.match(tf_name, spawned_object['name']):
                    idx += 1
            object_name = tf_name + str(idx)
            object_euler_pose = [float(val) for val in pose]
            object_transform = self.get_transform(object_name, object_euler_pose)
            self.spawned_objects.append({'name': object_name,
                                         'euler_pose': object_euler_pose,
                                         'transform': object_transform})

    def broadcast_frames(self):
        object_transforms = [spawned_object['transform'] for spawned_object in self.spawned_objects]
        self.broadcaster.sendTransform(object_transforms)

    def change_object_pose(self, moved_object_name, new_euler_pose):
        for spawned_object in self.spawned_objects:
            if moved_object_name == spawned_object['name']:
                spawned_object['euler_pose'] = new_euler_pose
                spawn_object['transform'] = self.get_transform(moved_object_name, new_euler_pose)
                break

    def change_model_pose_in_scene(self, object_name, new_pose):
        self.remove_model_from_scene(object_name)
        self.insert_model_in_scene(object_name, new_pose)

    def insert_all_models_in_scene(self):
        for spawned_object in self.spawned_objects:
            object_name = spawned_object['name']
            object_type = self.find_object_type(object_name)
            model_sdf_file = open(self.CONST_DESCRIPTION_PATH + '/models/{}/model.sdf'.format(object_type),
                        'r').read()
            object_pose = self.get_pose(spawned_object['euler_pose'])
            rospy.loginfo("Inserting {} at {}.".format(object_name, object_pose))
            spawn_sdf_model_client(object_name, model_sdf_file, "namespace", object_pose, 'world', "gazebo")

    def insert_model_in_scene(self, object_name, object_pose):
        object_type = self.find_object_type(object_name)
        model_sdf_file = open(self.CONST_DESCRIPTION_PATH + '/models/{}/model.sdf'.format(object_type),
                              'r').read()
        rospy.loginfo("Inserting {} at {}.".format(object_name, object_pose))
        spawn_sdf_model_client(object_name, model_sdf_file, "namespace", object_pose, 'world', "gazebo")

    def remove_all_models_from_scene(self):
        rospy.wait_for_service('gazebo/delete_model')
        for spawned_object in self.spawned_objects:
            try:
                print "*************8 here 3"
                delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
                print "**********8 model name: {}".format(spawned_object['name'])
                delete_model(spawned_object['name'])
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: {}".format(e))

    def remove_model_from_scene(self, object_name):
        rospy.wait_for_service('gazebo/delete_model')
        try:
            delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
            delete_model(object_name)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {}".format(e))

    def move_object_CB(self, req):
        rospy.loginfo("Moving object {} to position {}".format(req.object_id, req.place_pose))
        place_orientation = tf.transformations.euler_from_quaternion([req.place_pose.orientation.x,
                                                                      req.place_pose.orientation.y,
                                                                      req.place_pose.orientation.z,
                                                                      req.place_pose.orientation.w])
        place_pose_euler = [req.place_pose.position.x,
                            req.place_pose.position.y,
                            req.place_pose.position.z,
                            place_orientation[0],
                            place_orientation[1],
                            place_orientation[2]]

        self.change_object_pose(req.object_id, place_pose_euler)
        if self.gazebo:
            self.change_model_pose_in_scene(req.object_id, req.place_pose)
        self.broadcast_frames()
        return True

    def find_object_type(self, object_name):
        for known_object_type in self.known_object_types:
            if re.match(known_object_type, object_name):
                return known_object_type
        rospy.logwarn("Unknown object type!")
        return None

    @staticmethod
    def get_transform(name, euler_pose):
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = rospy.Time.now()

        static_transform_stamped.header.frame_id = "world"
        static_transform_stamped.child_frame_id = name

        static_transform_stamped.transform.translation.x = euler_pose[0]
        static_transform_stamped.transform.translation.y = euler_pose[1]
        static_transform_stamped.transform.translation.z = euler_pose[2]

        quat = tf.transformations.quaternion_from_euler(euler_pose[3], euler_pose[4], euler_pose[5])
        static_transform_stamped.transform.rotation.x = quat[0]
        static_transform_stamped.transform.rotation.y = quat[1]
        static_transform_stamped.transform.rotation.z = quat[2]
        static_transform_stamped.transform.rotation.w = quat[3]
        return static_transform_stamped

    @staticmethod
    def get_pose(euler_pose):
        pose = Pose()

        pose.position.x = euler_pose[0]
        pose.position.y = euler_pose[1]
        pose.position.z = euler_pose[2]

        quat = tf.transformations.quaternion_from_euler(euler_pose[3], euler_pose[4], euler_pose[5])
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

if __name__ == '__main__':
    rospy.init_node('spawn_object')

    parser = argparse.ArgumentParser()
    parser.add_argument('-g', '--gazebo', action='store_true', help='Use this flag if running in simulation')
    parser.add_argument('-p', '--pose', action='append', nargs=7, help='Block position, use as three ' +
                                                                       'floats separated by space sign only')
    parser.add_argument('-d', '--description', default='sr_description_common')
    args = parser.parse_args(rospy.myargv()[1:])

    frame_pub = DebugFramePublisher(args.description, args.gazebo)
    frame_pub.process_args(args.pose)
    frame_pub.broadcast_frames()

    if args.gazebo:
        frame_pub.insert_all_models_in_scene()
        rospy.on_shutdown(frame_pub.remove_all_models_from_scene)

    rospy.spin()
