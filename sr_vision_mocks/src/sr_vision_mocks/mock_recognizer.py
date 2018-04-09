#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from object_recognition_msgs.msg import RecognizedObject, RecognizedObjectArray
from threading import Thread, Lock


class MockRecognizer(object):
    def __init__(self, object_ids=None, object_xyzrpys=None, rate=10.0, input_frame='world',
                 db='{"collection":"object_recognition","root":"http://localhost:5984","type":"CouchDB"}',
                 output_frame='camera_depth_optical_frame'):
        self.mutex = Lock()
        self.rate = rospy.Rate(rate)
        self.input_frame = input_frame
        self.db = db
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.output_frame = output_frame
        self.set_objects(object_ids=object_ids, object_xyzrpys=object_xyzrpys)
        self.stopping = False

    def run(self):
        rospy.loginfo('Mock recognizer running.')
        self.publisher = rospy.Publisher('recognized_objects', RecognizedObjectArray, queue_size=10)
        while not (rospy.is_shutdown() or self.stopping):
            recognized_object_array = RecognizedObjectArray()
            recognized_object_array.header.frame_id = self.output_frame
            self.mutex.acquire()
            recognized_object_array.objects = self.recognized_objects
            self.publisher.publish(recognized_object_array)
            self.mutex.release()
            self.rate.sleep()

    def add_object(self, recognized_object):
        self.mutex.acquire()
        self.recognized_objects.append(recognized_object)
        self.mutex.release()

    def remove_objects(self, recognized_objects=None, object_ids=None):
        if not object_ids:
            object_ids = []
        for recognized_object in recognized_objects:
            object_ids.append(recognized_object.type.key)
        new_recognized_objects = []
        for old_recognize_object in self.recognized_objects:
            if old_recognize_object.type.key not in object_ids:
                new_recognized_objects.append(old_recognize_object)
        self.mutex.acquire()
        self.recognized_objects = new_recognized_objects
        self.mutex.release()

    def add_objects(self, recognized_objects=None, object_ids=None, object_xyzrpys=None):
        if recognized_objects:
            for recognized_object in recognized_objects:
                self.add_object(recognized_object=recognized_object)
        if object_ids and object_xyzrpys:
            for i, object_id in enumerate(object_ids):
                object_xyzrpy = object_xyzrpys[i]
                recognized_object = self.make_recognized_object(object_id, object_xyzrpy)
                self.add_object(recognized_object=recognized_object)

    def set_objects(self, recognized_objects=None, object_ids=None, object_xyzrpys=None):
        self.mutex.acquire()
        self.recognized_objects = []
        self.mutex.release()
        self.add_objects(recognized_objects=recognized_objects, object_ids=object_ids, object_xyzrpys=object_xyzrpys)

    def make_recognized_object(self, object_id, object_xyzrpy, input_frame=None, db=None):
        if input_frame is None:
            input_frame = self.input_frame
        if db is None:
            db = self.db
        transform = self.tf_buffer.lookup_transform(self.output_frame,
                                                    input_frame,
                                                    rospy.Time(0),
                                                    rospy.Duration(1.0))
        recognized_object = RecognizedObject()
        quaternion = tf.transformations.quaternion_from_euler(object_xyzrpy[3], object_xyzrpy[4], object_xyzrpy[5])
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = input_frame
        pose_stamped.pose.position.x = object_xyzrpy[0]
        pose_stamped.pose.position.y = object_xyzrpy[1]
        pose_stamped.pose.position.z = object_xyzrpy[2]
        pose_stamped.pose.orientation.x = quaternion[0]
        pose_stamped.pose.orientation.y = quaternion[1]
        pose_stamped.pose.orientation.z = quaternion[2]
        pose_stamped.pose.orientation.w = quaternion[3]
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        pose_with_covariance_stamped = PoseWithCovarianceStamped()
        pose_with_covariance_stamped.pose.pose = pose_transformed.pose
        recognized_object.pose = pose_with_covariance_stamped
        recognized_object.header.frame_id = self.output_frame
        recognized_object.pose.header.frame_id = self.output_frame
        recognized_object.type.key = object_id
        recognized_object.type.db = db
        return recognized_object

    def stop(self):
        self.stopping = True
