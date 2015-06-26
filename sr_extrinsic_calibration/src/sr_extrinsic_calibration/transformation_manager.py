#!/usr/bin/env python
"""
See README.md
"""
import rospy
from tf.transformations import vector_norm
import tf2_ros
from sr_extrinsic_calibration.abort_exception import AbortException


class TransformationManager(object):
    _tf_sample_data_count = 101

    def __init__(self):
        """
        Creates object which can provide information about transformations and broadcast them
        """
        self._transformations_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._transformations_buffer)
        self._tf2_broadcaster = tf2_ros.TransformBroadcaster()

    def get_recent_transformation(self, target_frame, source_frame, abort_callback=None):
        """
        Find transformation between target_frame and source_frame

        @param target_frame frame from which we would be searching transformation
        @param source_frame frame to which we are searching transformation
        @param abort_callback callback to abort listening
        @return transformation
        """
        rate = rospy.Rate(10)
        found = False
        transformation = None
        while not found and not rospy.is_shutdown():

            if abort_callback is not None and abort_callback():
                raise AbortException("Aborting search of the transformation from " + target_frame + " to " +
                                     source_frame)

            try:
                transformation = self._transformations_buffer.lookup_transform(target_frame, source_frame, rospy.Time())
                found = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self._spin_once()
                rate.sleep()

        if not found:
            raise Exception("Could not find the transformation from " + target_frame + " to " + source_frame)

        return transformation

    @staticmethod
    def point_from_tf(transformation):
        """
        Return point tuple corresponding to transformation object

        @param transformation input data
        @return tuple (x, y, z) of translation
        """
        return (transformation.transform.translation.x,
                transformation.transform.translation.y,
                transformation.transform.translation.z)

    @staticmethod
    def _compare_transformations(transformation1, transformation2):
        """
        Compares two transformations by length of it's translations

        @param transformation1 first transformation
        @param transformation1 second transformation
        @return the same as cmp function for two floating values
        """
        norm1 = vector_norm(TransformationManager.point_from_tf(transformation1))
        norm2 = vector_norm(TransformationManager.point_from_tf(transformation2))
        return cmp(norm1, norm2)

    def get_recent_filtered_transformation(self, target_frame, source_frame, abort_callback=None):
        """
        Filters transformations between two frames using multiple samples and return median from them

        @param target_frame frame from which we would be searching transformation
        @param source_frame frame to which we are searching transformation
        @param abort_callback callback to abort listening
        @return transformation
        """
        data = []
        for _ in range(self._tf_sample_data_count):
            transformation = self.get_recent_transformation(target_frame, source_frame, abort_callback)
            data.append(transformation)

        data.sort(TransformationManager._compare_transformations)
        return data[int(self._tf_sample_data_count / 2)]

    def send_transformation(self, transformation):
        """
        Sends transformation

        @param transformation object which would be send
        """
        self._tf2_broadcaster.sendTransform(transformation)

    def _spin_once(self):
        """
        Can be implemented in descendants
        """
        pass


