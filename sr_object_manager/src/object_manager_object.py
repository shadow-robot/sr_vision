import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from object_recognition_msgs.msg import RecognizedObject, RecognizedObjectArray
from sr_vision_common.pose_averager import PoseAverager


class ObjectManagerObject(object):
    def __init__(self, recognized_object=None, averaging_window_width=20):
        self.db_spec = None
        self.db_key = None
        self.confidence = 0.0
        self.pose_with_covariance_stamped = PoseWithCovarianceStamped()
        self.pose_averager = PoseAverager(window_width=averaging_window_width)
        if recognized_object is not None:
            self.init_from_recognized_object(recognized_object)

    def init_from_recognized_object(self, recognized_object):
        if not isinstance(recognized_object, RecognizedObject):
            error = 'Tried to initialise an {} object with an instance of {}, instead of an instance of {}.'.format(
                    self.__class__.__name__, recognized_object.__class__, 'RecognizedObject')
            rospy.logerror(error)
            raise TypeError(error)
        rospy.logdebug('Initialising an ObjectManagerObject from a RecognizedObject...')
        self.db_spec = recognized_object.type.db
        self.db_key = recognized_object.type.key
        self.confidence = recognized_object.confidence
        self.pose_with_covariance_stamped = recognized_object.pose
        self.pose_with_covariance_stamped.pose.pose = self.pose_averager.new_value(
                                                      self.pose_with_covariance_stamped.pose.pose)
        rospy.logdebug('Done. New ObjectManagerObject:')
        rospy.logdebug('{}'.format(self))

    def same_type(self, other):
        return self.same_db(other) and (self.db_key == other.db_key)

    def same_db(self, other):
        # TODO: Actually parse the database spec, and compare. For now, this is just a string comparison.
        return self.db_spec == other.db_spec

    def update(self, new):
        # TODO: Actually update values by averahing etc.
        rospy.logdebug('Updating ObjectManagerObject with new observations.')
        self.pose_with_covariance_stamped = new.pose
        self.pose_with_covariance_stamped.pose.pose.pose = self.pose_averager.new_value(new.pose.pose.pose)

    def __repr__(self):
        return """{} object:
db_key:       {}
pose:         {}""".format(self.__class__.__name__, self.db_key,
                           self.pose_with_covariance_stamped.pose.pose)

    @staticmethod
    def init_from_recognized_object_array(recognized_object_array, averaging_window_width):
        if not isinstance(recognized_object_array, RecognizedObjectArray):
            error = 'Tried to initialise an array of ObjectManagerObject objects with an instance of {}, instead of an '
            'instance of {}.'.format(recognized_object_array.__class__, 'RecognizedObjectArray')
            rospy.logerror(error)
            raise TypeError(error)
        rospy.logdebug('Initialising an array of ObjectManagerObject from a RecognizedObjectArray...')
        recognized_objects = []
        for recognized_object in recognized_object_array.objects:
            recognized_objects.append(ObjectManagerObject(recognized_object=recognized_object,
                                                          averaging_window_width=averaging_window_width))
        return recognized_objects
