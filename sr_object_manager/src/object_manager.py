import rospy
from object_recognition_msgs.msg import RecognizedObjectArray


class ObjectManager(object):
    def __init__(self):
        rospy.loginfo('Initialising ObjectManager instance:')
        self.get_params()
        rospy.loginfo('Recognized object topic:     {}'.format(self.recognized_objects_topic))
        self.recognized_objects_subscriber = rospy.Subscriber(self.recognized_objects_topic, RecognizedObjectArray,
                                                              self.on_recognized_object_array)

    def get_params(self):
        self.recognized_objects_topic = rospy.get_param('~recognized_objects_topic')

    def on_recognized_object_array(self, recognized_object_array):
        rospy.loginfo('Received recognized object array.')
