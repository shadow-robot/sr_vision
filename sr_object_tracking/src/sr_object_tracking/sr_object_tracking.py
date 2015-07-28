#!/usr/bin/env python
import rospy
import cv2

from sensor_msgs.msg import Image
from sr_vision_msgs.msg import TrackBoxes

from utils import Utils


class SrObjectTracking(object):
    """
    Base class for the tracking.
    """

    def __init__(self):
        # Initialize a number of global variables
        self.size = rospy.get_param('~size')
        self.color = rospy.get_param('~color')

        self.utils = Utils(self.color)

        self.track_boxes = []
        self.seg_boxes = []

        self.prev_frame = None
        self.frame = None
        self.next_frame = None
        self.mask = None
        self.vis = None

        # Subscribe to the image topic and set the appropriate callback
        self.image_sub = rospy.Subscriber('camera/image_raw', Image,
                                          self.image_callback)
        self.segmentation_sub = rospy.Subscriber("/roi/segmented_box",
                                                 TrackBoxes,
                                                 self.segmentation_callback)

        # Initialize the Region of Interest publishers
        self.roi_pub = rospy.Publisher("roi/track_box", TrackBoxes,
                                       queue_size=1)

    def image_callback(self, data):
        """
        Convert the ROS image to OpenCV format using a cv_bridge helper
        function and make a copy
        """
        self.prev_frame = self.frame
        self.frame = self.next_frame
        self.next_frame = self.utils.convert_image(data, "bgr8")

    def segmentation_callback(self, data):
        """
        Get the ROI box (selected or segmented)
        """
        self.seg_boxes = []
        for segment in data.boxes:
            box = (int(segment.top_left.x), int(segment.top_left.y),
                   int(segment.bottom_right.x), int(segment.bottom_right.y))
            seg = SrObjectTracked(box)
            self.seg_boxes.append(seg)

    def selection_callback(self, segment):
        """
        Get the ROI box (selected or segmented)
        """

        self.selection = (int(segment.top_left.x), int(segment.top_left.y),
                          int(segment.bottom_right.x),
                          int(segment.bottom_right.y))

    def run(self):
        """
        Track the object(s) and publish the result
        @return - Success of the tracking as a booleen
        """
        self.track_boxes = []

        self.frame = cv2.blur(self.frame, (5, 5))
        self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        self.mask = self.utils.mask

        if len(self.seg_boxes) > 0:
            for i, seg in enumerate(self.seg_boxes):
                s = self.tracking(seg)
                if not s:
                    self.seg_boxes.remove(seg)
                    break
                else:
                    box = self.utils.roi_to_box(s, id=i)
                    self.track_boxes.append(box)
                self.roi_pub.publish(self.track_boxes)
            return True
        else:
            return False

    def tracking(self):
        """
        Tracking main algorithm to be redefined in the child classes
        """
        return


class SrObjectTracked(object):
    """
    Class for tracked object
    """

    def __init__(self, box):
        self.selection = box
        self.hist = None
        self.tracking_state = 0
        self.track_window = None
