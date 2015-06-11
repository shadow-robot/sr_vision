#!/usr/bin/env python
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, RegionOfInterest

from sr_visualization.visualization import DisplayImage
from sr_object_tracking.camshift import CamshiftTracking


class VisionProcess(object):
    """
    Base class for the image processing from the camera.
    Segment it, track an object of interest and finally display the result.
    """

    def __init__(self, node_name, display=True):
        """
        Initialize the node
        @param display - Boolean controlling the display or not of the image
        """
        rospy.init_node(node_name)
        self.display = display

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Subscribe to the image topic and set the appropriate callback
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.image_callback)

        # Initialize the Region of Interest publishers
        self.roi_pub = rospy.Publisher("/roi", RegionOfInterest, queue_size=1)
        # self.roi_pc_pub = rospy.Publisher("/roi_pc", PointCloud2, queue_size=1)

        # Initialize a number of global parameters
        self.selection = None
        self.tracking_state = 0
        self.frame = None

        # Initialize the visualization object
        if self.display:
            self.display_window = DisplayImage()
            self.smin = self.display_window.smin
            self.threshold = self.display_window.threshold
        else:
            self.smin = 85
            self.threshold = 50

        # Initialize the tracking object
        self.track_algo = CamshiftTracking()

    def image_callback(self, data):
        """
        Launch the segmentation and the tracking, as well as the visualization if required.
        """
        # Convert the ROS image to OpenCV format using a cv_bridge helper function and make a copy
        self.frame = self.convert_image(data)

        # Process the tracking
        self.track_algo.tracking(self.frame, self.selection, self.tracking_state, self.smin, self.threshold)

        if self.display:
            self.show()

        try:
            self.publish_roi()
        except:
            pass

    def publish_roi(self):
        """
        Publish the region of interest as a RegionOfInterest message (2D box)
        """
        roi_box = box_to_rect(self.track_algo.track_box)

        # Watch out for negative offsets
        roi_box[0] = max(0, roi_box[0])
        roi_box[1] = max(0, roi_box[1])

        try:
            roi = RegionOfInterest()
            roi.x_offset = int(roi_box[0])
            roi.y_offset = int(roi_box[1])
            roi.width = int(roi_box[2])
            roi.height = int(roi_box[3])
            self.roi_pub.publish(roi)
        except:
            rospy.loginfo("Publishing ROI failed")

    def show(self):
        """
        In the case of display required, update the different variables needed for the tracking and display the window
        """
        # Update the variables
        self.selection = self.display_window.selection
        self.smin = self.display_window.smin
        self.threshold = self.display_window.threshold
        self.tracking_state = self.display_window.tracking_state

        # Show
        self.display_window.display(self.track_algo)

    def convert_image(self, ros_image):
        """
        Use cv_bridge() to convert the ROS image to OpenCV format
        @param ros_image - image in ROS format, to be converted in OpenCV format
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            return np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e


def box_to_rect(roi):
    """
    Convert a region of interest as box format into a rect format
    @param roi - region of interest (box format)
    @return - region of interest (rect format)
    """
    try:
        if len(roi) == 3:
            (center, size, angle) = roi
            pt1 = (int(center[0] - size[0] / 2), int(center[1] - size[1] / 2))
            pt2 = (int(center[0] + size[0] / 2), int(center[1] + size[1] / 2))
            rect = [pt1[0], pt1[1], pt2[0] - pt1[0], pt2[1] - pt1[1]]
        else:
            rect = list(roi)
    except:
        return [0, 0, 0, 0]

    return rect


def main(args):
    try:
        VisionProcess("vision_process")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
