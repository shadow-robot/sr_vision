#!/usr/bin/env python

import os
import cv2
import numpy as np
import rospy
import yaml

from utils import Utils

from sensor_msgs.msg import Image


class DisplayImage(object):
    """
    Visualization class (using OpenCV tools)
    """

    def __init__(self):
        self.cv_window_name = 'calibration'
        rospy.init_node(self.cv_window_name)

        # Initialize a number of global variables
        self.h_min, self.s_min, self.v_min = 157, 55, 0
        self.h_max, self.s_max, self.v_max = 255, 255, 255

        self.frame = None
        self.frame_width = None
        self.frame_height = None
        self.frame_size = None
        self.vis = None

        self.utils = Utils()

        self.image_sub = rospy.Subscriber('camera/image_raw', Image,
                                          self.display)

        # Create parameters window with the slider controls for saturation,
        #  value and threshold
        cv2.namedWindow("Parameters", cv2.CV_WINDOW_AUTOSIZE)

        cv2.createTrackbar("H_min", "Parameters", self.h_min, 255,
                           self.set_hmin)
        cv2.createTrackbar("S_min", "Parameters", self.s_min, 255,
                           self.set_smin)
        cv2.createTrackbar("V_min", "Parameters", self.v_min, 255,
                           self.set_vmin)
        cv2.createTrackbar("H_max", "Parameters", self.h_max, 255,
                           self.set_hmax)
        cv2.createTrackbar("S_max", "Parameters", self.s_max, 255,
                           self.set_smax)
        cv2.createTrackbar("V_max", "Parameters", self.v_max, 255,
                           self.set_vmax)

        # Create switch for saving parameters
        self.switch = '0 : Default \n1 : Save the new parameters'
        cv2.createTrackbar(self.switch, 'Parameters', 0, 1, self.nothing)

    def display(self, data):
        """
        Display the main window as well as the parameters choice window
        """
        # Create the main display window
        cv2.namedWindow(self.cv_window_name, cv2.cv.CV_WINDOW_NORMAL)
        cv2.resizeWindow(self.cv_window_name, 1000, 400)

        # Convert the ROS image to OpenCV format using a cv_bridge helper
        # function and make a copy
        self.frame = self.utils.convert_image(data, "bgr8")
        self.vis = self.frame.copy()

        self.boundaries = [[self.h_min, self.s_min, self.v_min],
                           [self.h_max, self.s_max, self.v_max]]

        img = self.utils.hsv_transform(self.vis, 'custom', self.boundaries)

        save = cv2.getTrackbarPos(self.switch, 'Parameters')
        if save:
            self.save_parameters()

        # Display the main window
        cv2.imshow(self.cv_window_name, np.hstack((self.vis, img)))
        cv2.waitKey(1)

    def save_parameters(self):
        path = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.realpath(__file__)))) + '/config/boundaries.yaml'
        with open(path, 'w') as outfile:
            outfile.write(yaml.dump(self.boundaries, default_flow_style=True))

    # Set the callbacks for the slider controls
    def set_hmin(self, pos):
        self.h_min = pos

    def set_smin(self, pos):
        self.s_min = pos

    def set_vmin(self, pos):
        self.v_min = pos

    def set_hmax(self, pos):
        self.h_max = pos

    def set_smax(self, pos):
        self.s_max = pos

    def set_vmax(self, pos):
        self.v_max = pos

    def nothing(self, x):
        pass


def main():
    try:
        DisplayImage()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down calibration node."
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
