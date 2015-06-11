#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import sys

from sr_transform.from_camera import FromCamera
from sr_transform.srv import transf


class DisplayImage():
    """
    Visualization with the OpenCV library
    """
    def __init__(self, node_name):
        """
        Initialize the different windows
        """
        node_name = node_name
        rospy.init_node(node_name)
        rospy.loginfo("Starting node " + str(node_name))
        rospy.on_shutdown(self.destroy)

        #rospy.init_node("camera_transform")
        #camera = FromCamera()


        self.frame = cv_image
        print self.frame
        # Initialize a number of global variables
        self.cv_window_name = None
        self.hist = None
        self.selection = None
        self.select_size = 0
        self.drag_start = None
        self.track_box = None
        self.track_window = None
        self.tracking_state = 0
        self.frame_width = None
        self.frame_height = None
        self.frame_size = None
        self.vis = None

        # The minimum saturation of the tracked color in HSV space,
        # as well as the min and max value (the V in HSV) and a
        # threshold on the backprojection probability image.
        self.smin = rospy.get_param("~smin", 85)
        self.vmin = rospy.get_param("~vmin", 50)
        self.vmax = rospy.get_param("~vmax", 254)
        self.threshold = rospy.get_param("~threshold", 50)

        self.run()

    def load_windows(self):
        cv2.namedWindow("Parameters", cv2.CV_WINDOW_AUTOSIZE)
        cv2.moveWindow("Parameters", 700, 325)

        # Create the slider controls for saturation, value and threshold
        cv2.createTrackbar("Saturation", "Parameters", self.smin, 130, self.set_smin)
        cv2.createTrackbar("Min Value", "Parameters", self.vmin, 100, self.set_vmin)
        cv2.createTrackbar("Max Value", "Parameters", self.vmax, 255, self.set_vmax)
        cv2.createTrackbar("Threshold", "Parameters", self.threshold, 255, self.set_threshold)


    def run(self):
        self.vis = self.frame.copy()
        hsv = cv2.cvtColor(self.vis, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array((0., self.smin, self.vmin)), np.array((180., 255., self.vmax)))

        self.load_windows()

        if self.selection:
            x0, y0, x1, y1 = self.selection
            # self.track_window = (max(0,x0), max(0,y0), max(0,x1 - x0), max(0,y1 - y0))
            self.track_window = (x0, y0, x1 - x0, y1 - y0)
            hsv_roi = hsv[y0:y1, x0:x1]
            mask_roi = mask[y0:y1, x0:x1]
            hist = cv2.calcHist([hsv_roi], [0], mask_roi, [16], [0, 180])
            cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
            self.hist = hist.reshape(-1)
            self.show_hist()

            vis_roi = self.vis[y0:y1, x0:x1]
            cv2.bitwise_not(vis_roi, vis_roi)
            self.vis[mask == 0] = 0

        if self.tracking_state == 1:
            self.selection = None
            prob = cv2.calcBackProject([hsv], [0], self.hist, [0, 180], 1)
            prob &= mask
            term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
            nb_iter = cv2.meanShift(prob, self.track_window, term_crit)[0]
            if nb_iter != 0:
                self.track_box, self.track_window = cv2.CamShift(prob, self.track_window, term_crit)
            try:
                cv2.ellipse(self.vis, self.track_box, (0, 0, 255), 2)
            except:
                print self.track_box

    def show_hist(self):
        """
        Show the color histogram of the selected region
        """
        bin_count = self.hist.shape[0]
        bin_w = 24
        img = np.zeros((256, bin_count * bin_w, 3), np.uint8)
        for i in xrange(bin_count):
            h = int(self.hist[i])
            cv2.rectangle(img, (i * bin_w + 2, 255), ((i + 1) * bin_w - 2, 255 - h),
                          (int(180.0 * i / bin_count), 255, 255), -1)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        cv2.imshow('Histogram', img)

    # These are the callbacks for the slider controls
    def set_smin(self, pos):
        self.smin = pos

    def set_vmin(self, pos):
        self.vmin = pos

    def set_vmax(self, pos):
        if pos > 50:
            self.vmax = pos

    def set_threshold(self, pos):
        self.threshold = pos

    def destroy(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

def main(args):

    try:
        rospy.init_node("visualization")
        rospy.wait_for_service('camera_transform')
        try:
            srv = rospy.ServiceProxy('camera_transform', transf)
            srv.spin()
            img = srv()
            print img
            Visualization(img)
            r.spin()
            #camera = FromCamera()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        #capture(args)
        #print camera.cv_image

    except KeyboardInterrupt:
        print "Shutting down visualization."
    '''
    try:
        DisplayImage("display_image")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down visualization node."
        cv2.DestroyAllWindows()
    '''

if __name__ == '__main__':
    main(sys.argv)
