#!/usr/bin/env python

import cv2
import numpy as np
import rospy

from sr_object_tracking.utils import Utils
from sr_object_segmentation.hsv_segmentation import hsv_transform

from sensor_msgs.msg import RegionOfInterest, Image
from geometry_msgs.msg import PoseStamped


class DisplayImage(object):
    """
    Visualization class (using OpenCV tools)
    """

    def __init__(self, node_name):

        rospy.init_node(node_name)

        self.color = rospy.get_param('~color')
        self.cv_window_name = 'Video'

        # Initialize a number of global variables
        self.smin = rospy.get_param('~saturation')
        self.hist = None
        self.drag_start = (-1, -1)
        self.track_box = None
        self.tracking_state = 0
        self.track_window = None
        self.selection = (0, 0, 0, 0)
        self.frame = None
        self.frame_width = None
        self.frame_height = None
        self.frame_size = None
        self.vis = None

        self.utils = Utils()

        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image,
                                          self.display)
        self.roi_sub = rospy.Subscriber("/roi/track_box", RegionOfInterest,
                                        self.roi_callback)

        self.selection_pub = rospy.Publisher("/roi/selection",
                                             RegionOfInterest, queue_size=1)
        self.pose_pub = rospy.Publisher("roi/pose", PoseStamped,
                                        queue_size=1)

    def display(self, data):
        """
        Display the main window as well as the parameters choice window and
        the histogram one (Camshift backprojection).
        Draw a rectangle around the tracking box
        """
        # Create the main display window and the histogram one
        cv2.namedWindow(self.cv_window_name, cv2.CV_WINDOW_AUTOSIZE)
        if self.hist is not None:
            cv2.namedWindow('Histogram', cv2.CV_WINDOW_AUTOSIZE)
            cv2.moveWindow("Histogram", 700, 600)

        # Set a call back on mouse clicks on the image window
        cv2.setMouseCallback(self.cv_window_name, self.on_mouse_click, None)

        # Convert the ROS image to OpenCV format using a cv_bridge helper
        # function and make a copy
        self.frame = self.utils.convert_image(data, "bgr8")
        self.vis = self.frame.copy()

        try:
            # Seems better with a blur
            self.vis = cv2.blur(self.vis, (5, 5))
            hsv = cv2.cvtColor(self.vis, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, np.array((0., self.smin, 54)),
                               np.array((180., 255., 255)))
            if self.selection != (0, 0, 0, 0):
                x0, y0, x1, y1 = self.selection
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
        except cv2.error:
            pass

        try:
            roi = self.utils.publish_box(self.selection)
            pose = self.utils.publish_pose(roi=roi)

            self.pose_pub.publish(pose)
            self.selection_pub.publish(roi)
        except (AttributeError, TypeError, IndexError):
            pass

        img = hsv_transform(self.vis, self.color)

        # Draw the tracking box, if possible
        try:
            cv2.rectangle(self.vis, self.track_box[0], self.track_box[1],
                          (0, 0, 255), 2)
            cv2.rectangle(img, self.track_box[0], self.track_box[1],
                          (0, 0, 255), 2)
        except (cv2.error, TypeError):
            pass

        # Display the main window
        cv2.imshow(self.cv_window_name, np.hstack((self.vis, img)))
        cv2.waitKey(1)

    def roi_callback(self, data):
        """
        Convert the sensor_msgs/RegionOfInterest into a tuple of extrema
        coordinates
        """
        pt1 = (data.x_offset, data.y_offset)
        pt2 = (data.x_offset + data.width, data.y_offset + data.height)
        self.track_box = (pt1, pt2)

    def on_mouse_click(self, event, x, y, flags, _):
        """
        Select a ROI using the dragging
        """

        x, y = np.int16([x, y])
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag_start = (x, y)
            self.tracking_state = 0
            return
        if self.drag_start != (-1, -1):
            if flags & cv2.EVENT_FLAG_LBUTTON:
                h, w = self.frame.shape[:2]
                xo, yo = self.drag_start
                x0, y0 = np.maximum(0, np.minimum([xo, yo], [x, y]))
                x1, y1 = np.minimum([w, h], np.maximum([xo, yo], [x, y]))
                self.selection = (0, 0, 0, 0)
                if x1 - x0 > 0 and y1 - y0 > 0:
                    self.selection = x0, y0, x1, y1

            else:
                self.drag_start = (-1, -1)
                if self.selection != (0, 0, 0, 0):
                    self.tracking_state = 1
                    self.selection = (0, 0, 0, 0)

    def show_hist(self):
        """
        Show the color histogram of the selected region
        """
        bin_count = self.hist.shape[0]
        bin_w = 24
        img = np.zeros((256, bin_count * bin_w, 3), np.uint8)
        for i in xrange(bin_count):
            h = int(self.hist[i])
            cv2.rectangle(img, (i * bin_w + 2, 255),
                          ((i + 1) * bin_w - 2, 255 - h),
                          (int(180.0 * i / bin_count), 255, 255), -1)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        cv2.imshow('Histogram', img)


def main():
    try:
        DisplayImage("viewer")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down sr_gui_servoing node."
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
