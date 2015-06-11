#!/usr/bin/env python
import cv
import sys
import rospy
import cv2
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class Tracking:
    def __init__(self, node_name):
        self.node_name = node_name

        # Initialize the Region of Interest and its publisher
        self.ROI = RegionOfInterest()
        self.roi_pub = rospy.Publisher("/roi", RegionOfInterest, queue_size=1)
        self.feature_size = 1
        self.show_boxes = True

        # Initialize a number of global variables
        self.frame = None
        self.frame_size = None
        self.frame_width = None
        self.frame_height = None
        self.depth_image = None
        self.marker_image = None
        self.display_image = None
        self.grey = None
        self.prev_grey = None
        self.selected_point = None
        self.selection = None
        self.drag_start = None
        self.keystroke = None
        self.detect_box = None
        self.track_box = None
        self.display_box = None
        self.keep_marker_history = False
        self.night_mode = False
        self.auto_face_tracking = False
        self.cps = 0  # Cycles per second = number of processing loops per second.
        self.cps_values = list()
        self.cps_n_values = 20
        self.busy = False
        self.resize_window_width = 0
        self.resize_window_height = 0
        self.face_tracking = False

        self.smin = 85
        self.vmin = 50
        self.vmax = 254
        self.threshold = 50


        # Create the cv_bridge object
        self.bridge = CvBridge()

        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=1)
        # Subscribe to the image and depth topics and set the appropriate callbacks
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.image_callback)
        # self.depth_sub = rospy.Subscriber("/camera/depth/image", Image, self.depth_callback)

    # The main processing function computes the histogram and backprojection
    def process_image(self, cv_image):
        # First blue the image
        frame = cv2.blur(cv_image, (5, 5))

        # Convert from RGB to HSV spave
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create a mask using the current saturation and value parameters
        mask = cv2.inRange(hsv, np.array((0., self.smin, self.vmin)), np.array((180., 255., self.vmax)))

        # If the user is making a selection with the mouse,
        # calculate a new histogram to track
        if self.selection is not None:
            x0, y0, w, h = self.selection
            x1 = x0 + w
            y1 = y0 + h
            self.track_window = (x0, y0, x1, y1)
            hsv_roi = hsv[y0:y1, x0:x1]
            mask_roi = mask[y0:y1, x0:x1]
            self.hist = cv2.calcHist([hsv_roi], [0], mask_roi, [16], [0, 180])
            cv2.normalize(self.hist, self.hist, 0, 255, cv2.NORM_MINMAX);
            self.hist = self.hist.reshape(-1)
            self.show_hist()

        if self.detect_box is not None:
            self.selection = None

        # If we have a histogram, tracking it with CamShift
        if self.hist is not None:
            # Compute the backprojection from the histogram
            backproject = cv2.calcBackProject([hsv], [0], self.hist, [0, 180], 1)

            # Mask the backprojection with the mask created earlier
            backproject &= mask

            # Threshold the backprojection
            ret, backproject = cv2.threshold(backproject, self.threshold, 255, cv.CV_THRESH_TOZERO)

            x, y, w, h = self.track_window
            if self.track_window is None or w <= 0 or h <= 0:
                self.track_window = 0, 0, self.frame_width - 1, self.frame_height - 1

            # Set the criteria for the CamShift algorithm
            term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)

            # Run the CamShift algorithm
            self.track_box, self.track_window = cv2.CamShift(backproject, self.track_window, term_crit)

            # Display the resulting backprojection
            cv2.imshow("Backproject", backproject)

        return cv_image

    def image_callback(self, data):

        # Create the main display window
        self.cv_window_name = self.node_name
        cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_AUTOSIZE)

        # Set a call back on mouse clicks on the image window
        cv.SetMouseCallback(self.node_name, self.on_mouse_click, None)

        # Convert the ROS image to OpenCV format using a cv_bridge helper function
        frame = self.convert_image(data)

        # Store the frame width and height in a pair of global variables
        if self.frame_width is None:
            self.frame_size = (frame.shape[1], frame.shape[0])
            self.frame_width, self.frame_height = self.frame_size

        # Create the marker image we will use for display purposes
        if self.marker_image is None:
            self.marker_image = np.zeros_like(frame)

        # Copy the current frame to the global image in case we need it elsewhere
        self.frame = frame.copy()

        # Process the image to detect and track objects or features
        processed_image = self.process_image(self.frame)

        # Make a global copy
        self.processed_image = processed_image.copy()

        # Display the user-selection rectangle or point
        self.display_selection()

        # Merge the processed image and the marker image
        self.display_image = cv2.bitwise_or(self.processed_image, self.marker_image)

        # If we have a track box, then display it.  The track box can be either a regular
        # cvRect (x,y,w,h) or a rotated Rect (center, size, angle).
        if self.show_boxes:
            if self.track_box is not None and self.is_rect_nonzero(self.track_box):
                print 'hello'
                if len(self.track_box) == 4:
                    x, y, w, h = self.track_box
                    size = (w, h)
                    center = (x + w / 2, y + h / 2)
                    angle = 0
                    self.track_box = (center, size, angle)
                else:
                    (center, size, angle) = self.track_box

                # For face tracking, an upright rectangle looks best
                if self.face_tracking:
                    pt1 = (int(center[0] - size[0] / 2), int(center[1] - size[1] / 2))
                    pt2 = (int(center[0] + size[0] / 2), int(center[1] + size[1] / 2))
                    cv2.rectangle(self.display_image, pt1, pt2, cv.RGB(50, 255, 50), self.feature_size, 8, 0)
                else:
                    # Otherwise, display a rotated rectangle
                    vertices = np.int0(cv2.cv.BoxPoints(self.track_box))
                    cv2.drawContours(self.display_image, [vertices], 0, cv.RGB(50, 255, 50), self.feature_size)

            # If we don't yet have a track box, display the detect box if present
            elif self.detect_box is not None and self.is_rect_nonzero(self.detect_box):
                (pt1_x, pt1_y, w, h) = self.detect_box
                if self.show_boxes:
                    cv2.rectangle(self.display_image, (pt1_x, pt1_y), (pt1_x + w, pt1_y + h), cv.RGB(50, 255, 50),
                                  self.feature_size, 8, 0)

        try:
            self.image_pub.publish(data)
        except CvBridgeError, e:
            print e

        # Publish the ROI
        # self.publish_roi()

        # Update the image display
        cv2.imshow(self.node_name, self.frame)
        cv2.waitKey(2)



    def depth_callback(self, data):
        # Convert the ROS image to OpenCV format using a cv_bridge helper function
        depth_image = self.convert_depth_image(data)

        # Process the depth image
        processed_depth_image = self.process_depth_image(depth_image)

        # Make global copies
        self.depth_image = depth_image.copy()
        self.processed_depth_image = processed_depth_image.copy()

    def on_mouse_click(self, event, x, y, flags, param):

        # This function allows the user to selection a ROI using the mouse
        if self.frame is None:
            return

        if event == cv.CV_EVENT_LBUTTONDOWN and not self.drag_start:
            self.features = []
            self.track_box = None
            self.detect_box = None
            self.selected_point = (x, y)
            self.drag_start = (x, y)

        if event == cv.CV_EVENT_LBUTTONUP:
            self.drag_start = None
            self.classifier_initialized = False
            self.detect_box = self.selection

        if self.drag_start:
            xmin = max(0, min(x, self.drag_start[0]))
            ymin = max(0, min(y, self.drag_start[1]))
            xmax = min(self.frame_width, max(x, self.drag_start[0]))
            ymax = min(self.frame_height, max(y, self.drag_start[1]))
            self.selection = (xmin, ymin, xmax - xmin, ymax - ymin)

    def is_rect_nonzero(self, rect):
        # First assume a simple CvRect type
        try:
            (_, _, w, h) = rect
            return (w > 0) and (h > 0)
        except:
            try:
                # Otherwise, assume a CvBox2D type
                ((_, _), (w, h), a) = rect
                return (w > 0) and (h > 0)
            except:
                return False

    def display_selection(self):
        # If the user is selecting a region with the mouse, display the corresponding rectangle for feedback.
        if self.drag_start and self.is_rect_nonzero(self.selection):
            x, y, w, h = self.selection
            cv2.rectangle(self.marker_image, (x, y), (x + w, y + h), (0, 255, 255), self.feature_size)
            self.selected_point = None

        # Else if the user has clicked on a point on the image, display it as a small circle.
        elif not self.selected_point is None:
            x = self.selected_point[0]
            y = self.selected_point[1]
            cv2.circle(self.marker_image, (x, y), self.feature_size, (0, 255, 255), self.feature_size)

    def convert_image(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            # cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            return np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e

    def convert_depth_image(self, ros_image):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(ros_image, "32FC1")
            # Convert to a numpy array since this is what OpenCV 2.3 uses
            depth_image = np.array(depth_image, dtype=np.float32)
            return depth_image

        except CvBridgeError, e:
            print e

    def publish_roi(self):
        if not self.drag_start:
            if self.track_box is not None:
                roi_box = self.track_box
            elif self.detect_box is not None:
                roi_box = self.detect_box
            else:
                return
        try:
            roi_box = self.cvBox2D_to_cvRect(roi_box)
        except:
            return

        # Watch out for negative offsets
        roi_box[0] = max(0, roi_box[0])
        roi_box[1] = max(0, roi_box[1])

        try:
            ROI = RegionOfInterest()
            ROI.x_offset = int(roi_box[0])
            ROI.y_offset = int(roi_box[1])
            ROI.width = int(roi_box[2])
            ROI.height = int(roi_box[3])
            self.roi_pub.publish(ROI)
        except:
            rospy.loginfo("Publishing ROI failed")

    '''
    def process_image(self, frame):
        return frame
    '''

    def process_depth_image(self, frame):
        return frame

    def cvBox2D_to_cvRect(self, roi):
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
    '''
    ic = image_converter()
    node_name = 'Tracking'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo("Starting node " + str(node_name))
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()
    '''
    try:

        node_name = "tracking"

        rospy.init_node(node_name)
        rospy.loginfo("Starting node " + str(node_name))
        Tracking(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down tracking node."
        cv.DestroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
