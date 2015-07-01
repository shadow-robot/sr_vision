#!/usr/bin/env python

import rospy
import sys

from sensor_msgs.msg import PointCloud2, RegionOfInterest
from sensor_msgs import point_cloud2


class PointCloudCropping(object):
    """
    Getting the whole PointCloud from the camera and the tracking box from the tracking node, process a crop to
    publish the ROI as a sensor_msgs/PointCloud2
    """

    def __init__(self):
        self.camera_cloud_sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2,
                                                 self.camera_cloud_callback)
        self.roi_sub = rospy.Subscriber("/roi/track_box", RegionOfInterest, self.roi_callback)
        self.cloud_pub = rospy.Publisher("/roi/track_cloud", PointCloud2, queue_size=1)

        self.camera_cloud = None
        self.track_box = None
        self.tracking_state = 0

    def roi_callback(self, data):
        if data.height != 0 and data.width != 0:
            self.track_box = data
            self.tracking_state = 1

        # Process the cropping if the track_box is initialized
        try:
            self.crop()
        except (TypeError, AttributeError):
            pass

    def camera_cloud_callback(self, data):
        self.camera_cloud = data

    def crop(self):
        """
        Create a new PointCloud from the tracking box's points and publish it.
        """
        pt1 = (self.track_box.x_offset, self.track_box.y_offset)
        pt2 = (self.track_box.x_offset + self.track_box.width, self.track_box.y_offset + self.track_box.height)
        pts = [(x, y) for x in range(pt1[0], pt2[0]) for y in range(pt1[1], pt2[1])]

        roi_pts = []
        try:
            gen = point_cloud2.read_points(self.camera_cloud, uvs=pts)
            try:
                roi_pts = list(gen)
            except AssertionError:
                pass
            roi_pc = point_cloud2.create_cloud(self.camera_cloud.header, self.camera_cloud.fields, roi_pts)
            self.publish_cloud(roi_pc)
        except (AttributeError, point_cloud2.struct.error):
            pass

    def publish_cloud(self, roi_cloud):
        try:
            self.cloud_pub.publish(roi_cloud)
        except rospy.ROSInterruptException:
            print 'Publishing roi_cloud failed'


def main():
    try:
        rospy.init_node('cloud_cropping')
        PointCloudCropping()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."


if __name__ == '__main__':
    main()
