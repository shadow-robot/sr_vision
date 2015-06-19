#!/usr/bin/env python

import ctypes
import struct
import rospy
import sys

from sensor_msgs.msg import PointCloud2, PointField, RegionOfInterest

_DATATYPES = {}
_DATATYPES[PointField.INT8] = ('b', 1)
_DATATYPES[PointField.UINT8] = ('B', 1)
_DATATYPES[PointField.INT16] = ('h', 2)
_DATATYPES[PointField.UINT16] = ('H', 2)
_DATATYPES[PointField.INT32] = ('i', 4)
_DATATYPES[PointField.UINT32] = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)


class PointCloudCropping(object):
    """
    Getting the whole PointCloud from the camera and the tracking box from the tracking node, process a crop to publish the ROI as a sensor_msgs/PointCloud2
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
        except:
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
            gen = read_points(self.camera_cloud, pts=pts)
            try:
                roi_pts = list(gen)
            except:
                pass

            roi_pc = create_cloud(self.camera_cloud.header, self.camera_cloud.fields, roi_pts)
            self.publish_cloud(roi_pc)
        except:
            pass

    def publish_cloud(self, roi_cloud):
        try:
            self.cloud_pub.publish(roi_cloud)
        except:
            print 'Publishing roi_cloud failed'


def read_points(cloud, pts):
    """
    Read points from a sensor_msgs/PointCloud2 message.

    @param cloud - The point cloud to read from.
    @param pts - Coordinates of the points to be returned in the roi_cloud
    @return - Generator which yields a list of values for each point.
    """
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, None)
    unpack_from = struct.Struct(fmt).unpack_from

    for u, v in pts:
        p = unpack_from(cloud.data, (cloud.row_step * v) + (cloud.point_step * u))
        yield p


def create_cloud(header, fields, points):
    """
    Create a sensor_msgs/PointCloud2 message.

    @param header - The point cloud header.
    @param fields - The point cloud fields.
    @param points - The point cloud points.
    @return - The point cloud.
    """

    cloud_struct = struct.Struct(_get_struct_fmt(False, fields))

    buff = ctypes.create_string_buffer(cloud_struct.size * len(points))

    point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
    offset = 0
    for p in points:
        pack_into(buff, offset, *p)
        offset += point_step

    return PointCloud2(header=header,
                       height=1,
                       width=len(points),
                       is_dense=False,
                       is_bigendian=False,
                       fields=fields,
                       point_step=cloud_struct.size,
                       row_step=cloud_struct.size * len(points),
                       data=buff.raw)


def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print >> sys.stderr, 'Skipping unknown PointField datatype [%d]' % field.datatype
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


def main(args):
    try:
        rospy.init_node('cloud_cropping')
        node = PointCloudCropping()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."


if __name__ == '__main__':
    main(sys.argv)
