#!/usr/bin/env python

# Copyright 2013 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

from geometry_msgs.msg import PoseWithCovarianceStamped
import sensor_msgs.point_cloud2

#
# Cloud and mesh utils
#


def centroid_mesh(mesh):
    """
    Takes a shape_msgs/Mesh as input and modifies it's
    points to be shifted centered around the centroid.
    """
    count = len(mesh.vertices)
    if count == 0:
        return
    sx = sy = sz = 0
    for p in mesh.vertices:
        sx += p.x
        sy += p.y
        sz += p.z
    cx = sx / count
    cy = sy / count
    cz = sz / count
    for p in mesh.vertices:
        p.x = p.x - cx
        p.y = p.y - cy
        p.z = p.z - cz


def centroid_point_cloud(cloud):
    """
    Takes a sensor_msgs/PointCloud2 as input returns a new cloud with
    points shifted to be centered around the centroid.
    """
    count = sx = sy = sz = 0
    for p in sensor_msgs.point_cloud2.read_points(cloud):
        count += 1
        sx += p[0]
        sy += p[1]
        sz += p[2]
    cx = sx / count
    cy = sy / count
    cz = sz / count

    new_points = []
    for p in sensor_msgs.point_cloud2.read_points(cloud):
        new_points.append((p[0] - cx, p[1] - cy, p[2] - cz) + p[3:])
    out_cloud = sensor_msgs.point_cloud2.create_cloud(
        cloud.header, cloud.fields, new_points)
    return out_cloud


def centroid_object(obj, frame_id=None):
    """
    Takes a object_recognition/RecognizedObject, modifiy in place the
    pointclouds and mesh points to be shifted to be centered around the
    centroid.
    """
    centroid_mesh(obj.bounding_mesh)
    for i in range(len(obj.point_clouds)):
        obj.point_clouds[i] = centroid_point_cloud(obj.point_clouds[i])
        if frame_id:
            obj.point_clouds[i].header.frame_id = frame_id
    if frame_id:
        obj.header.frame_id = frame_id
    # Everything centered on the frame so pose is all 0
    stamp = obj.pose.header.stamp
    obj.pose = PoseWithCovarianceStamped()
    obj.pose.header.stamp = stamp
