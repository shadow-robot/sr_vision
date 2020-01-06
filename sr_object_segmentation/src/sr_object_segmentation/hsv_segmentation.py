#!/usr/bin/env python

# Copyright 2019 Shadow Robot Company Ltd.
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

from scipy.ndimage.measurements import label, find_objects, center_of_mass
from sr_object_segmentation import SrObjectSegmentation


class HSVSegmentation(SrObjectSegmentation):
    """
    Segmentation based upon an HSV (Hue, Saturation, Value) colorization,
    according to a color given as parameter.
    """

    def __init__(self, color):
        """
        Initialize the HSV_based segmentation object
        """
        SrObjectSegmentation.__init__(self)
        self.name = 'HSV segmentation algorithm'
        self.color = color

    def segmentation(self, frame):
        """
        Segment the image with the proper color. Update the box attribute
        according to the biggest segment found.
        @param frame - image to be segmented (numpy format)
        """
        self.frame = frame

        closing = self.utils.hsv_transform(self.img, self.color, None)

        labeled_array, num_features = label(closing)
        objects = find_objects(labeled_array)
        self.nb_segments = num_features

        self.poses = center_of_mass(closing, labeled_array)

        segments = []
        for seg in objects:
            seg_x = seg[1]
            seg_y = seg[0]

            segments.append((int(seg_x.start), int(seg_y.start),
                             int(seg_x.stop - seg_x.start),
                             int(seg_y.stop - seg_y.start)))

        # Order the slices according to their size
        segments.sort(key=lambda s: s[2] * s[3], reverse=True)

        # Pick the biggest one
        self.segmented_box = segments[0]
