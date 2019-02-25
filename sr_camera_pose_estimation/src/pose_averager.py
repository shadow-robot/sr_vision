#!/usr/bin/python

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

from collections import deque
from copy import deepcopy
from geometry_msgs.msg import Pose


class PoseAverager(object):
    def __init__(self, initial_values=[], window_width=5):
        self.values = deque()
        self.average = None
        self.window_width = window_width
        for initial_value in initial_values:
            self.new_value(initial_value)

    def new_value(self, new_value):
        values = []
        for value in self.values:
            values.append(value.position.x)
        if not self.average:
            self.average = deepcopy(new_value)
            self.values.append(new_value)
        else:
            if len(self.values) >= self.window_width:
                self.remove_from_average(self.values.popleft())
            else:
                self.remove_from_average(self.average)
            self.add_to_average(new_value)
            self.values.append(new_value)
        values = []
        for value in self.values:
            values.append(value.position.x)
        return self.average

    def remove_from_average(self, pose):
        average_n = len(self.values) + 1
        self.average.position.x -= pose.position.x / average_n
        self.average.position.y -= pose.position.y / average_n
        self.average.position.z -= pose.position.z / average_n
        self.average.orientation.x -= pose.orientation.x / average_n
        self.average.orientation.y -= pose.orientation.y / average_n
        self.average.orientation.z -= pose.orientation.z / average_n
        self.average.orientation.w -= pose.orientation.w / average_n

    def add_to_average(self, pose):
        average_n = len(self.values) + 1
        self.average.position.x += pose.position.x / average_n
        self.average.position.y += pose.position.y / average_n
        self.average.position.z += pose.position.z / average_n
        self.average.orientation.x += pose.orientation.x / average_n
        self.average.orientation.y += pose.orientation.y / average_n
        self.average.orientation.z += pose.orientation.z / average_n
        self.average.orientation.w += pose.orientation.w / average_n

if __name__ == '__main__':
    test_vector = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    averager = PoseAverager(window_width=10)
    for value in test_vector:
        pose = Pose()
        pose.position.x = value
        averager.new_value(pose)
