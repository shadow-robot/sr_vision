#!/usr/bin/python
from collections import deque

class PoseAverager(object):
    def __init__(self, initial_values=[], window_width=5):
        self.values = deque()
        self.average = None
        self.window_width = window_width
        for initial_value in initial_values:
            self.new_value(initial_value)

    def new_value(self, new_value):
        if not self.average:
            self.average = new_value
            self.values.append(new_value)
        else:
            if len(self.values) >= self.window_width:
                self.remove_from_average(self.values.popleft())
            else:
                self.remove_from_average(self.average)
            self.add_to_average(new_value)
            self.values.append(new_value)
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
