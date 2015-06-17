sr_vision
------------

## Overview
Contains our vision related algorithm (segmentation, tracking, recognition, etc...)

  1. sr_object_tracking
  2. sr_object_segmentation
  3. sr_gui_servoing
  4. sr_point_cloud
  5. sr_vision_msgs

####sr_point_cloud
This package contains a tracker, a point cloud triangulator, a segmentation tool and a tool to transform point clouds.

####sr_object_tracking
This package contains the tracker executable. The launch file starts the Kinect as well as the tracking node and the visualization one.


You can find the architecture diagram below for a closer look at how this works.

![Architecture Diagram](doc/sr_vision.png)

