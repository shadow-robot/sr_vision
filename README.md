sr_vision
------------

## Overview
Contains our vision related algorithm (segmentation, tracking, recognition, etc...)

  1. ![Tracking](sr_object_tracking/) 
This package contains the tracker executable. The launch file starts the Kinect as well as the tracking node and the visualization one.
  2. ![Segmentation](sr_object_segmentation/)
Segment the image from the camera in order to find the region of interest.
  3. ![Benchmarking](sr_object_benchmarking/)
Benchmarking package for the different segmentation algorithms.
  4. ![Visualization](sr_gui_servoing/)
Visualization package : display the image from the camera, let the user select a region of interest, and display the tracking.
  5. ![PointCloud utils](sr_point_cloud/)
This package contains a tracker, a point cloud triangulator, a segmentation tool and a tool to transform point clouds.
  6. ![Messages](sr_vision_msgs/)
All messages, services and actions for sr_vision.


You can find the architecture diagram below for a closer look at how this works.

![Architecture Diagram](doc/sr_vision.png)

