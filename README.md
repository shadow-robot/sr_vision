[![Build Status](https://api.shippable.com/projects/5551c20fedd7f2c052e9809d/badge?branchName=kinetic-devel)](https://app.shippable.com/projects/5551c20fedd7f2c052e9809d) [![Circle CI](https://circleci.com/gh/shadow-robot/sr_vision.svg?style=shield)](https://circleci.com/gh/shadow-robot/sr_vision) [![Documentation Status](https://readthedocs.org/projects/sr-vision/badge/?version=latest)](http://sr-vision.readthedocs.org/)

sr_vision
------------

## Melodic migration notes 
### 28/01/2020
Several packages have been temporarily disabled or deliberately broken, rendering them partially or completely unusable. This has been done in order to quickly allow the repository to correctly build on our CI servers. Affected packages:
- sr_camera_launch
- sr_handeye_calibration
- sr_marker_recognition
- sr_object_segmentation
- sr_object_tracking
- sr_point_cloud

## Overview
Contains our vision related algorithm (segmentation, tracking, recognition, etc...)

  1. [Tracking](sr_object_tracking/)
This package contains the tracker executable. The launch file starts the video acquisition as well as the tracking node and the visualization one. A color parameter is necessary to process the segmentation.
  2. [Segmentation](sr_object_segmentation/)
Contains nodes for the color and shape based segmentation. The two of them can be set as parameters (default ones are red and strawberry).
  3. [Benchmarking](sr_object_benchmarking/)
Benchmarking package for the different segmentation algorithms.
  4. [Visualization](sr_gui_servoing/)
Visualization package : display the image from the camera, let the user select a region of interest, and display the tracking.
  5. [PointCloud utils](sr_point_cloud/)
This package contains a tracker, a point cloud triangulator, a segmentation tool and a tool to transform point clouds.
  6. [Messages](sr_vision_msgs/)
All messages, services and actions for sr_vision.
  7. [Extrinsic camera calibration](sr_extrinsic_calibration/)
Contains nodes for extrinsic camera calibration based on Alvar markers positions in camera and on real robot.

You can find the architecture diagram below for a closer look at how this works.

![Architecture Diagram](doc/sr_vision.png)


## Usage
For the segmentation, different colors are available : red, blue, green, yellow. A custom one can be added with the calibration script (see below for usage). Furthermore, different shape models : circle, rectangle, star, strawberry, banana, leaf. See the segmentation doc to add personnalize shapes.

### With a Kinect
`roslaunch sr_object_tracking tracking.launch kinect:=true color:=<color>`

### With an UVC camera
`roslaunch sr_object_tracking tracking.launch color:=<color>`

## Adding a custom color
To set the proper boundaries in order to process the image (which can be variable according to the luminosity, the camera, etc.) exists a [calibration script](src/sr_object_tracking/calibration.py).

### Usage
`roslaunch sr_object_tracking calibration.launch`

The original image is displayed on the left, and the mask on the right. Hue, Saturation and Value boundaries on which this one is based upon can be changed as convenience with the track bars. Finally, these values can be saved in a yaml file with the switch track bar at the bottom. To use it in the tracking, specify a 'custom' color argument. 



