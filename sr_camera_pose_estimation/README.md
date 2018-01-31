# sr_camera_pose_estimation

This is a package containing code necessary for calibrating camera pose in the scene with a marker.

## Overview

The idea of method is to find a root (usually robot base) -> camera tranform when camera -> marker and marker -> root transforms are known. The general concept is described in the figure below:
![alt text](https://github.com/shadow-robot/sr_vision/blob/SRC-1223/F_generify_extrinsic_calibration/sr_camera_pose_estimation/doc/diagram_camera_pose.jpg)

In order to find the calibration, as seen from the figure above, package needs to be provided three pieces of information:
- root frame - provided by mock node in this package or a properly set scene,
- camera frame to marker frame transform - provided by [camera driver](https://github.com/shadow-robot/sr_vision/tree/SRC-1223/F_generify_extrinsic_calibration/sr_camera_launch) and [marker recognition](https://github.com/shadow-robot/sr_vision/tree/SRC-1223/F_generify_extrinsic_calibration/sr_marker_recognition) node.
- marker pose with respect to the root - mocked or manually measured in the real scene.

