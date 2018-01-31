# sr_camera_pose_estimation

This is a package containing code necessary for calibrating camera pose in the scene with a marker.

## Overview

The idea of method is to find a root (usually robot base) -> camera tranform when camera -> marker and marker -> root transforms are known. The general concept is described in the figure below:
![alt text](https://github.com/shadow-robot/sr_vision/blob/SRC-1223/F_generify_extrinsic_calibration/sr_camera_pose_estimation/doc/diagram_camera_pose.jpg)

In order to close the tf chain and find the calibration, as seen from the figure above, package needs to be provided three pieces of information:
- root frame - provided by mock node in this package or a properly set scene,
- camera frame to marker frame transform - provided by [camera driver](https://github.com/shadow-robot/sr_vision/tree/SRC-1223/F_generify_extrinsic_calibration/sr_camera_launch) and [marker recognition](https://github.com/shadow-robot/sr_vision/tree/SRC-1223/F_generify_extrinsic_calibration/sr_marker_recognition) node.
- marker pose with respect to the root - mocked or manually measured in the real scene.

## Usage

In order to launch the calibration in a default mode run the following:

`roslaunch sr_camera_pose_estimation calibration.launch`

Before, make sure that camera driver and marker recognition are running (or include all launch files in one like in the [example launch](https://github.com/shadow-robot/sr_vision/blob/SRC-1223/F_generify_extrinsic_calibration/sr_camera_pose_estimation/launch/example_setup.launch) file).

Parameters available:

`marker_to_root_cartesian` - cartesian pose of the marker with respect to the root frame. The rotations in the frame are represented by [extrinsic rotations](https://en.wikipedia.org/wiki/Euler_angles#Definition_by_extrinsic_rotations) or eurler angles. To properly set this value you need to know the coordinate frame of the markers, as described [here](https://github.com/shadow-robot/sr_vision/tree/SRC-1223/F_generify_extrinsic_calibration/sr_marker_recognition).

`root_frame_name` - default `ra_base_link`. Can be set to any static frame that is in the scene as long as the distance to the marker can be measured (or mocked).

`camera_frame_name` - default `camera_link`. Frame of the camera from which we want to know the final transform to the root frame.

Additional parameters:

`filtering` - default `true`. Running an moving average filter on the camera -> marker filter to reduce noisy data.

`window_width` - default `500`. Window width for the moving average filter.

`continuous` - default `true`. Runs the camera pose estimation continuously (allowing movement of the camera while running the node) or calculates the pose only ones (removes the noise but camera needs to stay in the same pose until the next run of the calibration).

## Example use

An example use of the package can be shown by running

`roslaunch sr_camera_pose_estimation example_setup.launch`

In this setup, following elements are used:

- real Kinect camera with depth registration,
- Alvar marker recognition in bundle depth mode,
- mock `world` and `ra_base_link` frames are published using [this](https://github.com/shadow-robot/sr_vision/blob/SRC-1223/F_generify_extrinsic_calibration/sr_camera_pose_estimation/launch/mock_world_transforms.launch) launch file,
- [marker bundle](https://github.com/shadow-robot/common_resources/blob/kinetic-devel/sr_description_common/ar_markers/ar_marker_0_1_2_3_a4.xml) places in the view of the camera.

After launching the nodes, following tf tree is obtained:

![alt text](https://github.com/shadow-robot/sr_vision/blob/SRC-1223/F_generify_extrinsic_calibration/sr_camera_pose_estimation/doc/tf_tree_marker.jpg)

The scene can be investigated in rviz, where properly positioned frames and point cloud are present:

![alt text](https://github.com/shadow-robot/sr_vision/blob/SRC-1223/F_generify_extrinsic_calibration/sr_camera_pose_estimation/doc/calib_scene.jpg)
