# sr_marker_recognition

This is a package containing code necessary for running marker recognition libraries used at Shadow.

## Currently supported libraries

###### Alvar
- Package: ar_track_alvar
- Source: <http://wiki.ros.org/ar_track_alvar>
- Usage:

Before running marker node, make sure that a camera driver is running or put both in one launch file. To run marker tracker in default mode:

`roslaunch sr_marker_recognition alvar.launch`

Parameters available:

`depth` - Default `true` (recommended). Increases quality of the pose estimation. Set to `false` if no depth data available from the camera driver

`bundle` - Default `true` (recommended). Increases quality of the pose estimation. Set to `false` if you want to use only one marker or set of individual markers. On how to generate bundle, please see the [wiki page](http://wiki.ros.org/ar_track_alvar#ar_track_alvar.2BAC8-post-fuerte.Automatic_XML_bundle_file_generation).

`marker_size` - Default `9` (cm).

`bundle_files` - as default, [this bundle](https://github.com/shadow-robot/common_resources/blob/kinetic-devel/sr_description_common/ar_markers/ar_marker_0_1_2_3_a4.xml) used.

`cam_image_topic`, `cam_info_topic` and `output_frame` need to be specified for the specific camera driver you are using. As default they are set to work with Kinect and Astra cameras.

- Frame description:

Frame of a recognized Alvar marker is denoted as in the figure below:

![alt text](https://github.com/shadow-robot/sr_vision/blob/SRC-1223/F_generify_extrinsic_calibration/sr_marker_recognition/docs/alvar_frame.jpg)

where red, green and blue axes denote x, y and z respectively.

###### Aruco
- Package: aruco_ros
- Source: <http://wiki.ros.org/aruco_ros>
- Usage:

Before running marker node, make sure that a camera driver is running or put both in one launch file. To run marker tracker in default mode:

`roslaunch sr_marker_recognition aruco.launch`

Parameters available:

`marker_size` - Default `0.1965` (m).

`marker_id` - Default `1`. Represents [this marker](https://github.com/shadow-robot/sr_vision/blob/SRC-1223/F_generify_extrinsic_calibration/sr_marker_recognition/utils/aruco.png).

`reference_frame`and `camera_frame` need to be specified for the specific camera driver you are using. As default they are set to work with Kinect and Astra cameras.

- Frame description:

Frame of a recognized Aruco marker is denoted as in the figure below:

![alt text](https://github.com/shadow-robot/sr_vision/blob/SRC-1223/F_generify_extrinsic_calibration/sr_marker_recognition/docs/aruco_frame.jpg)

where red, green and blue axes denote x, y and z respectively.
