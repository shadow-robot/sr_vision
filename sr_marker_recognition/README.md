# sr_marker_recognition

This is a package containing code necessary for running marker recognition libraries used at Shadow.

## Currently supported libraries

### Alvar
- Package: ar_track_alvar
- Source: <http://wiki.ros.org/ar_track_alvar>
- Usage:

To run the marker tracker in default mode:

`roslaunch sr_marker_recognition alvar.launch`

Parameters available:

`depth` - default *true* (recommended). Increases quality of pose estimation. Set to *false* if no depth data available from the camera driver.

`bundle` - default *true* (recommended). Increases quality of pose estimation. Set to *false* if you want to use for individual markers tracking. Please see the [wiki page](http://wiki.ros.org/ar_track_alvar#ar_track_alvar.2BAC8-post-fuerte.Automatic_XML_bundle_file_generation) on how to generate bundles.

`marker_size` - default *9* (cm).

`bundle_files` - as default, [this bundle](https://github.com/shadow-robot/common_resources/blob/kinetic-devel/sr_description_common/ar_markers/ar_marker_0_1_2_3_a4.xml) is used. Image file of the bundle for printing can be found [here](https://github.com/shadow-robot/sr_vision/blob/SRC-1223/F_generify_extrinsic_calibration/sr_marker_recognition/doc/ar_marker_a4_border_right_size.png).

`cam_image_topic`, `cam_info_topic` and `output_frame` need to be set for the specific camera driver you are using. As default they are set to work with Kinect and Astra cameras.

- Frame description:

Frame of a recognized Alvar marker is denoted as in the figure below:

![alt text](https://github.com/shadow-robot/sr_vision/blob/SRC-1223/F_generify_extrinsic_calibration/sr_marker_recognition/doc/alvar_frame.jpg)

where red, green and blue axes denote x, y and z coordinate respectively.

### Aruco
- Package: aruco_ros
- Source: <http://wiki.ros.org/aruco_ros>
- Usage:

To run marker tracker in default mode:

`roslaunch sr_marker_recognition aruco.launch`

Parameters available:

`marker_size` - default *0.1965* (m).

`marker_id` - Default *1*. Represents [this marker](https://github.com/shadow-robot/sr_vision/blob/SRC-1223/F_generify_extrinsic_calibration/sr_marker_recognition/utils/aruco.png).

`reference_frame`and `camera_frame` need to be set for the specific camera driver you are using. As default they are set to work with Kinect and Astra cameras.

- Frame description:

Frame of a recognized Aruco marker is denoted as in the figure below:

![alt text](https://github.com/shadow-robot/sr_vision/blob/SRC-1223/F_generify_extrinsic_calibration/sr_marker_recognition/doc/aruco_frame.jpg)

where red, green and blue axes denote x, y and z coordinate respectively.

## Troubleshooting
- Before launching either marker tracker, make sure that camera driver is running or put both in one launch file
- When printing the markers, make sure they are exactly the size you specified in you launch files
- Make sure your camera is properly intrinsicly calibrated which can significantly improve marker recognition reliability
