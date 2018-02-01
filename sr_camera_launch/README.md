# sr_camera_launch

This is a package containing code necessary for running cameras commonly used at Shadow.

## Currently supported cameras
### Kinect
- Driver: <http://wiki.ros.org/freenect_launch>

- Usage:

To run the camera in default mode:

`roslaunch sr_camera_launch kinect.launch`

Parameters available:

`sim` - default *false*. Set *true* for simulated camera

`depth_registration` - default *true*. Set *false* to switch off depth registration in the driver

### Orbbec Astra
- Driver: <http://wiki.ros.org/astra_launch>

- Usage:

To run the camera in default mode:

`roslaunch sr_camera_launch astra.launch`

Parameters available:

`sim` - default *false*. Set *true* for simulated camera

`depth_registration` - default *true*. Set *false* to switch off depth registration in the driver

## Utilities
### Camera calibration
To run intrinsic calibration for the camera:

`roslaunch sr_camera_launch camera_calibration.launch`

As default set to be used with Kinect and with a checkerboard used in Shadow workshop. To switch camera use flag:

`camera_name` - set *kinect* for Kinect and *astra* for Orbbec Astra.

In you're using different checkerboard, `size` and `square` parameters can be adjusted. For more information on calibration, see used package [wiki](http://wiki.ros.org/camera_calibration).

## Troubleshooting

- If you are running the code in a docker container, make sure your camera is plugged in before you run/start the container
