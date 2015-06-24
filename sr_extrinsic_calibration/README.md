# sr_extrinsic_calibration

Contains nodes for extrinsic camera calibration based on Alvar markers positions in camera and on real robot

## Camera calibration node

This node publishes transformations from base of the robot to camera position

### Parameters

* camera_frame - name of the camera frame
* base_frame - name of the base frame of the robot
* robot_marker_frames - array of robot marker holder frames 3 or 4 items
* marker_frames - array of markers from any marker tracking system which should correspond to robot_marker_frames
* initial_tf - (optional) initial position of the camera

### How to add to launch file

This node can be added to launch file in order to correct camera position base on Alvar marker positions.
Examples of node usage in the launch file 

```
  <node name="camera_calibration" pkg="sr_extrinsic_calibration" type="camera_calibration_node" output="screen">
    <param name="camera_frame" value="camera"/>
    <param name="base_frame" value="delta_base"/>
    <rosparam param="robot_marker_frames">["marker_holder_1", "marker_holder_2", "marker_holder_3", "marker_holder_4"]</rosparam>
    <rosparam param="marker_frames">["ar_marker_1", "ar_marker_2", "ar_marker_3", "ar_marker_4"]</rosparam>
    <rosparam param="initial_tf">[0, 0, 10.0, 0, 0, 3.1415926]</rosparam>
  </node>
```



## Print camera TF

This node prints xml code for static transformation node with translation and rotation of the camera.
There is separate launch file print_camera_tf.launch
Before running this launch file robot description, tf publisher and camera nodes should be running.

### Parameters

The same as for camera calibration node

### How to run

```
roslaunch sr_extrinsic_calibration print_camera_tf.launch
```

In output of the command you will see following text

```
[INFO] [WallTime: 1427874441.813467] Camera transformation static node code
[INFO] [WallTime: 1427874441.813618] <node pkg="tf" type="static_transform_publisher" name="base_to_camera_publisher" args="0.000000 0.000000 2.000000 0.000000 0.000000 0.000000 1.000000 /delta_base /camera 100" />
```

You can put XML code of the static TF node into your launch file where you need to have camera in correct position.


