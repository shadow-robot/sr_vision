# sr_vision_mocks

This package provides various utilities for mocking vision solutions used at Shadow.

## mock_recognized_objects

This node finds the tfs of recognized objects and publishes them in a proper format to `recognized_objects` topic. That topic is used by grasping pipeline defined in [sr_manipulatio_grasp_conductor](https://github.com/shadow-robot/sr_manipulation/tree/F_%23SRC-1550_isee_demo/sr_manipulation_grasp_conductor/src/sr_manipulation_grasp_conductor) package.
 
 Currently supported objects: `utl5_small`, `duplo_2x4x1`.

## spawn_object

Spawns a tf(s) of specified object type in the scene. Example use:
```sh
rosrun sr_vision_mocks spawn_object.py -p "duplo_2x4x1" 0.7 0.6 0.763 0 0 1.57 -p "duplo_2x4x1" 0.7 0.7 0.763 0 0 0
```

This will spawn two duplo tfs. If you are running scene in simulation and want the objects to appear in Gazebo as well, use `-g` or `--gazebo` flag, e.g.:
```sh
rosrun sr_vision_mocks spawn_object.py -p "duplo_2x4x1" 0.7 0.6 0.763 0 0 1.57 -g
```
In order to be able to spawn model in gazebo, an dedicated sdf file needs to be present in [sr_description_common](https://github.com/shadow-robot/common_resources/tree/F_adding_common_msgs/sr_description_common) package.

## move_object

Used together with spawn_object node. Calls provided service that moves a tf (and model if running gazebo) to a specified position. Example use:

```sh
rosrun sr_vision_mocks move_object.py -o duplo_2x4x1_0 -p 0.6 0.701 0.7633 0.0 0.0 1.57
```