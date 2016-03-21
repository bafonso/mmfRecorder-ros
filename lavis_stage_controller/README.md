#lavis_stage_controller

Authors:

    Peter Polidoro <polidorop@janelia.hhmi.org>

License:

    BSD

##Running

```shell
roslaunch lavis_stage_controller lavis_stage_controller.launch stage_feedback_period:=0.250 image_x_stage_x_gain:=1 image_x_stage_y_gain:=0 image_y_stage_x_gain:=0 image_y_stage_y_gain:=1 lavis_stage:="zaber_stage_node"
```

```shell
rostopic pub -1 /camera/lavis_stage_controller_node/start std_msgs/Empty
rostopic pub -1  /camera/lavis_stage_controller_node/stop std_msgs/Empty
```
