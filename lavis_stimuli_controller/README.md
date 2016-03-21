#lavis_stimuli_controller

Authors:

    Peter Polidoro <polidorop@janelia.hhmi.org>

License:

    BSD

##Running

```shell
roslaunch lavis_stimuli_controller lavis_stimuli_controller.launch stimuli_feedback_period:=0.50 lavis_lights_controller:="mightex_controller_node" lavis_audio_controller:="pyaudio_controller_node"
```

```shell
rostopic pub -1 /camera/lavis_stimuli_controller_node/start std_msgs/Empty
rostopic pub -1  /camera/lavis_stimuli_controller_node/stop std_msgs/Empty
```
