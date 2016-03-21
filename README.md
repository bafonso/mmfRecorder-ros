#lavis

Authors:

    Peter Polidoro <polidorop@janelia.hhmi.org>

License:

    BSD

##Running

```shell
lavis
# wait until you see "lavis running!"
```

Keyboard control of stage.

```shell
lavis_key
```

###Web GUI

Open a browser and navigate to

    http://localhost:5000/

###Console for Monitoring Status Messages

```shell
rqt_console
```

###Monitoring Features on Command Line

```shell
rostopic echo /camera/features
```

###Plotting Features

```shell
rqt_plot /camera/features
rqt_plot /camera/features/eig_reduced
```

###Dynamically Reconfiguring Parameters (like camera frame rate)

```shell
rosrun rqt_reconfigure rqt_reconfigure
```

###Recompiling After Modifying Workspace Code

Modify files in ~/lavis_ws/src/

```shell
cd ~/lavis_ws
catkin_make
```

##Data Files

Data files are writen to the directory ~/data

##Staying Updated

```shell
cd ~/lavis_ws/src/lavis
git pull origin master
cd ~/lavis_ws/src
wstool merge lavis/.rosinstall
cd ~/lavis_ws
wstool update -t src
rosdep update
rosdep install --from-paths . --reinstall
catkin_make
```

##Installation (Ubuntu 14.04)

###ROS

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install libgl1-mesa-dev-lts-utopic
sudo apt-get install ros-jade-desktop-full
echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall
sudo apt-get install git
mkdir ~/git
cd ~/git
git clone https://github.com/janelia-ros/rosdep_sources.git
sudo cp ~/git/rosdep_sources/19-janelia.list /etc/ros/rosdep/sources.list.d/19-janelia.list
sudo rosdep init
rosdep update
sudo apt-get install python-wstool
mkdir -p ~/lavis_ws/src
cd ~/lavis_ws/src
git clone https://github.com/JaneliaSciComp/lavis.git
cd ..
wstool init src src/lavis/.rosinstall
rosdep install --from-paths . --reinstall
catkin_make
echo "source ~/lavis_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
mkdir ~/bin
ln -s ~/lavis_ws/src/lavis/lavis ~/bin/lavis
echo "alias lavis_key='rosrun zaber_stage key_teleop.py _up_x_rate:=-5 _down_x_rate:=5 _up_y_rate:=0 _down_y_rate:=0 _left_x_rate:=0 _right_x_rate:=0 _left_y_rate:=-5 _right_y_rate:=5'"  >> ~/.bash_aliases
source ~/.profile
```

Add yourself to the group 'dialout' in order to have write permissions on the USB port:

```shell
sudo usermod -aG dialout $USER
sudo reboot
```

rosdep is broken for now. Install some packages manually until it is fixed.

```shell
sudo apt-get install ros-jade-rosbridge-server
sudo apt-get install python-flask
sudo apt-get install ros-jade-web-video-server
```

###Camera Support

```shell
sudo nano /etc/default/grub
```

Replace this line:

```shell
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"
```

with this:

```shell
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=1000"
```

```shell
sudo update-grub
```

```shell
cd ~/lavis_ws/src/lavis/
sudo sh flycap2-conf
sudo reboot
```

###Uninstall Camera Support

```shell
sudo nano /etc/default/grub
```

Replace this line:

```shell
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=1000"
```

with this:

```shell
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"
```

```shell
sudo update-grub
```

```shell
cd ~/lavis_ws/src/lavis/
sh remove_flycapture.sh
```

###Zaber Stage Setup

```shell
ipython
```

```python
from zaber_device import ZaberDevice
dev = ZaberDevice()
dev.restore_settings()
dev.renumber()
dev.disable_potentiometer()
dev.set_serial_number(123)
dev.set_alias(0,10)
dev.set_alias(1,10)
dev.set_alias(2,11)
dev.set_alias(3,11)
```

###Testing Nodes

```shell
rosrun zaber_stage home.py
rosservice call /zaber_stage_node/get_pose
rosservice call /zaber_stage_node/moving
```

```shell
rosrun zaber_stage key_teleop.py _up_x_rate:=-5 _down_x_rate:=5 _up_y_rate:=0 _down_y_rate:=0 _left_x_rate:=0 _right_x_rate:=0 _left_y_rate:=-5 _right_y_rate:=5
```

```shell
rosrun image_view image_view image:=/camera/image_raw _image_transport:=theora
```

```shell
rostopic echo /camera/image_raw | grep -e height -e width
```

```shell
rostopic hz /camera/image_raw
```

```shell
rostopic pub -1 /mightex_controller_node/cmd_current mightex_controller/CmdCurrent -- 1 200
rostopic pub -1 /mightex_controller_node/cmd_off mightex_controller/CmdChannel -- 1
```

```shell
rostopic pub -1 /pyaudio_controller_node/play_tone pyaudio_controller/Tone -- 3000 500
```

```shell
rosrun image_view image_view image:=/camera/blob_out/image_raw
```

```shell
rostopic echo /camera/blobs
```

```shell
rosrun image_view image_view image:=/camera/behavior_out/image_raw
```

```shell
rostopic echo /camera/behaviors
```

```shell
rostopic pub -1 /camera/lavis_stage_controller_node/start std_msgs/Empty
rostopic pub -1  /camera/lavis_stage_controller_node/stop std_msgs/Empty
```

```shell
rostopic pub -1 /camera/lavis_stimuli_controller_node/start std_msgs/Empty
rostopic pub -1  /camera/lavis_stimuli_controller_node/stop std_msgs/Empty
```

```shell
rosrun zaber_stage move_absolute_percent.py _x:=50 _y:=25
```

```shell
rostopic pub -1 /camera/blob_tracker/save_background_image std_msgs/Empty
```

###Run Nodes Independently

```shell
roslaunch zaber_stage zaber_stage.launch x_serial_number:=123 x_alias:=10 x_microstep_size:=0.00049609375 y_serial_number:=123 y_alias:=11 y_microstep_size:=0.00049609375
```

```shell
rosrun pointgrey_camera_driver list_cameras
roslaunch pointgrey_camera_driver camera.launch
```

```shell
roslaunch mightex_controller mightex_controller.launch
```

```shell
roslaunch pyaudio_controller pyaudio_controller.launch
```

```shell
roslaunch lavis_launch first.launch
```

```shell
ROS_NAMESPACE=camera rosrun blob_tracker blob_tracker
```

```shell
ROS_NAMESPACE=camera roslaunch blob_tracker blob_tracker.launch manager:=camera_nodelet_manager
```

```shell`
ROS_NAMESPACE=camera roslaunch larvae_behavior_classifier larvae_behavior_classifier.launch manager:=camera_nodelet_manager
``

