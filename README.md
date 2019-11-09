# MiiVii GMSL Camera

MiiVii GMSL Camera demo is an application software running under ROS environment. It depends on MiiVii low level GMSL SDK, and can be run on MiiVii S2 Pro, MiiVii Apex.
It publish fully synchronized image topics, and color format process is accelerated by hardware, which makes the node cost very limited CPU resource. Also, various camera vendors are supported.

Platform:

| Platform      | Main Chip     | Appearance     |
| ---------- | :-----------:  | :-----------:  |
| S2  Pro     |   TX2    |<img src="images/s2pro.png" width="100">|
| Apex     | Xavier | <img src="http://www.miivii.com/en/img/prodcut1.png" width="100">      |


## MiiVii GMSL ROS User Guide

The MiiVii GMSL Camera contains only one ROS package miivii_gmsl_ros for now.

### Compile
Clone the repository and build:
```
    mkdir -p ~/catkin_ws/src & cd ~/catkin_ws/src
    git clone https://github.com/MiiViiDynamics/miivii_gmsl_camera
    cd ../..
    catkin_make_isolated
    source devel_isolated/setup.bash
```

### Run ROS Demo and Display Camera by rviz
Open 2 terminals, launch miivii_gmsl_ros, rviz.

#### 1 camera connected
```
    roslaunch miivii_gmsl_ros 1_node_with_1_camera.launch
```

#### 4 camera connected
```
    roslaunch miivii_gmsl_ros 1_node_with_4_cameras.launch
```

You can check the image topic in rviz.


### Configuration

MiiVii GMSL Solution provides a configurable synchronize solution, which can be used to configure camera framerate at hardware level.
1. In launch file, change fps parameter and launch. This will change the low level trigger signal, change the camera shutter at hardware level.

## Contact
For technology issue, please file bugs on github directly.
For busniess contact, you can either visit our [taobao shop](https://shop324175547.taobao.com/?spm=a230r.7195193.1997079397.2.3154636cYGG7Vj)
, or mail to bd#miivii.com
