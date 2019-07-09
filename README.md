# MiiVii GMSL Camera

MiiVii GMSL Camera demo is an application software running under ROS environment. It depends on MiiVii low level GMSL SDK, and can be run on MiiVii S2 Pro, MiiVii Apex.
It publish fully synchronized image topics, and color format process is accelerated by hardware, which makes the node cost very limited CPU resource and no GPU resource is used. Various camera vendors are supported.


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
Assume we are using only 1 camera in this case.
```
    roslaunch miivii_gmsl_ros 1_node_with_1_camera.launch
```
You can check the image topic in rviz.


### Configuration

MiiVii GMSL Solution provides a configurable synchronize solution, which can be used to configure camera framerate at hardware level.
1. In launch file, change fps parameter and launch. This will change the low level trigger signal, change the camera shutter at hardware level.