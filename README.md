# Adv RoboCup Cheat Sheet

## Overview

## Contents

- [TIAGo Robot](#tiago-robot)
- [Containers](#containers)
- [Useful Commands](#useful-commands)
- [TBD](#tbd)

## TIAGo Robot

### TIAGo Handbook

<https://docs.pal-robotics.com/tiago-base/handbook.html>

### Connection with TIAGo Robot

**TIAGo Webmanager**: <http://192.168.1.200:8080>

Use `ifconfig` to determine the IP address of the development computer

```bash
ifconfig
```

Connect the TIAGo Robot with **Ethernet**

```bash
export ROS_MASTER_URI=http://192.168.1.200:11311 \
export ROS_IP=10.68.0.131   # use your own IP
```

Connect the TIAGo Robot with **WLAN**

You need to Register TIAGo as a host first

```bash
sudo gedit /etc/hosts
# Add 192.168.1.200 tiago-46c to the file
```

```bash
export ROS_MASTER_URI=http://192.168.1.200:11311 \
export ROS_IP=192.168.1.84    # use your own IP
```

### Connection Test

```bash
ping tiago-46c
ping 192.168.1.200

ssh pal@tiago-46c

rostopic pub /test_topic std_msgs/String "data: 'test'"
rostopic echo /test_topic
```

### Timer Synchronization

```bash
# set your time zone first

# sudo apt install ntpdate
sudo ntpdate 192.168.1.200

# check the date
watch -n 0.1 date
```

## Containers

### To download the image

```bash
docker pull foode258/tiago_robot:env1.0
```

for TIAGo in real world

When starting this container, please **disconnect from the internet**. Reconnect to the internet only after the container has successfully started.

```bash
rocker --nvidia --x11 --privileged \
    --volume /home/zby/ros/RoboCup_workspace/src/adv_robocup_home:/tiago_public_ws/src/adv_robocup_home \
    --network host \
    --name tiago_container \
    foode258/tiago_robot:env1.0
```

for TIAGo in simulation environment

```bash
rocker --nvidia --x11 --privileged \
    --volume /home/zby/ros/RoboCup_workspace/src/adv_robocup_home:/tiago_public_ws/src/adv_robocup_home \
    --network host \
    --name tiago_simulation \
    foode258/tiago_yolo:fptest11.2
```

for object detection

```bash
rocker --nvidia --x11 --privileged \
    --volume /home/zby/ros/workspace \
    --network host \
    --name yolo_container \
    foode258/yolo_ros:fptest1
```

for Advanced Perception

```bash
rocker --nvidia --x11 --privileged \
    --volume /home/zby/ros/FP_workspace/src:/catkin_ws/src \
    --network host \
    --name ultralytics_container \
    foode258/ultralytics_ros_base:env1.3

rocker --nvidia --x11 --privileged \
    --volume /home/zby/ros/FP_workspace/src:/catkin_ws/src \
    --network host \
    --name clip_container \
    foode258/clip_ros_base:env1.0
```

## Useful Commands

### Linux & ROS

```bash
sudo kill <pid>

rqt

rosbag play -l <filename.bag>

rosnode cleanup

rosrun smach_viewer smach_viewer.py

rosservice list

roslaunch tiago_moveit_config moveit_rviz.launch config:=true
```

### TIAGo MoveIt Visualization

```bash
roslaunch tiago_moveit_config moveit_rviz.launch config:=true
```

### TIAGo Navigation & Map

```bash
rosrun map_server map_server src/carry_my_luggage/ics_map/map.yaml

rosrun rviz rviz -d'rospack find tiago_2dnav'/config/rviz/navigation.rviz

rosservice call /pal_map_manager/change_map "input: 'map_file_name'"
```

## TBD

**Commands below will be deleted!!!**

not tested, some of them will then be deleted

in `yolo_container`

```bash
roslaunch object_detection object_detection.launch
```

in `tiago_container`

```bash
roslaunch carry_navi grasp_drop_simulation.launch

roslaunch tiago_moveit_config move_group.launch planning_frame:=base_footprint end_effector:=pal-gripper

rosrun grasping insert_grasp_object.py 

rosrun grasping table_collision_publisher.py

rosrun grasping grasp.py
```
