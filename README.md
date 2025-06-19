# Adv RoboCup Cheat Sheet

## Overview

## Contents

- [TIAGo Robot](#tiago-robot)
- [Containers](#containers)
- [Useful Commands](#useful-commands)
- [TBD](#tbd)

## TIAGo Robot

### TIAGo Handbook

<https://docs.pal-robotics.com/TIAGo-base/handbook.html>

### Connection with TIAGo Robot

**TIAGo Webmanager**: <http://192.168.1.200:8080>

```bash
# to determine the IP address of the development computer
ifconfig

# Ethernet
export ROS_MASTER_URI=http://192.168.1.200:11311 \
export ROS_IP=10.68.0.131

# WLAN
export ROS_MASTER_URI=http://192.168.1.200:11311 \
export ROS_IP=192.168.1.84
```

### Connection Test

```bash
ping TIAGo-46c
ping 192.168.1.200

ssh pal@TIAGo-46c

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

for TIAGo in real world

When starting this container, please **disconnect from the internet**. Reconnect to the internet only after the container has successfully started.

```bash
rocker --nvidia --x11 --privileged \
    --volume /home/zby/ros/RoboCup_workspace/src/adv_robocup_home:/TIAGo_public_ws/src/adv_robocup_home \
    --network host \
    --name TIAGo_container \
    foode258/TIAGo_yolo:fpmoveit10

# commandes below are not required if you don't use MoveIt
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update
sudo apt install ros-noetic-moveit-commander
```

for TIAGo in simulation environment

```bash
rocker --nvidia --x11 --privileged \
    --volume /home/zby/ros/RoboCup_workspace/src/adv_robocup_home:/TIAGo_public_ws/src/adv_robocup_home \
    --network host \
    --name TIAGo_simulation \
    foode258/TIAGo_yolo:fptest11.2
```

for object detection

```bash
rocker --nvidia --x11 --privileged \
    --volume /home/zby/ros/workspace \
    --network host \
    --name yolo_container \
    foode258/yolo_ros:fptest1
```

for Advanced Object Detection

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

roslaunch TIAGo_moveit_config moveit_rviz.launch config:=true
```

### TIAGo Navigation & Map

```bash
rosrun map_server map_server src/carry_my_luggage/ics_map/map.yaml

rosrun rviz rviz -d'rospack find TIAGo_2dnav'/config/rviz/navigation.rviz

rosservice call /pal_map_manager/change_map "input: '2025-01-30_124458'"

**TIAGo Webmanager**: <http://192.168.1.200:8080>

```bash
# to asure the IP
ifconfig

# Ethernet
export ROS_MASTER_URI=http://192.168.1.200:11311 \
export ROS_IP=10.68.0.131

# WLAN
```

## TBD

**Commands below will be deleted!!!**

not tested, some of them will then be deleted

in `yolo_container`

```bash
roslaunch object_detection object_detection.launch
```

in `TIAGo_container`

```bash
roslaunch carry_navi grasp_drop_simulation.launch

roslaunch TIAGo_moveit_config move_group.launch planning_frame:=base_footprint end_effector:=pal-gripper

rosrun grasping insert_grasp_object.py 

rosrun grasping table_collision_publisher.py

rosrun grasping grasp.py

roslaunch carry_moveit item_grasp.launch

roslaunch carry_moveit grasp_vertical.launch
```

arm prepare

```bash
rostopic pub /arm_controller/command trajectory_msgs/JointTrajectory "header:
    seq: 0
    stamp:
    secs: 0
    nsecs: 0
    frame_id: ''   
joint_names:
    - 'arm_1_joint'
    - 'arm_2_joint'
    - 'arm_3_joint'
    - 'arm_4_joint'
    - 'arm_5_joint'
    - 'arm_6_joint'
    - 'arm_7_joint'
points:
    - positions: [1.5, -1.5, -3.0, 2.5, -1.6, -1.0, 0.0] 
    time_from_start: {secs: 1, nsecs: 0}"
```

head control

```bash
rostopic pub /head_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- 'head_1_joint'
- 'head_2_joint'
points:
- positions: [0.0, -1.0] 
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 1, nsecs: 0}"
```

base control

tiago_move_base

rostopic echo /mobile_base_controller/cmd_vel

```bash
rostopic pub /mobile_base_controller/cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -2.0" 
```

torso

```bash
rostopic pub /torso_controller/increment/goal teleop_tools_msgs/IncrementActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  increment_by:
  - 0.3"
```

speaking

```bash
rostopic echo /tts/goal
rostopic pub /tts/goal pal_interaction_msgs/TtsActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  text:
    section: ''
    key: ''
    lang_id: ''
    arguments:
    - section: ''
      key: ''
      expanded: ''
  rawtext:
    text: 'test'
    lang_id: 'en_GB'
  speakerName: ''
  wait_before_speaking: 0.0" 
  ```

SSH, MAP

 ```bash
rosservice call /pal_map_manager/change_map "input: '2025-01-30_124458'"
```

useful command:

```bash
rosrun rviz rviz -d'rospack find tiago_2dnav'/config/rviz/navigation.rviz


roslaunch tiago_moveit_config moveit_rviz.launch config:=true
rosservice call /parallel_gripper_controller/grasp
roslaunch carry_moveit arm_prepare.launch
rosrun carry_moveit arm_torso 0.2 -0.5 1.4 0 -1.57 0
rosrun carry_moveit arm 0.2 -0.5 1.4 0 -1.57 0
rosrun tiago_moveit_tutorial plan_arm_torso_ik 0.2 -0.5 1.4 0 -1.57 0
rosrun tiago_moveit_tutorial plan_arm_torso_ik 0.2 -0.5 1.2 0 -1.57 0

rosrun map_server map_server src/carry_my_luggage/ics_map/map.yaml
rosrun rviz rviz -d'rospack find tiago_2dnav'/config/rviz/navigation.rviz

```
