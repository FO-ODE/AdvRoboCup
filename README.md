# Adv RoboCup Cheat Sheet

## Container

for real world tiago, **offline start**

    rocker --nvidia --x11 --privileged \
        --volume /home/zby/ros/RoboCup_workspace/src/adv_robocup_home:/tiago_public_ws/src/adv_robocup_home \
        --network host \
        --name tiago_container \
        foode258/tiago_yolo:fpmoveit10


    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'

    sudo apt install curl -y
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

    sudo apt update
    sudo apt install ros-noetic-moveit-commander

for simulation environment

    rocker --nvidia --x11 --privileged \
        --volume /home/zby/ros/RoboCup_workspace/src/adv_robocup_home:/tiago_public_ws/src/adv_robocup_home \
        --network host \
        --name tiago_simulation \
        foode258/tiago_yolo:fptest11.2

for object detection

    rocker --nvidia --x11 --privileged \
        --volume /home/zby/ros/workspace \
        --network host \
        --name yolo_container \
        foode258/yolo_ros:fptest1

## Procedure

in `yolo_image`

    roslaunch object_detection object_detection.launch

in `core_image`

    roslaunch carry_navi grasp_drop_simulation.launch

    roslaunch tiago_moveit_config move_group.launch planning_frame:=base_footprint end_effector:=pal-gripper

    rosrun grasping insert_grasp_object.py 

    rosrun grasping table_collision_publisher.py

    rosrun grasping grasp.py

    roslaunch carry_moveit item_grasp.launch

    roslaunch carry_moveit grasp_vertical.launch

arm prepare

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
        
