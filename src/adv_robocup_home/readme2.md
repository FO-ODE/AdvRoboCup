catkin build object_detection object_detection_world object_labeling \
obstacle_converter pick_place place_pose plane_segmentation \
project_msgs task_manager tiago_localization tiago_moveit_config


export ROS_MASTER_URI=http://192.168.1.200:11311
export ROS_IP=192.168.1.170

ping tiago-46c

rosservice call /pal_map_manager/change_map "input: 'adv_map0624'"

roslaunch carry_navi localization.launch