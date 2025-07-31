
cd ~/tiago_public_ws/src
git clone https://github.com/leggedrobotics/darknet_ros.git
cd ../
catkin build darknet_ros -DCMAKE_BUILD_TYPE=Release

catkin build perception_msgs

catkin build object_detection object_detection_world object_labeling \
obstacle_converter pick_place place_pose plane_segmentation \
project_msgs task_manager tiago_localization tiago_moveit_config\

tian jia ignore
jia ru perception msg

catkin build pick
catkin build task_manager
catkin build adv_nav
catkin build tiago_state_machine
catkin build carry_navi
sudo apt install ros-noetic-smach-viewer
rosrun smach_viewer smach_viewer.py

rosrun tiago_gazebo tuck_arm.py 

sudo ntpdate 192.168.1.200

export ROS_MASTER_URI=http://192.168.1.200:11311
export ROS_IP=192.168.1.170

export ROS_IP=10.68.0.129

ping tiago-46c

rosservice call /pal_map_manager/change_map "input: 'adv_map0624'"

roslaunch carry_navi localization.launch