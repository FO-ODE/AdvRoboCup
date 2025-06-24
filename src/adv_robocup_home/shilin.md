export ROS_MASTER_URI=http://192.168.1.200:11311
export ROS_IP=192.168.1.226

catkin build adv_nav carry_navi
source devel/setup.bash

rosrun adv_nav head_scan.py
rostopic pub /head_scan_command std_msgs/String "data: 'stop'"

rosservice call /pal_map_manager/save_map "directory: '~/home/tiago_public_ws/src/adv_robocup_home/resource/maps/adv_map0624'"

rosservice call /pal_map_manager/change_map "input: 'adv_map0624'"

rosrun rviz rviz -d "$(rospack find pmb2_2dnav)/config/rviz/advanced_navigation.rviz"

scp -r /home/zsl/ros/AdvRoboCup/src/adv_robocup_home/resources/maps/adv_map0624 pal@192.168.1.200:/home/pal/.pal/tiago_maps/configurations/

base link的坐标
rosrun tf tf_echo map base_link

Translation: [1.963, 2.124, 0.099]
Rotation: in Quaternion [0.000, 0.000, 0.980, -0.201]
            in RPY (radian) [0.000, -0.000, -2.737]
            in RPY (degree) [0.000, -0.000, -156.798]