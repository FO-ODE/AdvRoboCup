# Adv RoboCup Cheat Sheet

## Containerzz

for real world tiago, **offline start**

    rocker --nvidia --x11 --privileged \
        --volume /home/troy/adv_robocup/AdvRoboCup/src/adv_robocup_home:/tiago_public_ws/src/adv_robocup_home \
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
        --volume /home/troy/adv_robocup/AdvRoboCup/src/adv_robocup_home:/tiago_public_ws/src/adv_robocup_home \
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
        


🔍 具体对比：rocker vs docker run
功能/行为	docker run	rocker
👁️ GUI (X11显示支持)	❌ 默认不行，需要手动设置 DISPLAY、.Xauthority、挂载 socket	✅ 自动处理，带 --x11 一键搞定
🎮 GPU支持 (NVIDIA)	需要手动加 --gpus all 或 --runtime=nvidia	✅ 加 --nvidia 即启用 GPU
🔐 权限处理 (--privileged, /dev, udev)	需要自己加一堆 --device、权限配置	✅ 默认处理好，适配 ROS 硬件设备（如摄像头）
📡 网络配置	默认桥接网络，--network host 需手动指定	✅ 自动支持 host 网络
🐳 命令复杂度	比较冗长繁琐，容易出错	✅ 更简洁，特别适合新手或做仿真
📦 额外功能	无	支持 --reuse、--env, --volume, --user 等封装功能
💡 举个例子：
使用 docker run 启动一个 ROS + GUI 容器需要：

xhost +local:docker
docker run -it \
  --gpus all \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume ~/.Xauthority:/root/.Xauthority \
  --privileged \
  --network host \
  --name tiago_container \
  foode258/tiago_yolo:fpmoveit10

🧠 很麻烦，对吧？
而用 rocker：

rocker --nvidia --x11 --privileged \
  --name tiago_container \
  --network host \
  foode258/tiago_yolo:fpmoveit10

一条命令，效果一样，还多了防呆和封装逻辑。