    rocker --nvidia --x11 --privileged \
        --volume /home/troy/adv_robocup/AdvRoboCup/src/adv_robocup_home:/tiago_public_ws/src/adv_robocup_home \
        --network host \
        --name tiago_container \
        foode258/tiago_robot:env1.0

export ROS_MASTER_URI=http://192.168.1.200:11311
export ROS_IP=192.168.1.170

sudo ntpdate 192.168.1.200

catkin build pick
catkin build adv_nav
catkin build tiago_state_machine
catkin build carry_navi

rosservice call /pal_map_manager/change_map "input: 'adv_map0624'"

roslaunch carry_navi localization.launch

roslaunch pick pick.launch

roslaunch tiago_state_machine test.launch



# Launch a Docker container for the TIAGO robot environment
rocker --nvidia --x11 --privileged \
    --volume /home/troy/adv_robocup/AdvRoboCup/src/adv_robocup_home:/tiago_public_ws/src/adv_robocup_home \
    --network host \
    --name tiago_container \
    foode258/tiago_robot:env1.0

    rocker: Runs a container with GUI and NVIDIA GPU support.

    --nvidia: Enables GPU acceleration inside the container.

    --x11: Allows GUI forwarding (Rviz, Gazebo).

    --privileged: Grants the container full privileges (needed for some ROS drivers).

    --volume: Mounts your local adv_robocup_home into the container workspace.

    --network host: Shares host networking, required for ROS communication.

    --name tiago_container: Names the container for easy reference.

    foode258/tiago_robot:env1.0: The Docker image to use.

# Set ROS network environment variables
export ROS_MASTER_URI=http://192.168.1.200:11311
export ROS_IP=192.168.1.170

    ROS_MASTER_URI: Specifies the ROS master (robot or PC running roscore).

    ROS_IP: Sets your local machine IP so ROS nodes can communicate properly.

# Sync local time with the robot (important for TF and ROS message timestamps)
sudo ntpdate 192.168.1.200

    This synchronizes your PC time with the robot (at IP 192.168.1.200).

# Build individual ROS packages
catkin build pick
catkin build adv_nav
catkin build tiago_state_machine
catkin build carry_navi

    catkin build: Compiles specific packages instead of the whole workspace.

    pick, adv_nav, tiago_state_machine, carry_navi: ROS packages used in your project.

# Change the map used by the robot
rosservice call /pal_map_manager/change_map "input: 'adv_map0624'"

    Calls a service to switch to the map adv_map0624.

    Used for navigation localization in the environment.

# Launch localization node
roslaunch carry_navi localization.launch

    Starts the localization process (e.g., AMCL) for the TIAGO robot using the specified map.

# Launch the pick node
roslaunch pick pick.launch

    Runs the pick package launch file.

    Usually starts MoveIt! and the picking node to control the robot's arm.

# Launch the state machine
roslaunch tiago_state_machine test.launch

    Starts the SMACH-based state machine to execute the full task sequence.