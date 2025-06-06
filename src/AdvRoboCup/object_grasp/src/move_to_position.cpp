#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string>
#include <vector>
#include <map>

#include <tf/transform_broadcaster.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <geometry_msgs/PoseStamped.h>



void body_move_to_position(const geometry_msgs::PoseStamped& goal)
{
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    ROS_INFO("Waiting for the move_base action server to start...");
    ac.waitForServer(ros::Duration(5.0));

    ROS_INFO("Sending goal position...");
    move_base_msgs::MoveBaseGoal goal_msg;
    goal_msg.target_pose = goal;
    ac.sendGoal(goal_msg);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Robot successfully reached the goal position!");
    }
    else
    {
        ROS_ERROR("Failed to reach the goal position.");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_to_position");
    ros::NodeHandle nh;
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = atof(argv[1]);
    goal_pose.pose.position.y = atof(argv[2]);
    goal_pose.pose.position.z = atof(argv[3]);
    goal_pose.pose.orientation.x =  atof(argv[4]);
    goal_pose.pose.orientation.y =  0.0;
    goal_pose.pose.orientation.z =  0.0;  
    goal_pose.pose.orientation.w =  atof(argv[5]);


// // 
//     geometry_msgs::PoseStamped goal_pose;
//     goal_pose.header.frame_id = "map";
//     goal_pose.pose.position.x = -2.56;
//     goal_pose.pose.position.y = 0.175;
//     goal_pose.pose.position.z = 0.618;
//     goal_pose.pose.orientation.z = 0.0;
//     goal_pose.pose.orientation.w = 1.0;

    ros::Duration(1.0).sleep();
    body_move_to_position(goal_pose);
    system("rosrun play_motion move_joint head_2_joint -0.8 2.0");
    return 0;
}
