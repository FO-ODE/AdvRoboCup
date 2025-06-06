// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/GraspPlanning.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PickupGoal.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf/transform_broadcaster.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
using namespace std; 

geometry_msgs::PoseStamped _goal_pose;
geometry_msgs::PoseStamped _lift_pose;

void _goal_position()
{
//依据桌子2位置设定的放置物品的位置（基于机器人坐标系 非map）
_goal_pose.header.frame_id = "base_link";
_goal_pose.pose.position.x = 0.58;
_goal_pose.pose.position.y = 0.0;
_goal_pose.pose.position.z = 0.6;
_goal_pose.pose.orientation.w = -0.5;
_goal_pose.pose.orientation.x = 0.5;
_goal_pose.pose.orientation.y = 0.5;
_goal_pose.pose.orientation.z = -0.5;

//放下物体后抬升手臂
_lift_pose.header.frame_id = "base_link";
_lift_pose.pose.position.x = 0.58;
_lift_pose.pose.position.y = 0.0;
_lift_pose.pose.position.z = 0.8;
_lift_pose.pose.orientation.w = -0.5;
_lift_pose.pose.orientation.x = 0.5;
_lift_pose.pose.orientation.y = 0.5;
_lift_pose.pose.orientation.z = -0.5;


}


int move_arm(geometry_msgs::PoseStamped& goal)
{
  moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");

  group_arm_torso.setPlannerId("RRTConnectkConfigDefault");
  group_arm_torso.setPoseReferenceFrame("base_link");
  group_arm_torso.setEndEffectorLink("gripper_link");
  group_arm_torso.setPoseTarget(goal);


  ROS_INFO_STREAM("Planning to move " <<
                  group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                  group_arm_torso.getPlanningFrame());

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  group_arm_torso.setPlanningTime(5.0);
  bool success = bool(group_arm_torso.plan(my_plan));

  if ( !success )
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  ros::Time start = ros::Time::now();

  moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();
  // if (!bool(e))
  //   throw std::runtime_error("Error executing plan");

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
  
  return EXIT_SUCCESS;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_arm");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  _goal_position();
  move_arm(_goal_pose);
  ros::Duration(5.0).sleep();
  //打开爪子
  system("rosrun pal_gripper_controller_configuration_gazebo home_gripper.py");
  ros::Duration(3.0).sleep();
  //抬升手臂
   move_arm(_lift_pose);
    ros::Duration(5.0).sleep();
    //收回手臂
    system("rosrun tiago_gazebo tuck_arm.py");


  spinner.stop();  
  return EXIT_SUCCESS;
}