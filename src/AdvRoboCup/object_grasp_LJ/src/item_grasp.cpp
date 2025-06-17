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
geometry_msgs::PoseStamped _liftObject;
geometry_msgs::PoseStamped _attachObject;
// std::vector<double> extractPose;

void poseCallback(const visualization_msgs::MarkerArray& msg) 
{
  for(int i = 0; i<msg.markers.size(); i++)
  {
    _goal_pose.header.frame_id = "base_link";
    _goal_pose.pose.position.x = msg.markers[i].pose.position.x - 0.3;
    _goal_pose.pose.position.y = msg.markers[i].pose.position.y;
    _goal_pose.pose.position.z = msg.markers[i].pose.position.z-0.06 ;
    _goal_pose.pose.orientation.w = -0.5;
    _goal_pose.pose.orientation.x = 0.5;
    _goal_pose.pose.orientation.y = 0.5;
    _goal_pose.pose.orientation.z = -0.5;

    _attachObject.header.frame_id = "base_link";
    _attachObject.pose.position.x = msg.markers[i].pose.position.x -0.1;
    _attachObject.pose.position.y = msg.markers[i].pose.position.y;
    _attachObject.pose.position.z = msg.markers[i].pose.position.z-0.06 ;
    _attachObject.pose.orientation.w = -0.5;
    _attachObject.pose.orientation.x = 0.5;
    _attachObject.pose.orientation.y = 0.5;
    _attachObject.pose.orientation.z = -0.5;


    _liftObject.header.frame_id = "base_link";
    _liftObject.pose.position.x = msg.markers[i].pose.position.x ;
    _liftObject.pose.position.y = msg.markers[i].pose.position.y;
    _liftObject.pose.position.z = msg.markers[i].pose.position.z + 0.4 ;
    _liftObject.pose.orientation.w = -0.5;
    _liftObject.pose.orientation.x = 0.5;
    _liftObject.pose.orientation.y = 0.5;
    _liftObject.pose.orientation.z = -0.5;



  }
}


trajectory_msgs::JointTrajectory openGripper()
{
  trajectory_msgs::JointTrajectory posture;
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_left_finger_joint";
  posture.joint_names[1] = "gripper_right_finger_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 1;
  posture.points[0].positions[1] = 1;
  posture.points[0].time_from_start = ros::Duration(1.0);
  
  return posture;
}

trajectory_msgs::JointTrajectory closedGripper()
{ 
  trajectory_msgs::JointTrajectory posture;
  posture.joint_names.resize(2);
  posture.joint_names[0] = "gripper_left_finger_joint";
  posture.joint_names[1] = "gripper_right_finger_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0;
  posture.points[0].positions[1] = 0;
  posture.points[0].time_from_start = ros::Duration(1.0);

  return posture;
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


int main(int argc, char** argv)
{ 
  ros::init(argc, argv, "item_grasp");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Subscriber goalPose_sub = nh.subscribe("/text_markers", 1, poseCallback);
  ros::Publisher gripper_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 1);
  
  trajectory_msgs::JointTrajectory open = openGripper();
  trajectory_msgs::JointTrajectory close = closedGripper();


  system("rosrun pal_gripper_controller_configuration_gazebo home_gripper.py");
  // gripper_pub.publish(open);
   ros::Duration(5.0).sleep();

  ROS_INFO_STREAM("Preparing grasing position.");
  move_arm(_goal_pose);
  ros::Duration(5.0).sleep();
  
  ROS_INFO_STREAM("Moving to object.");
  move_arm(_attachObject);
  ros::Duration(5.0).sleep();
  ROS_INFO_STREAM("goal_pose" << _goal_pose);
  ROS_INFO_STREAM("attachobj" << _attachObject);


  gripper_pub.publish(close);
  ros::Duration(2.0).sleep();
  
  ROS_INFO_STREAM("Lifting the object.");
  move_arm(_liftObject);
  ros::Duration(2.0).sleep();
  
  spinner.stop();
  //  system("rosrun tiago_gazebo tuck_arm.py");
  return EXIT_SUCCESS;
}

