#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/String.h>
#include <play_motion_msgs/PlayMotionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <thread>

class GraspExecutor {
public:
  GraspExecutor() : 
    move_group_("arm_torso"),
    tf_listener_(tf_buffer_),
    planning_scene_interface_() {

    ros::NodeHandle nh;

    gripper_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 10);
    base_pub_ = nh.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 10);
    point_sub_ = nh.subscribe("/object_center", 1, &GraspExecutor::poseCallback, this);

    move_group_.setPlannerId("RRTkConfigDefault");
    move_group_.setPoseReferenceFrame("base_footprint");
    move_group_.setEndEffectorLink("gripper_link");
    move_group_.setMaxVelocityScalingFactor(0.5);
    move_group_.setPlanningTime(10.0);
  }

  void poseCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = msg->header.frame_id;

    pose.pose.position.x = msg->point.x - 0.3;
    pose.pose.position.y = msg->point.y + 0.02;
    pose.pose.position.z = msg->point.z + 0.003;
    pose.pose.orientation.x = 0.5;
    pose.pose.orientation.y = 0.5;
    pose.pose.orientation.z = -0.5;
    pose.pose.orientation.w = -0.5;
    goal_pose_ = pose;

    attach_pose_ = pose;
    attach_pose_.pose.position.x = msg->point.x - 0.15;

    lift_pose_ = attach_pose_;
    lift_pose_.pose.position.z += 0.2;
  }

  trajectory_msgs::JointTrajectory createGripperTrajectory(double position) {
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names = {"gripper_left_finger_joint", "gripper_right_finger_joint"};
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = {position, position};
    point.time_from_start = ros::Duration(1.0);
    traj.points.push_back(point);
    return traj;
  }

  bool moveArm(const geometry_msgs::PoseStamped& target_pose) {
    move_group_.setPoseTarget(target_pose);
    move_group_.setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (!success) {
      ROS_ERROR("No plan found to target pose");
      return false;
    }

    move_group_.move();
    return true;
  }

  bool moveCartesianTo(const geometry_msgs::PoseStamped& target_pose) {
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose start_pose = move_group_.getCurrentPose().pose;

    geometry_msgs::Pose target = target_pose.pose;
    target.orientation = start_pose.orientation; // 保持当前朝向

    waypoints.push_back(start_pose);
    waypoints.push_back(target);

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group_.computeCartesianPath(waypoints, 0.02, 1.0, trajectory);

    if (fraction < 0.9) {
      ROS_WARN("Only %.1f%% of the Cartesian path was planned", fraction * 100.0);
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    move_group_.execute(plan);
    return true;
  }

  void execute() {
    ros::Duration(3.0).sleep(); // 等待订阅和场景初始化

    auto open_gripper = createGripperTrajectory(0.045);
    auto close_gripper = createGripperTrajectory(0.0);

    gripper_pub_.publish(open_gripper);
    ros::Duration(1.0).sleep();

    ROS_INFO("Moving to pre-grasp pose...");
    moveArm(goal_pose_);
    ros::Duration(1.0).sleep();

    planning_scene_interface_.removeCollisionObjects({"grasp_target"});
    ros::Duration(2.0).sleep();

    ROS_INFO("Approaching object...");
    moveCartesianTo(attach_pose_);
    ros::Duration(1.0).sleep();

    gripper_pub_.publish(close_gripper);
    ros::Duration(1.0).sleep();

    planning_scene_interface_.removeCollisionObjects({"top_obstacle"});
    ros::Duration(2.0).sleep();

    ROS_INFO("Lifting object...");
    moveCartesianTo(lift_pose_);
    ros::Duration(3.0).sleep();

    ROS_INFO("Tucking arm...");
    system("rosrun tiago_gazebo tuck_arm.py");
  }

private:
  ros::Publisher gripper_pub_;
  ros::Publisher base_pub_;
  ros::Subscriber point_sub_;

  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::PoseStamped goal_pose_;
  geometry_msgs::PoseStamped attach_pose_;
  geometry_msgs::PoseStamped lift_pose_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "grasp_executor_node");
  GraspExecutor executor;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  executor.execute();
  ros::waitForShutdown();
  return 0;
}
