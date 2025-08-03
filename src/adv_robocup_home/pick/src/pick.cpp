#include <pick/pick.h>
#include <std_srvs/Trigger.h>

bool Pick::init()
{
  pick_target_topic_ = "/adv_robocup/object_position";
  pick_done_topic_ = "/pick_done";

  // Initialize subscriber and publishers
  pick_target_sub_ = nh_.subscribe(pick_target_topic_, 1, &Pick::poseCallback, this);
  gripper_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 1);
  torso_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/torso_controller/command", 1);
  pick_done_pub_ = nh_.advertise<std_msgs::Bool>(pick_done_topic_, 1);
  label_pick_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/label_pick", 1);

  // Register service
  pick_service_ = nh_.advertiseService("/adv_robocup/pick_service", &Pick::pickServiceCallback, this);

  command_ = false;
  pick_done_.data = true;
  grasp_command_ = false;

  // Transport pose
  transport_value_ = {0.35, 0.1, -0.3, -0.3, 2.269, -1.57, 0.45, 0.2};

  gripper_close_value_.joint_names = {"gripper_left_finger_joint", "gripper_right_finger_joint"};
  gripper_close_value_.points.resize(1);
  gripper_close_value_.points[0].positions = {0, 0};
  gripper_close_value_.points[0].time_from_start = ros::Duration(1.0);

  ref_frame_ = "base_footprint";
  arm_torso_group.setPlannerId("RRTConnectkConfigDefault");
  arm_torso_group.setPlanningTime(30.0);
  arm_torso_group.setMaxAccelerationScalingFactor(0.3);
  arm_torso_group.setMaxVelocityScalingFactor(0.3);
  arm_torso_group.setEndEffectorLink("arm_tool_link");

  openGripper();
  return true;
}

bool Pick::pickServiceCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  if (!command_)
  {
    ROS_WARN("[PickService] Called but command_ is false");
    res.success = false;
    res.message = "No object position received yet.";
    return true;
  }

  grasp_command_ = true;  // ✅ Set service trigger
  ROS_INFO("[PickService] Pick command received. Will be processed in update().");

  // Set flag, do not execute pick directly (main thread will handle it)
  res.success = true;
  res.message = "Pick command set. Waiting for update loop to handle it.";
  return true;
}

void Pick::update()
{
  if (command_ && grasp_command_)
  {
    ROS_INFO("[Pick::update] Executing pick sequence...");
    // Wait for 2 sec to update the planning scene
    ros::Duration(2.0).sleep();
    
    pick();

    pick_done_pub_.publish(pick_done_);
    label_pick_pub_.publish(labeled_centroid_);

    // Clear all collision objects in the planning scene
    object_names_ = PSI_.getKnownObjectNames();
    ROS_INFO_STREAM("Number of collision objects in the scene: " << object_names_.size());
    PSI_.removeCollisionObjects(object_names_);
    object_names_.clear();

    command_ = false;
    grasp_command_ = false;
  }
}

// ---------------- Pick Pipeline ----------------

void Pick::pick()
{
  ROS_INFO("[Pick] Lowering torso");
  lowerTorso();

  ROS_INFO("[Pick] Opening gripper");
  openGripper();

  ROS_INFO("[Pick] Approaching pre-pick pose");
  prePickApproach();

  ROS_INFO("[Pick] Moving to pick pose");
  toPickPose();

  ROS_INFO("[Pick] Closing gripper");
  closeGripper();

  ROS_INFO("[Pick] Post-pick retreat");
  postPickRetreat();

  ROS_INFO("[Pick] To transport pose");
  toTransportPose();
}

void Pick::lowerTorso()
{
  // Lower the torso to reach the object
  trajectory_msgs::JointTrajectory torso_lower_value;
  torso_lower_value.joint_names.resize(1);
  torso_lower_value.joint_names[0] = "torso_lift_joint";
  torso_lower_value.points.resize(1);
  torso_lower_value.points[0].positions.resize(1);
  torso_lower_value.points[0].positions[0] = 0.1;
  torso_lower_value.points[0].time_from_start = ros::Duration(1.0);

  torso_pub_.publish(torso_lower_value);
  ros::Duration(5.0).sleep();
}

void Pick::prePickApproach()
{
  // Move to pre-pick approach position
  arm_torso_group.setPoseTarget(pre_approach_pose_);
  bool succ = (arm_torso_group.plan(arm_plan_) == moveit_msgs::MoveItErrorCodes::SUCCESS);

  if (!succ)
  {
    ROS_INFO_STREAM("Planning failed");
  }
  arm_torso_group.move();
  ROS_INFO_STREAM("Pre-pick goal reached");
}

void Pick::openGripper()
{
  // Open the gripper before picking
  std::vector<double> open_value = {0.044, 0.044};
  gripper_group.setJointValueTarget(open_value);
  gripper_group.plan(gripper_plan_);
  gripper_group.move();
  ROS_INFO_STREAM("Gripper opened");
}

void Pick::toPickPose()
{
  // Move the end effector to the pick position
  pick_pose_ = pre_approach_pose_.pose;
  pick_pose_.position.x += 0.26;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(pick_pose_);

  moveit_msgs::RobotTrajectory trajectory;

  double eef_step = 0.01; 
  double jump_threshold = 0.0;  

  double fraction = arm_torso_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  int trajectory_points = trajectory.joint_trajectory.points.size();

  ROS_INFO_STREAM("Number of points in the trajectory = " << trajectory_points);

  arm_torso_group.execute(trajectory);

  geometry_msgs::PoseStamped pick_pose;
  pick_pose = arm_torso_group.getCurrentPose("arm_tool_link");
  ROS_INFO_STREAM("Arrived at: " << pick_pose.pose.position);
}

void Pick::closeGripper()
{
  gripper_pub_.publish(gripper_close_value_);
  ros::Duration(1.0).sleep();
}

void Pick::wristRotate()
{
  // Get current joint status (contains torso_lift_joint + arm_1_joint ~ arm_7_joint)
  std::vector<double> current_joints = arm_torso_group.getCurrentJointValues();

  // Safety check: verify the arm_torso_group has 8 joints
  if(current_joints.size() < 8)
  {
    ROS_WARN("Expected 8 joints in arm_torso_group, got %zu. Cannot rotate wrist!", current_joints.size());
    return;
  }

  double original_angle = current_joints[7]; // arm_7_joint is usually the wrist roll joint
  double rotate_offset = 0.7;                

  current_joints[7] = original_angle + rotate_offset;
  arm_torso_group.setJointValueTarget(current_joints);
  if (arm_torso_group.plan(arm_plan_) == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    arm_torso_group.move();
    ros::Duration(3.0).sleep(); 
  }
  else
  {
    ROS_WARN("Wrist rotate: plan to +1 rad failed");
    return;
  }

  // Rotate back to the original angle
  current_joints[7] = original_angle;
  arm_torso_group.setJointValueTarget(current_joints);
  if (arm_torso_group.plan(arm_plan_) == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    arm_torso_group.move();
    ros::Duration(1.0).sleep(); 
  }
  else
  {
    ROS_WARN("Wrist rotate: plan back to original failed");
  }
}

void Pick::higherTorso()
{
  trajectory_msgs::JointTrajectory torso_higher_value;

  torso_higher_value.joint_names.resize(1);
  torso_higher_value.joint_names[0] = "torso_lift_joint";
  torso_higher_value.points.resize(1);
  torso_higher_value.points[0].positions.resize(1);
  torso_higher_value.points[0].positions[0] = 0.35;
  torso_higher_value.points[0].time_from_start = ros::Duration(1.0);

  torso_pub_.publish(torso_higher_value);
  ros::Duration(4.0).sleep();
}

void Pick::postPickRetreat()
{
  retreat_pose_ = pick_pose_;
  retreat_pose_.position.z += 0.05;

  geometry_msgs::Pose retreat_pose_1;
  retreat_pose_1 = retreat_pose_;
  retreat_pose_1.position.x -= 0.28;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(retreat_pose_);
  waypoints.push_back(retreat_pose_1);

  moveit_msgs::RobotTrajectory trajectory;

  double eef_step = 0.01;  
  double jump_threshold = 0.0;  

  double fraction = arm_torso_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  arm_torso_group.execute(trajectory);

  ROS_INFO_STREAM("Retreated");
}

void Pick::toTransportPose()
{
  arm_torso_group.setJointValueTarget(transport_value_);
  arm_torso_group.plan(arm_plan_);
  arm_torso_group.move();

  ROS_INFO_STREAM("Ready to go");
}

// ---------------- Callback ----------------

void Pick::poseCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  command_ = true;

  target_position_ << msg->point.x, msg->point.y, msg->point.z;

  pre_approach_pose_.header.frame_id = ref_frame_;
  pre_approach_pose_.pose.position = msg->point;
  pre_approach_pose_.pose.position.x -= 0.415;
  pre_approach_pose_.pose.position.z += 0.01;

  tf2::Quaternion q;
  q.setRPY(1.57, 0.0, 0.0);
  pre_approach_pose_.pose.orientation = tf2::toMsg(q);

  ROS_INFO_STREAM("Target received: " << target_position_.transpose());
}
