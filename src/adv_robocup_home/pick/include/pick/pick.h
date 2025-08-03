#ifndef PICK_H
#define PICK_H

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <project_msgs/LabeledCentroid.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>

class Pick
{
  private:
    moveit::planning_interface::MoveGroupInterface arm_torso_group;
    moveit::planning_interface::MoveGroupInterface gripper_group;

  public:
    Pick():arm_torso_group("arm_torso"), gripper_group("gripper") {};

  public:
    bool init();
    void update();

  /* Robot Motion */
  public:
    void pick();
  
  private:
    void lowerTorso();
    void prePickApproach();
    void openGripper();
    void toPickPose();
    void closeGripper();
    void higherTorso();
    void wristRotate();
    void postPickRetreat();
    void toTransportPose();
    bool grasp_command_;           // Flag indicating a grasp command
    bool target_received_;         // Flag indicating the target position is received

  private:
    void setWorkspace();

  /* ROS Communication */
  public:
    ros::NodeHandle nh_;

  private:
    ros::Subscriber pick_target_sub_;   // Subscriber for target object position, stored locally
    ros::Subscriber grasp_command_sub_;
    ros::Publisher gripper_pub_;
    ros::Publisher torso_pub_;
    ros::Publisher pick_done_pub_;
    ros::Publisher label_pick_pub_;

    // Added service server and its callback function
    ros::ServiceServer pick_service_;
    bool pickServiceCallback(std_srvs::Trigger::Request &req,
                             std_srvs::Trigger::Response &res);
  private:
    std::string pick_comm_topic_;
    std::string pick_target_topic_;
    std::string pick_done_topic_;

    trajectory_msgs::JointTrajectory gripper_close_value_;

  private:
    // void poseCallback(const project_msgs::LabeledCentroid::ConstPtr &msg);
    void poseCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
    int object_label_;
    void graspCommandCallback(const std_msgs::Bool::ConstPtr &msg);

  /* Local Variables */
  private:
    // For ROS
    bool command_;
    std_msgs::Bool pick_done_;

    geometry_msgs::PoseStamped labeled_centroid_;

    // Helper variables
    std::string ref_frame_;
    Eigen::Vector3d target_position_;
    std::vector<double> transport_value_;
    geometry_msgs::PoseStamped pre_approach_pose_;
    geometry_msgs::Pose pick_pose_;
    geometry_msgs::Pose retreat_pose_;
    geometry_msgs::Pose transport_pose_;

  /* MoveIt */
  private:
    moveit::planning_interface::MoveGroupInterface::Plan arm_plan_;
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan_;
    moveit::planning_interface::PlanningSceneInterface PSI_;
    std::vector<std::string> object_names_;
};

#endif
