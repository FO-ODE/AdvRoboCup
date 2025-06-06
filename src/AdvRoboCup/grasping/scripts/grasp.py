import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PointStamped, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState
import os
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_commander import PlanningSceneInterface


class GraspExecutor:
    def __init__(self):
        rospy.init_node("item_grasp_py")
        moveit_commander.roscpp_initialize([])
        self.scene = PlanningSceneInterface(synchronous=True)
        self.obj_id = "grasp_target"

        self.goal_pose = geometry_msgs.msg.PoseStamped()
        self.attachObject = geometry_msgs.msg.PoseStamped()
        self.liftObject = geometry_msgs.msg.PoseStamped()

        self.gripper_pub = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=10)
        self.base_pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/object_center", PointStamped, self.pose_callback)

        self.group = moveit_commander.MoveGroupCommander("arm_torso")
        self.group.set_planner_id("RRTkConfigDefault") # RRTConnectkConfigDefault SBLkConfigDefault
                                                                # rosparam get /move_group/planner_configs
        self.group.set_pose_reference_frame("base_footprint")
        self.group.set_end_effector_link("gripper_link")
        self.group.set_max_velocity_scaling_factor(0.5)
        self.group.set_planning_time(10.0)

    def pose_callback(self, msg: PointStamped):
        self.goal_pose.header.frame_id = msg.header.frame_id
        self.goal_pose.pose.position.x = msg.point.x - 0.3
        self.goal_pose.pose.position.y = msg.point.y + 0.0
        self.goal_pose.pose.position.z = msg.point.z + 0.003
        self.goal_pose.pose.orientation.x = 0.5
        self.goal_pose.pose.orientation.y = 0.5
        self.goal_pose.pose.orientation.z = -0.5
        self.goal_pose.pose.orientation.w = -0.5

        self.attachObject.header.frame_id = msg.header.frame_id
        self.attachObject.pose.position.x = msg.point.x - 0.15
        self.attachObject.pose.position.y = msg.point.y + 0.0
        self.attachObject.pose.position.z = msg.point.z + 0.003
        self.attachObject.pose.orientation = self.goal_pose.pose.orientation

        self.liftObject.header.frame_id = msg.header.frame_id
        self.liftObject.pose.position.x = msg.point.x - 0.15
        self.liftObject.pose.position.y = msg.point.y + 0.0
        self.liftObject.pose.position.z = msg.point.z + 0.2
        self.liftObject.pose.orientation = self.goal_pose.pose.orientation

    def create_gripper_trajectory(self, position):
        traj = JointTrajectory()
        traj.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.time_from_start = rospy.Duration(1)
        traj.points.append(point)
        return traj

    def move_arm(self, target_pose):
        
        
        self.group.set_pose_target(target_pose)
        self.group.set_start_state_to_current_state()
        # rospy.loginfo(f"Using Planner: {self.group.get_planner_id()}")
        # rospy.loginfo(f"Moving to Position: {target_pose.pose.position}")
        # rospy.loginfo(f"In Frame: {target_pose.header.frame_id}")

        rospy.loginfo("Planning motion to target pose...")
        plan = self.group.plan()
        if not plan or not plan[0]:
            os.system("rosrun tiago_gazebo tuck_arm.py")
            rospy.sleep(1.0)
            raise RuntimeError("No plan found")

        rospy.loginfo("Executing motion...")
        result = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        return result
    
    def move_cartesian_to(self, target_pose):
        # 1. 当前末端位姿
        start_pose = self.group.get_current_pose().pose

        # 2. 目标位姿（保持当前朝向）
        target = geometry_msgs.msg.Pose()
        target.position = target_pose.pose.position
        target.orientation = start_pose.orientation

        # 3. 组装路点
        waypoints = [start_pose, target]

        rospy.loginfo("Planning Cartesian path to attachObject...")

        # 4. 只给前两个必填位置参数
        plan, fraction = self.group.compute_cartesian_path(
            waypoints,
            0.01,  # eef_step
            0.0    # jump_threshold
        )

        if fraction < 0.9:
            rospy.logwarn(f"Only {fraction*100:.1f}% of the path was planned!")
            raise RuntimeError("Cartesian path planning failed")

        rospy.loginfo("Executing Cartesian path...")
        self.group.execute(plan, wait=True)
        self.group.stop()




    def execute(self):
        rospy.sleep(3.0) # waiting for initialization, must have
        open_gripper = self.create_gripper_trajectory(0.045)
        close_gripper = self.create_gripper_trajectory(0.0)
        
        rospy.logwarn(f"Actual planner: {self.group.get_planner_id()}")
        rospy.loginfo(f"Grasp Pose: {self.goal_pose}")
        rospy.loginfo(f"Attach Pose: {self.attachObject}")

        rospy.loginfo("Opening gripper")
        self.gripper_pub.publish(open_gripper)
        rospy.sleep(1.0)

        rospy.loginfo("Preparing grasping position")
        self.move_arm(self.goal_pose)
        rospy.sleep(1.0)
        
        self.scene.remove_world_object(self.obj_id)
        rospy.logwarn(f"Removed object '{self.obj_id}' from Planning Scene")
        rospy.sleep(2.0)

        rospy.loginfo("Approaching object")
        # self.move_arm(self.attachObject)
        self.move_cartesian_to(self.attachObject)
        rospy.sleep(1.0)
        
        rospy.loginfo("Closing gripper")
        self.gripper_pub.publish(close_gripper)
        rospy.sleep(1.0)
        
        self.scene.remove_world_object("top_obstacle")
        rospy.logwarn(f"Removed object top_obstacle from Planning Scene")
        rospy.sleep(2.0)

        rospy.loginfo("Lifting object")
        self.move_arm(self.liftObject)
        rospy.sleep(3.0)

        rospy.loginfo("Tuck arm")
        # self.tuck_arm()
        os.system("rosrun tiago_gazebo tuck_arm.py")
        rospy.sleep(1.0)

        rospy.loginfo("Grasp sequence finished")
        rospy.sleep(1.0)

if __name__ == "__main__":
    executor = GraspExecutor()
    executor.execute()
