#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, Point
from tf.transformations import quaternion_from_euler
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class PoseAdjuster:
    def __init__(self):
        rospy.init_node('pose_adjuster', anonymous=True)
        
        # Create tf2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo("TF2 Listener initialized.")
        
        # Use action client
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_client.wait_for_server()
        rospy.loginfo("Move_base action server connected!")
        
        # Subscribe to object position point data
        rospy.Subscriber('/adv_robocup/object_position', PointStamped, self.object_callback)
        
        # Store object position data
        self.object_point = None
        
        # Parameter configuration
        self.approach_distance = 0.8  # Distance to object (meters) - 安全距离
        self.robot_height = 0.0  # Robot base height
        
        # Fixed robot orientation quaternion
        self.fixed_orientation = Quaternion(x=0.000, y=0.000, z=0.961, w=-0.277)
        
        # Task completion flag
        self.task_completed = False
        
        # 添加精确调整参数
        self.fine_tune_enabled = True
        self.max_fine_tune_attempts = 3
        self.fine_tune_threshold = 0.15  # 15cm tolerance
        self.current_attempt = 0
        
        # 视觉反馈订阅器
        rospy.Subscriber('/adv_robocup/object_position', PointStamped, self.current_object_callback)
        self.current_object_point = None
        
        rospy.loginfo("Pose Adjuster Node Initialized.")
        rospy.loginfo("Fixed robot orientation: [0.000, 0.000, 0.961, -0.277]")
        rospy.loginfo("Input coordinate frame is fixed to: base_link")
        rospy.loginfo("Waiting for object position point...")
        rospy.loginfo("Fine-tuning enabled with visual feedback")
        rospy.loginfo("Robot will position behind the target point along the target orientation")
    
    def object_callback(self, msg):
        """Receive object position point callback"""
        if self.task_completed:
            return
        
        # 如果不是 base_link 坐标系，先转换到 base_link
        if msg.header.frame_id != "base_link":
            rospy.loginfo(f"Received object position in '{msg.header.frame_id}' frame. Converting to base_link...")
            transformed_msg = self.transform_point_to_base_link(msg)
            if transformed_msg is None:
                rospy.logwarn(f"Failed to transform point from '{msg.header.frame_id}' to base_link. Ignoring message.")
                return
            self.object_point = transformed_msg
        else:
            self.object_point = msg
        
        rospy.loginfo(f"Object position in base_link frame: ({self.object_point.point.x:.2f}, {self.object_point.point.y:.2f}, {self.object_point.point.z:.2f})")
        self.process_point()
    
    def current_object_callback(self, msg):
        """当前物体位置回调（用于精确调整）"""
        if not self.fine_tune_enabled or self.task_completed:
            return
        self.current_object_point = msg
    
    def transform_point_to_base_link(self, point_stamped):
        """Transform PointStamped from any coordinate system to base_link"""
        try:
            # 检查变换是否可用
            if not self.tf_buffer.can_transform("base_link", point_stamped.header.frame_id, 
                                               point_stamped.header.stamp, rospy.Duration(1.0)):
                rospy.logwarn(f"Transform from '{point_stamped.header.frame_id}' to 'base_link' not available")
                return None
            
            # 执行坐标变换
            point_in_base_link = self.tf_buffer.transform(point_stamped, "base_link", rospy.Duration(1.0))
            
            rospy.loginfo(f"Point transformed from '{point_stamped.header.frame_id}' to base_link: "
                         f"({point_in_base_link.point.x:.2f}, {point_in_base_link.point.y:.2f}, {point_in_base_link.point.z:.2f})")
            
            return point_in_base_link
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Failed to transform point from '{point_stamped.header.frame_id}' to base_link: {e}")
            return None
    
    def transform_point_to_map(self, point_stamped):
        """Transform PointStamped from base_link to map coordinate system"""
        try:
            # Wait for transform to be available
            self.tf_buffer.can_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            
            # Execute coordinate transformation
            point_in_map = self.tf_buffer.transform(point_stamped, "map", rospy.Duration(1.0))
            
            rospy.loginfo(f"Point transformed from base_link to map: "
                         f"({point_in_map.point.x:.2f}, {point_in_map.point.y:.2f}, {point_in_map.point.z:.2f})")
            
            return point_in_map
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Failed to transform point from base_link to map: {e}")
            return None
    
    def calculate_approach_pose(self, object_position_map):
        """Calculate robot approach pose - 机器人位于目标点沿着设定朝向反方向的安全距离处"""
        
        # Calculate approach direction from fixed orientation
        # Quaternion [0.000, 0.000, 0.961, -0.277] corresponding yaw angle
        qx, qy, qz, qw = 0.000, 0.000, 0.961, -0.277
        
        # Calculate yaw angle (rotation around z-axis) - 机器人的目标朝向
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        target_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        rospy.loginfo(f"Target yaw angle: {math.degrees(target_yaw):.1f} degrees")
        
        # 计算机器人位置：目标点沿着机器人朝向的反方向移动安全距离
        # 机器人朝向的反方向就是朝向角度 + 180度
        retreat_direction = target_yaw + math.pi
        
        # 计算机器人位置偏移量（沿着朝向反方向）
        retreat_offset_x = self.approach_distance * math.cos(retreat_direction)
        retreat_offset_y = self.approach_distance * math.sin(retreat_direction)
        
        # 机器人最终位置 = 目标点位置 + 后退偏移量
        robot_x = object_position_map.x + retreat_offset_x
        robot_y = object_position_map.y + retreat_offset_y
        
        rospy.loginfo(f"Robot positioned behind target point by {self.approach_distance}m")
        rospy.loginfo(f"Retreat direction: {math.degrees(retreat_direction):.1f} degrees")
        
        return robot_x, robot_y
    
    def process_point(self):
        """Process position point data and generate target pose"""
        if self.object_point is None or self.task_completed:
            return
        
        try:
            # First transform object position from base_link to map coordinate system
            object_point_map = self.transform_point_to_map(self.object_point)
            
            if object_point_map is None:
                rospy.logerr("Failed to transform object position to map frame")
                return
            
            object_position_map = object_point_map.point
            
            rospy.loginfo(f"Object position in map frame: ({object_position_map.x:.2f}, {object_position_map.y:.2f}, {object_position_map.z:.2f})")
            
            # Calculate robot approach pose based on object position in map coordinate system
            robot_x, robot_y = self.calculate_approach_pose(object_position_map)
            
            rospy.loginfo(f"Robot target position in map frame: ({robot_x:.2f}, {robot_y:.2f})")
            
            # Create target pose (already in map coordinate system)
            target_pose = PoseStamped()
            target_pose.header.frame_id = "map"
            target_pose.header.stamp = rospy.Time.now()
            
            # Set position
            target_pose.pose.position.x = robot_x
            target_pose.pose.position.y = robot_y
            target_pose.pose.position.z = self.robot_height
            
            # Set fixed orientation
            target_pose.pose.orientation = self.fixed_orientation
            
            # Send movement goal
            self.move_to_goal(target_pose)
            
        except Exception as e:
            rospy.logerr(f"Error processing point: {e}")
    
    def move_to_goal(self, pose_stamped):
        """Send movement goal and wait for completion，添加精确调整"""
        rospy.loginfo(f"Moving to goal: position=({pose_stamped.pose.position.x:.2f}, "
                     f"{pose_stamped.pose.position.y:.2f})")
        
        # Create MoveBaseGoal
        goal = MoveBaseGoal()
        goal.target_pose = pose_stamped
        
        # Send goal
        self.move_client.send_goal(goal)
        rospy.loginfo("Goal sent. Waiting for robot to reach the target...")
        
        # Wait for movement completion
        self.move_client.wait_for_result()
        
        # Check result
        result = self.move_client.get_result()
        state = self.move_client.get_state()
        
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Initial navigation completed!")
            
            # 执行精确调整
            if self.fine_tune_enabled and self.current_attempt < self.max_fine_tune_attempts:
                if self.perform_fine_adjustment():
                    return
            
            rospy.loginfo("SUCCESS: Robot reached the target position!")
            self.task_completed = True
            rospy.loginfo("Task completed. Shutting down node...")
            rospy.signal_shutdown("Task completed successfully")
        else:
            self.handle_navigation_failure(state)
    
    def perform_fine_adjustment(self):
        """执行精确调整 - 修改为基于新的定位方式"""
        rospy.loginfo("Starting fine adjustment...")
        self.current_attempt += 1
        
        # 等待当前物体位置更新
        rospy.sleep(10.0)
        
        if self.current_object_point is None:
            rospy.logwarn("No current object position available for fine adjustment")
            return True  # 跳过精确调整
        
        # 转换到base_link坐标系
        if self.current_object_point.header.frame_id != "base_link":
            current_obj = self.transform_point_to_base_link(self.current_object_point)
        else:
            current_obj = self.current_object_point
        
        if current_obj is None:
            rospy.logwarn("Failed to transform current object position")
            return True
        
        # 检查当前偏差
        obj_x, obj_y = current_obj.point.x, current_obj.point.y
        
        # 计算期望的物体位置：
        # 如果机器人正确定位，物体应该在机器人前方approach_distance处
        # 考虑到机器人的朝向，物体应该在机器人朝向方向上
        expected_x = self.approach_distance  # 物体在机器人前方
        expected_y = 0.0  # 物体在机器人正前方
        
        # 计算偏差
        error_x = obj_x - expected_x
        error_y = obj_y - expected_y
        error_magnitude = math.sqrt(error_x**2 + error_y**2)
        
        rospy.loginfo(f"Fine adjustment attempt {self.current_attempt}/{self.max_fine_tune_attempts}")
        rospy.loginfo(f"Current object position: ({obj_x:.3f}, {obj_y:.3f})")
        rospy.loginfo(f"Expected position (in front of robot): ({expected_x:.3f}, {expected_y:.3f})")
        rospy.loginfo(f"Position error: ({error_x:.3f}, {error_y:.3f}), magnitude: {error_magnitude:.3f}")
        
        if error_magnitude < self.fine_tune_threshold:
            rospy.loginfo("Position is within tolerance. Fine adjustment complete!")
            return True
        
        # 计算调整量（保守调整，避免过度修正）
        adjustment_factor = 0.8  # 80%的误差修正
        adj_x = -error_x * adjustment_factor
        adj_y = -error_y * adjustment_factor
        
        rospy.loginfo(f"Applying adjustment: ({adj_x:.3f}, {adj_y:.3f})")
        
        # 创建调整目标（相对于当前位置）
        adjustment_pose = PoseStamped()
        adjustment_pose.header.frame_id = "base_link"
        adjustment_pose.header.stamp = rospy.Time.now()
        adjustment_pose.pose.position.x = adj_x
        adjustment_pose.pose.position.y = adj_y
        adjustment_pose.pose.position.z = 0.0
        adjustment_pose.pose.orientation = Quaternion(0, 0, 0, 1)  # 不改变朝向
        
        # 转换到map坐标系
        adjustment_pose_map = self.transform_pose_to_map(adjustment_pose)
        if adjustment_pose_map is None:
            rospy.logwarn("Failed to transform adjustment pose to map")
            return True
        
        # 发送调整目标
        goal = MoveBaseGoal()
        goal.target_pose = adjustment_pose_map
        
        self.move_client.send_goal(goal)
        self.move_client.wait_for_result()
        
        state = self.move_client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Fine adjustment movement completed")
            # 递归调用进行下一次检查
            return self.perform_fine_adjustment()
        else:
            rospy.logwarn(f"Fine adjustment failed with state: {state}")
            return True
    
    def transform_pose_to_map(self, pose_stamped):
        """将PoseStamped从base_link转换到map坐标系"""
        try:
            self.tf_buffer.can_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            pose_in_map = self.tf_buffer.transform(pose_stamped, "map", rospy.Duration(1.0))
            return pose_in_map
        except Exception as e:
            rospy.logerr(f"Failed to transform pose to map: {e}")
            return None
    
    def handle_navigation_failure(self, state):
        """Handle navigation failure and take appropriate action"""
        if state == actionlib.GoalStatus.ABORTED:
            rospy.logwarn("ABORTED: Failed to reach the target position")
            self.terminate_with_failure()
        elif state == actionlib.GoalStatus.REJECTED:
            rospy.logwarn("REJECTED: Goal was rejected by move_base")
            self.terminate_with_failure()
        else:
            rospy.logwarn(f"Movement ended with state: {state}")
            self.terminate_with_failure()
    
    def terminate_with_failure(self):
        """Handle task failure termination"""
        rospy.logerr("Task failed. Shutting down node...")
        self.task_completed = True
        rospy.signal_shutdown("Task failed")
    
    def set_approach_distance(self, distance):
        """Set approach distance"""
        self.approach_distance = distance
        rospy.loginfo(f"Approach distance set to {distance:.2f}m")

def main():
    try:
        adjuster = PoseAdjuster()
        
        # Set approach distance via ROS parameter
        approach_dist = rospy.get_param('~approach_distance', 0.8)
        adjuster.set_approach_distance(approach_dist)
        
        rospy.loginfo("Pose adjuster ready. Publish point to:")
        rospy.loginfo("  - /adv_robocup/object_position (object position point)")
        rospy.loginfo("Robot will position behind the target point along the target orientation")
        rospy.loginfo("Robot orientation is fixed to [0.000, 0.000, 0.961, -0.277]")
        rospy.loginfo("Node will terminate after reaching the target position")
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Pose Adjuster Node terminated.")
    except KeyboardInterrupt:
        rospy.loginfo("Pose Adjuster Node interrupted by user.")
    except Exception as e:
        rospy.logerr(f"Error in Pose Adjuster: {e}")

if __name__ == '__main__':
    main()