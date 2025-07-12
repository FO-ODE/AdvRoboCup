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
        
        # 创建tf2监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo("TF2 Listener initialized.")
        
        # 使用action client
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_client.wait_for_server()
        rospy.loginfo("Move_base action server connected!")
        
        # 订阅物体位置点数据
        rospy.Subscriber('/adv_robocup/object_position', PointStamped, self.object_callback)
        
        # 存储物体位置数据
        self.object_point = None
        
        # 参数配置
        self.approach_distance = 0.8  # 距离物体的距离（米）
        self.robot_height = 0.0  # 机器人基座高度
        
        # 固定的机器人朝向四元数
        self.fixed_orientation = Quaternion(x=0.000, y=0.000, z=0.961, w=-0.277)
        
        # 任务完成标志
        self.task_completed = False
        
        rospy.loginfo("Pose Adjuster Node Initialized.")
        rospy.loginfo("Fixed robot orientation: [0.000, 0.000, 0.961, -0.277]")
        rospy.loginfo("Input coordinate frame is fixed to: base_link")
        rospy.loginfo("Waiting for object position point...")
    
    def object_callback(self, msg):
        """接收物体位置点回调"""
        if self.task_completed:
            return
        
        # 检查坐标系是否为base_link
        if msg.header.frame_id != "base_link":
            rospy.logwarn(f"Expected frame_id 'base_link', but received '{msg.header.frame_id}'. Ignoring message.")
            return
        
        self.object_point = msg
        rospy.loginfo(f"Received object position: ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f}) in base_link frame")
        self.process_point()
    
    def transform_point_to_map(self, point_stamped):
        """将PointStamped从base_link转换到map坐标系"""
        try:
            # 等待变换可用
            self.tf_buffer.can_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            
            # 执行坐标变换
            point_in_map = self.tf_buffer.transform(point_stamped, "map", rospy.Duration(1.0))
            
            rospy.loginfo(f"Point transformed from base_link to map: "
                         f"({point_in_map.point.x:.2f}, {point_in_map.point.y:.2f}, {point_in_map.point.z:.2f})")
            
            return point_in_map
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Failed to transform point from base_link to map: {e}")
            return None
    
    def calculate_approach_pose(self, object_position_map):
        """计算机器人的接近位姿 - 基于map坐标系中的物体位置点"""
        
        # 从固定朝向计算接近方向
        # 四元数 [0.000, 0.000, 0.961, -0.277] 对应的yaw角度
        # 使用四元数到欧拉角的转换
        qx, qy, qz, qw = 0.000, 0.000, 0.961, -0.277
        
        # 计算yaw角度（绕z轴旋转）
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        rospy.loginfo(f"Fixed yaw angle: {math.degrees(robot_yaw):.1f} degrees")
        
        # 计算机器人应该站立的位置（在map坐标系中）
        # 机器人在物体前方，距离为approach_distance，面向固定方向
        approach_offset_x = self.approach_distance * math.cos(robot_yaw)
        approach_offset_y = self.approach_distance * math.sin(robot_yaw)
        
        robot_x = object_position_map.x - approach_offset_x
        robot_y = object_position_map.y - approach_offset_y
        
        return robot_x, robot_y
    
    def process_point(self):
        """处理位置点数据并生成目标位姿"""
        if self.object_point is None or self.task_completed:
            return
        
        try:
            # 先将物体位置从base_link转换到map坐标系
            object_point_map = self.transform_point_to_map(self.object_point)
            
            if object_point_map is None:
                rospy.logerr("Failed to transform object position to map frame")
                return
            
            object_position_map = object_point_map.point
            
            rospy.loginfo(f"Object position in map frame: ({object_position_map.x:.2f}, {object_position_map.y:.2f}, {object_position_map.z:.2f})")
            
            # 基于map坐标系中的物体位置计算机器人接近位姿
            robot_x, robot_y = self.calculate_approach_pose(object_position_map)
            
            rospy.loginfo(f"Robot target position in map frame: ({robot_x:.2f}, {robot_y:.2f})")
            
            # 创建目标位姿（已在map坐标系中）
            target_pose = PoseStamped()
            target_pose.header.frame_id = "map"
            target_pose.header.stamp = rospy.Time.now()
            
            # 设置位置
            target_pose.pose.position.x = robot_x
            target_pose.pose.position.y = robot_y
            target_pose.pose.position.z = self.robot_height
            
            # 设置固定朝向
            target_pose.pose.orientation = self.fixed_orientation
            
            # 发送移动目标
            self.move_to_goal(target_pose)
            
        except Exception as e:
            rospy.logerr(f"Error processing point: {e}")
    
    def move_to_goal(self, pose_stamped):
        """发送移动目标并等待完成"""
        rospy.loginfo(f"Moving to goal: position=({pose_stamped.pose.position.x:.2f}, "
                     f"{pose_stamped.pose.position.y:.2f}), "
                     f"orientation=[{pose_stamped.pose.orientation.x:.3f}, "
                     f"{pose_stamped.pose.orientation.y:.3f}, "
                     f"{pose_stamped.pose.orientation.z:.3f}, "
                     f"{pose_stamped.pose.orientation.w:.3f}], "
                     f"frame={pose_stamped.header.frame_id}")
        
        # 创建MoveBaseGoal
        goal = MoveBaseGoal()
        goal.target_pose = pose_stamped
        
        # 发送目标
        self.move_client.send_goal(goal)
        rospy.loginfo("Goal sent. Waiting for robot to reach the target...")
        
        # 等待移动完成
        self.move_client.wait_for_result()
        
        # 检查结果
        result = self.move_client.get_result()
        state = self.move_client.get_state()
        
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("SUCCESS: Robot reached the target position!")
            self.task_completed = True
            rospy.loginfo("Task completed. Shutting down node...")
            rospy.signal_shutdown("Task completed successfully")
        elif state == actionlib.GoalStatus.ABORTED:
            rospy.logwarn("ABORTED: Failed to reach the target position")
            self.terminate_with_failure()
        elif state == actionlib.GoalStatus.REJECTED:
            rospy.logwarn("REJECTED: Goal was rejected by move_base")
            self.terminate_with_failure()
        else:
            rospy.logwarn(f"Movement ended with state: {state}")
            self.terminate_with_failure()
    
    def terminate_with_failure(self):
        """任务失败时的终止处理"""
        rospy.logerr("Task failed. Shutting down node...")
        self.task_completed = True
        rospy.signal_shutdown("Task failed")
    
    def set_approach_distance(self, distance):
        """设置接近距离"""
        self.approach_distance = distance
        rospy.loginfo(f"Approach distance set to {distance:.2f}m")

def main():
    try:
        adjuster = PoseAdjuster()
        
        # 可以通过ROS参数设置接近距离
        approach_dist = rospy.get_param('~approach_distance', 0.8)
        adjuster.set_approach_distance(approach_dist)
        
        rospy.loginfo("Pose adjuster ready. Publish point to:")
        rospy.loginfo("  - /adv_robocup/object_position (物体位置点)")
        rospy.loginfo("Only accepts PointStamped messages with frame_id 'base_link'")
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