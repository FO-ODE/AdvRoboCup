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
        self.approach_distance = 0.8  # Distance to object (meters)
        self.robot_height = 0.0  # Robot base height
        
        # Fixed robot orientation quaternion
        self.fixed_orientation = Quaternion(x=0.000, y=0.000, z=0.961, w=-0.277)
        
        # Task completion flag
        self.task_completed = False
        
        rospy.loginfo("Pose Adjuster Node Initialized.")
        rospy.loginfo("Fixed robot orientation: [0.000, 0.000, 0.961, -0.277]")
        rospy.loginfo("Input coordinate frame is fixed to: base_link")
        rospy.loginfo("Waiting for object position point...")
    
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
        """Calculate robot approach pose - based on object position point in map coordinate system"""
        
        # Calculate approach direction from fixed orientation
        # Quaternion [0.000, 0.000, 0.961, -0.277] corresponding yaw angle
        # Use quaternion to Euler angle conversion
        qx, qy, qz, qw = 0.000, 0.000, 0.961, -0.277
        
        # Calculate yaw angle (rotation around z-axis)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        rospy.loginfo(f"Fixed yaw angle: {math.degrees(robot_yaw):.1f} degrees")
        
        # Calculate robot position (in map coordinate system)
        # Robot stands in front of object, at distance approach_distance, facing fixed direction
        approach_offset_x = self.approach_distance * math.cos(robot_yaw)
        approach_offset_y = self.approach_distance * math.sin(robot_yaw)
        
        robot_x = object_position_map.x - approach_offset_x
        robot_y = object_position_map.y - approach_offset_y
        
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
        """Send movement goal and wait for completion"""
        rospy.loginfo(f"Moving to goal: position=({pose_stamped.pose.position.x:.2f}, "
                     f"{pose_stamped.pose.position.y:.2f}), "
                     f"orientation=[{pose_stamped.pose.orientation.x:.3f}, "
                     f"{pose_stamped.pose.orientation.y:.3f}, "
                     f"{pose_stamped.pose.orientation.z:.3f}, "
                     f"{pose_stamped.pose.orientation.w:.3f}], "
                     f"frame={pose_stamped.header.frame_id}")
        
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